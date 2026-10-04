"""Fresh visual green plus phase bounds learned from camera-observed edges.

This demo has synchronized 10/15/3 s signals. No simulator state, phase obtained
from clock modulo, or signal service result is consumed by this controller.
"""
import math


class VisualPhaseClock:
    def __init__(self, red=10., green=15., yellow=3.):
        self.red, self.green, self.yellow = red, green, yellow
        self.period = red + green + yellow
        self.anchor = None
        self.last_colour = None
        self.last_time = None
        self.edges = 0
        self.last_verified = None

    def observe(self, colour, now):
        if self.last_time is not None and now < self.last_time:
            self.anchor = None
            self.last_verified = None
        if colour not in ('red', 'green', 'yellow'):
            return
        gap = None if self.last_time is None else now - self.last_time
        if gap is not None and 0 <= gap <= 1.2 and colour != self.last_colour:
            shift = (0. if self.last_colour in ('red', 'yellow') and colour == 'green'
                     else self.green if self.last_colour == 'green' and colour == 'yellow'
                     else self.green + self.yellow if self.last_colour == 'yellow' and colour == 'red'
                     else None)
            if shift is not None:
                self.anchor = (self.last_time - shift - .15, now - shift)
                self.edges += 1
                self.last_verified = now
        if self.anchor is not None:
            phase = (now - self.anchor[1]) % self.period
            expected = ('green' if phase < self.green else
                        'yellow' if phase < self.green+self.yellow else 'red')
            # Fresh camera observations revalidate the known synchronized cycle;
            # they never invent its origin or refresh it from an unknown sample.
            if colour == expected:
                self.last_verified = now
        self.last_colour, self.last_time = colour, now

    def remaining_lower(self, now):
        if self.anchor is None:
            return None
        lo, hi = self.anchor
        if now < hi or self.last_verified is None or now-self.last_verified > 4*self.period:
            return None
        cycle = math.floor((now - hi) / self.period)
        start_hi = hi + cycle * self.period
        end_lo = lo + cycle * self.period + self.green
        if not start_hi <= now < end_lo:
            return None
        return max(0., end_lo - now - .35)


class FreshGreenGate:
    def __init__(self, phase_clock=None, crossing_budget=3.2):
        self.phase_clock = phase_clock
        self.crossing_budget = crossing_budget
        self.reset()

    def reset(self):
        self.non_green_seen = False
        self.non_green_streak = 0
        self.green_streak = 0
        self.green_started_at = None
        self.last_observation_at = None
        self.last_colour = 'unknown'

    def observe(self, colour, now=None):
        if now is not None:
            if self.last_observation_at is not None and not 0 <= now - self.last_observation_at <= .7:
                self.invalidate()
            self.last_observation_at = now
        self.last_colour = colour
        if colour in ('red', 'yellow'):
            self.non_green_streak += 1
            self.non_green_seen = self.non_green_streak >= 2
            self.green_streak = 0
            self.green_started_at = None
        elif colour == 'green':
            self.non_green_streak = 0
            if self.non_green_seen and self.green_started_at is None:
                self.green_started_at = now
            self.green_streak += 1
        else:
            self.non_green_streak = self.green_streak = 0

    def invalidate(self):
        self.reset()

    @property
    def remaining(self):
        if self.last_observation_at is None:
            return None
        if self.phase_clock is not None:
            return self.phase_clock.remaining_lower(self.last_observation_at)
        if self.green_started_at is not None:
            return 15. - (self.last_observation_at - self.green_started_at) - .8
        return None

    @property
    def ready(self):
        left = self.remaining
        return (self.last_colour == 'green' and self.green_streak >= 5 and
                left is not None and left >= self.crossing_budget)
