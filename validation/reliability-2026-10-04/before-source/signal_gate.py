"""Camera-observed phase history, retained from approach until the stop point."""


class FreshGreenGate:
    def __init__(self):
        self.reset()

    def reset(self):
        self.non_green_seen = False
        self.non_green_streak = 0
        self.green_streak = 0
        self.green_started_at = None
        self.last_observation_at = None

    def observe(self, colour, now=None):
        if now is not None:
            if self.last_observation_at is not None and not 0 <= now - self.last_observation_at <= .7:
                self.invalidate()
            self.last_observation_at = now
        if colour in ('red', 'yellow'):
            self.non_green_streak += 1
            self.non_green_seen = self.non_green_streak >= 2
            self.green_streak = 0
            self.green_started_at = None
        elif colour == 'green':
            self.non_green_streak = 0
            if self.non_green_seen and self.green_started_at is None:
                self.green_started_at = now
            self.green_streak = self.green_streak + 1 if self.non_green_seen else 0
        else:
            # Brief unknown samples must not make an old green look newly started.
            self.non_green_streak = self.green_streak = 0

    def invalidate(self):
        # A sensor outage can hide a complete phase change. Observe it anew.
        self.reset()

    @property
    def ready(self):
        # A delay on approach must not turn remembered early green into a late
        # release. This bounded window is verified with the demo's 15 s green;
        # it is not a guarantee for arbitrary shorter signal cycles.
        early = (self.last_observation_at is None or
                 (self.green_started_at is not None and
                  self.last_observation_at - self.green_started_at <= 4.0))
        return self.non_green_seen and self.green_streak >= 5 and early
