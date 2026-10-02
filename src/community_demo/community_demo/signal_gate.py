"""Release only after seeing a non-green phase and a fresh green transition."""


class FreshGreenGate:
    def __init__(self):
        self.reset()

    def reset(self):
        self.non_green_seen = False
        self.non_green_streak = 0
        self.green_streak = 0

    def observe(self, colour):
        if colour in ('red', 'yellow'):
            self.non_green_streak += 1
            self.non_green_seen |= self.non_green_streak >= 2
            self.green_streak = 0
        elif colour == 'green':
            self.non_green_streak = 0
            self.green_streak = self.green_streak + 1 if self.non_green_seen else 0
        else:
            self.non_green_streak = self.green_streak = 0

    def invalidate(self):
        # A sensor outage can hide a complete phase change. Observe it anew.
        self.reset()

    @property
    def ready(self):
        return self.non_green_seen and self.green_streak >= 5
