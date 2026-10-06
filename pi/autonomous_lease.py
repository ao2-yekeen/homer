"""Bridge-local lease, independently tested without ROS or serial hardware."""
import math

class AutonomousLease:
    def __init__(self, timeout=.25):
        if not 0 < timeout <= .25:
            raise ValueError('lease must be in (0, 250 ms]')
        self.timeout = timeout
        self.enabled = False
        self.expired = False
        self.deadline = 0.0

    def mode(self, enabled, now):
        if enabled and not self.enabled:
            self.expired = False
            self.deadline = now + self.timeout
        self.enabled = enabled

    def accept(self, stamp, wall_now, now, linear, angular):
        if not self.enabled or self.expired or not all(math.isfinite(v) for v in (stamp, wall_now, linear, angular)):
            return False
        age = wall_now - stamp
        if not 0 <= age < self.timeout or max(abs(linear), abs(angular)) > 5:
            return False
        # A buffered command gets only its remaining lifetime.
        self.deadline = now + self.timeout - age
        return True

    def tick(self, now):
        if self.enabled and now >= self.deadline - 1e-9:
            self.expired = True
        return self.enabled and self.expired
