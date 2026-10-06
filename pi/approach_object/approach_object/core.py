"""Fail-closed approach state machine; all times are monotonic seconds."""
from dataclasses import dataclass
import math

ACTIVE = {'SEARCH', 'ALIGN', 'APPROACH'}

@dataclass(frozen=True)
class Observation:
    stamp: float
    sequence: int
    identity: str
    bearing: float
    clearance: float
    view: int
    valid: bool = True
    reason: str = ''

class Controller:
    def __init__(self, config):
        self.config = config
        self.state, self.target, self.reason = 'IDLE', '', ''
        self.clearance = None
        self.view = 0
        self.observation = None
        self.identity = None
        self.confirmations = 0
        self.last_sequence = -1
        self.view_started = 0.0
        self.last_seen = 0.0

    def finish(self, state, reason):
        self.state, self.reason = state, reason
        return (0.0, 0.0)

    def start(self, target, now, authorized):
        if self.state in ACTIVE:
            return False, 'task already active'
        if not isinstance(target, str) or not target.strip():
            return False, 'target must be a nonempty detector class name'
        if not authorized:
            return False, 'operator autonomous mode, link and no override required'
        if not self.config['qualified']:
            return False, self.config.get('movement_blocker', 'REQUIRES_HARDWARE_TEST: calibration/drive qualification incomplete')
        self.__init__(self.config)
        self.target, self.state = target.strip(), 'SEARCH'
        self.view_started = self.last_seen = now
        return True, 'search started'

    def observe(self, obs, now):
        if self.state not in ACTIVE or obs.sequence <= self.last_sequence:
            return
        self.last_sequence = obs.sequence
        if obs.view != self.view or obs.stamp < self.view_started + self.config['settle_s'][self.view]:
            return
        if not obs.valid:
            self.confirmations = 0
            if self.state != 'SEARCH' or obs.reason == 'ambiguous identity':
                self.finish('FAILED', obs.reason or 'invalid depth')
            return
        if not all(math.isfinite(x) for x in (obs.stamp, obs.bearing, obs.clearance)) or not 0 <= now - obs.stamp <= .5:
            self.finish('FAILED', 'stale or invalid perception')
            return
        if self.identity is not None and obs.identity != self.identity:
            self.finish('FAILED', 'ambiguous identity')
            return
        self.identity = obs.identity
        self.observation, self.last_seen, self.clearance = obs, now, obs.clearance
        self.confirmations += 1
        if self.state == 'SEARCH' and self.confirmations >= 3:
            self.state = 'ALIGN'

    def tick(self, now, authorized, path_clear):
        if self.state not in ACTIVE:
            return 0.0, 0.0
        if not authorized:
            return self.finish('CANCELLED', 'teleop takeover, override or link/mode loss')
        if self.state == 'SEARCH':
            if now - self.view_started > self.config['settle_s'][self.view] + self.config['search_s']:
                if self.view == 1:
                    return self.finish('FAILED', 'target absent in both views')
                self.view = 1
                self.view_started = now
                self.identity, self.observation = None, None
                self.confirmations = 0
            return 0.0, 0.0
        obs = self.observation
        if obs is None or not 0 <= now - obs.stamp <= .5:
            return self.finish('FAILED', 'target loss or stale perception')
        if not path_clear:
            return self.finish('FAILED', 'blocked path or stale lidar')
        if obs.clearance <= .30 + self.config['braking_m'] + self.config['uncertainty_m']:
            return self.finish('STOPPED', 'clearance reached; no reversal')
        bearing = obs.bearing
        if self.state == 'ALIGN':
            if abs(bearing) > .06:
                return 0.0, self.config['turn_command'] * (1 if bearing > 0 else -1)
            self.state = 'APPROACH'
        if abs(bearing) > .15:
            return self.finish('FAILED', 'target left approach corridor')
        # Signed commands are measured mappings, not presumed metres per second.
        return self.config['forward_command'], self.config['turn_command'] * max(-1, min(1, bearing / .15))
