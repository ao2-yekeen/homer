"""Single-owner approach replay and direct Xbox joystick control for collection."""
import json
import gzip
import math
import time
from pathlib import Path


class XboxJoystick:
    """Poll current kernel state, including disconnect checks; no ROS writer."""
    def __init__(self, path):
        from evdev import InputDevice, ecodes
        self.ecodes = ecodes
        self.device = InputDevice(str(path))
        self.codes = [ecodes.ABS_X, ecodes.ABS_Y, ecodes.ABS_RX,
                      ecodes.ABS_RY, ecodes.ABS_HAT0X, ecodes.ABS_HAT0Y]
        capabilities = self.device.capabilities()
        buttons = capabilities.get(ecodes.EV_KEY, [])
        axes = dict(capabilities.get(ecodes.EV_ABS, []))
        if (not all(code in axes for code in self.codes)
                or not all(code in buttons for code in (ecodes.BTN_TR, ecodes.BTN_EAST))):
            self.device.close()
            raise RuntimeError('Expected Xbox sticks, D-pad, R1 and B button')
        self.name = self.device.name

    def snapshot(self):
        # ioctl reads detect a disconnected device even with no pending events.
        keys = self.device.active_keys()
        axes = []
        for code in self.codes:
            info = self.device.absinfo(code)
            if info.max <= info.min:
                raise RuntimeError('Invalid joystick axis range')
            value = 2 * (info.value - info.min) / (info.max - info.min) - 1
            # Match /joy conventions: left/up positive, right/down negative.
            axes.append(max(-1., min(1., -value)))
        return axes, self.ecodes.BTN_TR in keys, self.ecodes.BTN_EAST in keys

    def close(self):
        self.device.close()


def read_limits(packet, cfg, checked):
    limits = []
    for sid in cfg['servo_ids']:
        mode = checked(packet.read1ByteTxRx(sid, 33))[0]
        low = checked(packet.read2ByteTxRx(sid, 9))[0]
        high = checked(packet.read2ByteTxRx(sid, 11))[0]
        if mode != 0 or not 0 <= low < high <= 4095:
            raise RuntimeError(f'Servo {sid}: unsupported mode or travel limits')
        limits.append((low, high))
    return limits


def validate_ticks(ticks, limits):
    if (not isinstance(ticks, list) or len(ticks) != len(limits)
            or any(type(v) is not int or not low <= v <= high
                   for v, (low, high) in zip(ticks, limits))):
        raise RuntimeError('Trajectory position outside servo travel limits')


def load_prefix(directory, until_frame, cfg, limits, start_pose):
    directory = Path(directory)
    metadata = json.loads((directory / 'metadata.json').read_text())
    if metadata.get('outcome') != 'success':
        raise RuntimeError('Replay requires a successful episode')
    recorded_cfg = metadata.get('config', {})
    if (recorded_cfg.get('joint_names') != cfg['joint_names']
            or recorded_cfg.get('servo_ids') != cfg['servo_ids']):
        raise RuntimeError('Replay joint order or servo IDs do not match this rig')
    if until_frame is None:
        until_frame = metadata.get('approach_end_frame')
    if type(until_frame) is not int or until_frame < 1:
        raise RuntimeError('Mark the approach with p when teaching, or supply --until-frame N')
    rows = []
    path = directory / 'samples.jsonl.gz'
    stream = gzip.open(path, 'rt') if path.exists() else (directory / 'samples.jsonl').open()
    with stream:
        for index, line in enumerate(stream):
            if index > until_frame:
                break
            rows.append(json.loads(line))
    if len(rows) != until_frame + 1:
        raise RuntimeError('Handoff frame is outside the source episode')
    positions, seconds = [], []
    first_ns = rows[0]['read_end_ns']
    previous_ns = None
    for row in rows:
        ticks = row['state_ticks']
        validate_ticks(ticks, limits)
        stamp = row['read_end_ns']
        if type(stamp) is not int or (previous_ns is not None and stamp <= previous_ns):
            raise RuntimeError('Replay timestamps must increase')
        positions.append(ticks)
        seconds.append((stamp - first_ns) / 1e9)
        previous_ns = stamp
    if any(abs(a - b) > 30 for a, b in zip(positions[0], start_pose)):
        raise RuntimeError('Replay begins at a different session start pose')
    # Reject corrupted/discontinuous source trajectories; replay can slow further.
    for i in range(1, len(positions)):
        dt = seconds[i] - seconds[i - 1]
        # Check source integrity per sample, independent of how fast it was
        # hand-guided. Replay advances in bounded steps and waits for feedback.
        if dt > 1 or max(abs(a - b) for a, b in zip(positions[i], positions[i-1])) > 250:
            raise RuntimeError('Replay has a capture gap or implausible joint step')
    return dict(source=str(directory.resolve()), until_frame=until_frame,
                positions=positions, seconds=seconds)


class AssistedControl:
    """Called by the recorder loop, which is also the sole serial bus owner."""
    AXIS_DEADZONE = .25
    MOTOR_SPEED = 1000
    MOTOR_ACCELERATION = 50
    REPLAY_MAX_STEP_TICKS = 100

    def __init__(self, packet, torque_register, cfg, joystick, limits,
                 checked, set_torque, prefix=None, replay_speed=1.):
        self.packet, self.torque_register, self.cfg = packet, torque_register, cfg
        self.joystick, self.limits = joystick, limits
        self.checked, self.set_torque = checked, set_torque
        self.prefix, self.replay_speed = prefix, replay_speed
        self.enabled = False
        self.phase = 'idle'
        self.target = None
        self.ready = False

    def _write(self, target):
        validate_ticks(target, self.limits)
        for sid, tick in zip(self.cfg['servo_ids'], target):
            self.checked(self.packet.WritePosEx(
                sid, tick, self.MOTOR_SPEED, self.MOTOR_ACCELERATION))
        self.target = list(target)

    def begin(self, ticks, now):
        # Stage live targets before enabling torque, even on a later episode.
        self._write(ticks)
        self.enabled = True  # cleanup must run even if enabling partly fails
        self.set_torque(self.packet, self.torque_register, self.cfg['servo_ids'], True)
        self.phase = 'replay' if self.prefix else 'joystick'
        self.last_time = now
        self.replay_time = 0.
        self.index = 0
        self.settle_started = None
        self.ready = False  # release R1 and centre axes after each handoff
        self.was_held = False
        self.moving_axes = [False] * 6
        self.axis_directions = [0] * 6
        self.joystick_goals = list(map(float, ticks))
        self.progress_time = now
        self.last_axes = [0.] * 6
        print('REPLAYING APPROACH WHILE RECORDING' if self.prefix else 'JOYSTICK: centre sticks and release R1, then hold R1 to move', flush=True)

    def adopt_powered_hold(self, ticks):
        """Take over an already powered arm without toggling torque or old goals."""
        self.enabled = True
        self._write(ticks)

    def update(self, ticks, now):
        axes, held, stop = self.joystick.snapshot() if self.joystick else ([0.] * 6, False, False)
        self.last_axes = axes
        dt = max(0., min(.2, now - self.last_time))
        self.last_time = now
        if stop:
            if self.phase != 'paused':
                self._write(ticks)
                print('B: motion stopped. Discard and start a new demo to resume.', flush=True)
            self.phase = 'paused'
            self.ready = False
            return
        if self.phase in ('idle', 'paused'):
            return
        if self.phase == 'replay':
            positions, times = self.prefix['positions'], self.prefix['seconds']
            candidate_time = min(times[-1], self.replay_time + dt * self.replay_speed)
            index = self.index
            while index + 1 < len(times) and times[index + 1] <= candidate_time:
                index += 1
            if index + 1 < len(times):
                weight = (candidate_time - times[index]) / (times[index+1] - times[index])
                target = [round(a + (b-a)*weight) for a, b in zip(positions[index], positions[index+1])]
            else:
                target = list(positions[-1])
            # First send bounded goals, including the first goal that starts a
            # source movement. Only advance the source clock once feedback is
            # within 30 ticks of that interpolated source pose. Checking this
            # before writing any goal deadlocks when the first source change is
            # larger than 30 ticks: the arm is waiting for a command that is
            # never sent.
            difference = max(abs(a-b) for a, b in zip(target, self.target))
            if difference > self.REPLAY_MAX_STEP_TICKS:
                bounded = [a + round((b-a)*self.REPLAY_MAX_STEP_TICKS/difference)
                           for a, b in zip(self.target, target)]
                self._write(bounded)
                return
            self._write(target)
            if max(abs(a-b) for a, b in zip(target, ticks)) > 30:
                if now - self.progress_time > 20:
                    raise RuntimeError('Replay stalled waiting for measured joint feedback')
                return
            self.progress_time = now
            self.index, self.replay_time = index, candidate_time
            if candidate_time >= times[-1]:
                self.phase = 'settling'
                self.settle_started = now
        if self.phase == 'settling':
            if all(abs(a-b) <= 12 for a, b in zip(ticks, self.target)):
                if self.joystick is None:
                    self.phase = 'holding'
                    print('APPROACH COMPLETE — holding position with torque ON. '
                          'Support the arm, then press h to release torque and finish by hand.', flush=True)
                    return
                self.phase = 'joystick'
                self.ready = False
                print('APPROACH COMPLETE — NOW FINISH THE MOVEMENT: centre sticks and release R1, then hold R1 and use the joystick to complete the alignment/grasp. Recording continues; press f when done.', flush=True)
            elif now - self.settle_started > 20:
                raise RuntimeError('Approach did not settle at the handoff pose')
        if self.phase == 'joystick':
            if not self.ready:
                self.ready = not held and all(abs(v) <= .12 for v in axes)
                return
            if not held:
                # Cancel outstanding motion on dead-man release, retaining torque.
                if self.was_held:
                    self._write([actual if active else goal
                                 for actual, active, goal in
                                 zip(ticks, self.moving_axes, self.target)])
                self.was_held = False
                self.moving_axes = [False] * 6
                self.axis_directions = [0] * 6
                self.joystick_goals = list(map(float, self.target))
                return
            self.was_held = True
            # Only rebase joints the user commands. Rebasing every joint to
            # feedback lets an untouched, loaded joint's hold target drift.
            target = []
            moving_axes = []
            for i, (actual, value, (low, high), goal, was_moving) in enumerate(zip(
                    ticks, axes, self.limits, self.target, self.moving_axes)):
                # The controller reports small axis offsets while buttons are
                # pressed. Treat that as neutral so R1 alone remains a dead-man.
                amount = (0 if abs(value) <= self.AXIS_DEADZONE else
                          math.copysign((abs(value)-self.AXIS_DEADZONE) /
                                        (1-self.AXIS_DEADZONE), value))
                active = amount != 0
                direction = (amount > 0) - (amount < 0)
                moving_axes.append(active)
                if active:
                    # Integrate the commanded position, not sagging feedback.
                    # Rebase on a new/reversed input to cancel any old goal in
                    # the opposite direction. Preserve fractional stick steps.
                    base = (self.joystick_goals[i]
                            if direction == self.axis_directions[i] else actual)
                    self.joystick_goals[i] = max(low, min(high, base + amount * 50 * dt))
                    target.append(round(self.joystick_goals[i]))
                else:
                    target.append(actual if was_moving else goal)
                    self.joystick_goals[i] = float(target[-1])
                self.axis_directions[i] = direction
            # R1 is only the dead-man. Do not send fresh position goals when
            # the sticks and D-pad are centred; pressing R1 alone must not jog.
            if any(moving_axes) or any(self.moving_axes):
                self._write(target)
            self.moving_axes = moving_axes

    def release_for_hand(self):
        self.set_torque(self.packet, self.torque_register, self.cfg['servo_ids'], False)
        self.enabled = False
        self.phase = 'hand'
        print('TORQUE OFF — finish the movement by hand. Recording continues; press f when done.', flush=True)

    def finish(self, ticks):
        if self.enabled:
            self._write(ticks)
        self.phase = 'idle'

    def close(self):
        try:
            if self.enabled:
                self.set_torque(self.packet, self.torque_register, self.cfg['servo_ids'], False)
                self.enabled = False
        finally:
            if self.joystick is not None:
                self.joystick.close()
