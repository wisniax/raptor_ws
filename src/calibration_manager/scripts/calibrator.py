#!/usr/bin/env python3
"""
Interactive terminal tool to send rex_interfaces/msg/CalibrateAxis messages
for stepper motor calibration.

Defaults to direct rclpy publishing.

Use --use-ros2-cli to instead call:
  ros2 topic pub --once /MQTT/CalibrateAxis rex_interfaces/msg/CalibrateAxis "{...}"

Usage:
  python3 calibrator.py [--dry-run] [--interval 0.1] [--use-ros2-cli]

Controls: (shown in the UI too)
  TAB           - cycle modes (offset -> speed -> idle)
  1..4          - select wheel motor (FRONT_LEFT, FRONT_RIGHT, REAR_RIGHT, REAR_LEFT)
  5..0          - select manipulator axis (5=AXIS_1, ..., 0=AXIS_6)
  Up/Down       - adjust step size (offset mode)
  Left/Right    - send offset (offset mode) or adjust speed (speed mode)
  Space         - toggle periodic sending (speed mode)
  R             - return to origin (single)
  O             - set origin / confirm (single)
  S             - stop (single)
  C             - cancel (single)
  q or Ctrl-C   - quit (sends no automatic STOP unless you press S)

Notes:
  - Offsets are sent with action_type = ACTION_TYPE_OFFSET (4)
  - Velocities are sent with action_type = ACTION_TYPE_SET_VELOCITY (5)
  - SetOrigin uses ACTION_TYPE_CONFIRM (2) per your instruction
  - This tool intentionally does not clamp values before sending; you requested that
    clamping be handled on the device side for testing.

"""

import argparse
import curses
import datetime
import subprocess
import threading
import time
from collections import deque

# ---------- Top-level configuration constants ----------
PERIODIC_INTERVAL_DEFAULT = 0.1  # seconds
HISTORY_MAX = 12

# Motor IDs mapping (user confirmed)
ID_FRONT_LEFT_STEPPER = 96    # 0x60
ID_FRONT_RIGHT_STEPPER = 97   # 0x61
ID_REAR_RIGHT_STEPPER = 98    # 0x62
ID_REAR_LEFT_STEPPER = 99     # 0x63
ID_MANIPULATOR_AXIS_1 = 112   # 0x70
ID_MANIPULATOR_AXIS_2 = 113   # 0x71
ID_MANIPULATOR_AXIS_3 = 114   # 0x72
ID_MANIPULATOR_AXIS_4 = 115   # 0x73
ID_MANIPULATOR_AXIS_5 = 116   # 0x74
ID_MANIPULATOR_AXIS_6 = 117   # 0x75
MOTOR_MAP = {
    1: ID_FRONT_LEFT_STEPPER,
    2: ID_FRONT_RIGHT_STEPPER,
    3: ID_REAR_RIGHT_STEPPER,
    4: ID_REAR_LEFT_STEPPER,
    5: ID_MANIPULATOR_AXIS_1,
    6: ID_MANIPULATOR_AXIS_2,
    7: ID_MANIPULATOR_AXIS_3,
    8: ID_MANIPULATOR_AXIS_4,
    9: ID_MANIPULATOR_AXIS_5,
    0: ID_MANIPULATOR_AXIS_6,
}

# Action enums
ACTION_TYPE_STOP = 0
ACTION_TYPE_RETURN_TO_ORIGIN = 1
ACTION_TYPE_CONFIRM = 2  # SetOrigin per user
ACTION_TYPE_CANCEL = 3
ACTION_TYPE_OFFSET = 4
ACTION_TYPE_SET_VELOCITY = 5

# Default UI values
DEFAULT_STEP_DEG = 1.0    # degrees (offset step size)
DEFAULT_SPEED_RPM = 0.0  # RPM
STEP_ADJUST = 0.5         # Up/Down changes step by this amount
SPEED_ADJUST = 200.0        # Left/Right changes speed by this amount

# -------------------------------------------------------


class CalibCLI:
    def __init__(self, use_rclpy=False, dry_run=False, interval=PERIODIC_INTERVAL_DEFAULT):
        self.use_rclpy = use_rclpy
        self.dry_run = dry_run
        self.interval = float(interval)

        # state
        self.mode_cycle = ['offset', 'speed', 'idle']
        self.mode_idx = 0
        self.mode = self.mode_cycle[self.mode_idx]
        self.selected_motor_button = 1
        self.selected_motor = MOTOR_MAP[self.selected_motor_button]
        self.step_deg = DEFAULT_STEP_DEG
        self.speed_rpm = DEFAULT_SPEED_RPM
        self.periodic_on = False
        self.history = deque(maxlen=HISTORY_MAX)

        # periodic worker control
        self._periodic_thread = None
        self._periodic_stop_event = threading.Event()

        # optional rclpy publisher (lazy init)
        self.rclpy_ok = False
        self.rclpy_node = None
        self.rclpy_pub = None
        if self.use_rclpy:
            self._try_init_rclpy()

    # ----------------- rclpy optional support -----------------
    def _try_init_rclpy(self):
        try:
            import rclpy
            from std_msgs.msg import Header
            from rex_interfaces.msg import CalibrateAxis
            from builtin_interfaces.msg import Time as RosTime
        except Exception as e:
            self._log(
                f"rclpy mode requested but failed to import rclpy or messages: {e}")
            self.rclpy_ok = False
            return

        try:
            rclpy.init()
            node = rclpy.create_node('calib_cli_node')
            pub = node.create_publisher(
                CalibrateAxis, '/MQTT/CalibrateAxis', 10)
            self.rclpy_ok = True
            self.rclpy_node = node
            self.rclpy_pub = pub
            self._log("rclpy publisher initialized")
        except Exception as e:
            self._log(f"Failed to initialize rclpy: {e}")
            self.rclpy_ok = False

    def _rclpy_publish(self, action_type, value, vesc_id):
        # Publish a rex_interfaces.msg.CalibrateAxis using rclpy
        try:
            import rclpy
            from std_msgs.msg import Header
            from rex_interfaces.msg import CalibrateAxis
            from builtin_interfaces.msg import Time as RosTime
        except Exception as e:
            self._log(f"rclpy publish failed import: {e}")
            return False

        if not self.rclpy_ok or self.rclpy_node is None or self.rclpy_pub is None:
            self._log("rclpy not initialized")
            return False

        msg = CalibrateAxis()
        now = time.time()
        sec = int(now)
        nanosec = int((now - sec) * 1e9)
        msg.header = Header()
        msg.header.stamp = RosTime(sec=sec, nanosec=nanosec)
        msg.header.frame_id = ""
        msg.vesc_id = int(vesc_id)
        msg.action_type = int(action_type)
        msg.value = float(value)

        try:
            self.rclpy_pub.publish(msg)
            # allow callbacks to process
            rclpy.spin_once(self.rclpy_node, timeout_sec=0.01)
            return True
        except Exception as e:
            self._log(f"rclpy publish failed: {e}")
            return False

    # ----------------- Message formatting & sending -----------------
    def _format_message_str(self, action_type, value, vesc_id):
        now = time.time()
        sec = int(now)
        nanosec = int((now - sec) * 1e9)
        # match example format
        msg = (
            "{header: {stamp: {sec: %d, nanosec: %d}, frame_id: ''}, vesc_id: %d, action_type: %d, value: %s}"
            % (sec, nanosec, int(vesc_id), int(action_type), float(value))
        )
        return msg

    def _send_via_ros2_cli(self, message_str):
        cmd = [
            'ros2', 'topic', 'pub', '--once', '/MQTT/CalibrateAxis', 'rex_interfaces/msg/CalibrateAxis', message_str
        ]
        if self.dry_run:
            self._log(f"DRY-RUN: {' '.join(cmd)}")
            return True, 'dry-run'
        try:
            # call ros2 topic pub CLI
            res = subprocess.run(cmd, stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE, text=True)
            ok = res.returncode == 0
            if not ok:
                self._log(f"ros2 CLI failed (rc={res.returncode}): {
                          res.stderr.strip()}")
                return False, res.stderr.strip()
            else:
                return True, res.stdout.strip()
        except Exception as e:
            self._log(f"ros2 CLI call exception: {e}")
            return False, str(e)

    def send_message(self, action_type, value, vesc_id):
        message_str = self._format_message_str(action_type, value, vesc_id)
        timestamp = datetime.datetime.now().isoformat(timespec='seconds')
        self.history.appendleft(
            (timestamp, action_type, value, vesc_id, message_str))

        if self.dry_run:
            self._log(f"DRY-RUN: action={action_type} value={value} id={vesc_id}")
            return True

        # prefer rclpy if requested and available
        if self.use_rclpy and self.rclpy_ok:
            ok = self._rclpy_publish(action_type, value, vesc_id)
            if ok:
                self._log(f"Published (rclpy) action={
                          action_type} value={value} id={vesc_id}")
                return True
            else:
                # fallback to CLI
                self._log("Falling back to ros2 CLI")

        ok, out = self._send_via_ros2_cli(message_str)
        if ok:
            self._log(f"Sent via CLI action={
                      action_type} value={value} id={vesc_id}")
        else:
            self._log(f"Failed to send action={action_type} value={
                      value} id={vesc_id}: {out}")
        return ok

    # ----------------- periodic worker -----------------
    def _periodic_worker(self):
        self._log(f"Periodic worker started (interval={self.interval}s)")
        while not self._periodic_stop_event.is_set():
            # only send if still in speed mode and periodic_on true
            if self.mode == 'speed' and self.periodic_on:
                # send current speed as SET_VELOCITY
                try:
                    self.send_message(ACTION_TYPE_SET_VELOCITY,
                                      self.speed_rpm, self.selected_motor)
                except Exception as e:
                    self._log(f"Periodic send error: {e}")
            # wait with early exit
            if self._periodic_stop_event.wait(self.interval):
                break
        self._log("Periodic worker stopped")

    def start_periodic(self):
        if self._periodic_thread and self._periodic_thread.is_alive():
            return
        self._periodic_stop_event.clear()
        self._periodic_thread = threading.Thread(
            target=self._periodic_worker, daemon=True)
        self._periodic_thread.start()

    def stop_periodic(self):
        self._periodic_stop_event.set()
        if self._periodic_thread:
            self._periodic_thread.join(timeout=0.2)

    # ----------------- UI logging -----------------
    def _log(self, text):
        ts = datetime.datetime.now().strftime('%H:%M:%S')
        self.history.appendleft((ts, 'LOG', text))

    # ----------------- UI / input handling -----------------
    def cycle_mode(self):
        self.mode_idx = (self.mode_idx + 1) % len(self.mode_cycle)
        self.mode = self.mode_cycle[self.mode_idx]

    def set_motor_button(self, btn):
        if btn in MOTOR_MAP:
            self.selected_motor_button = btn
            self.selected_motor = MOTOR_MAP[btn]
            self._log(f"Selected motor {btn} id={self.selected_motor}")

    def adjust_step(self, delta):
        self.step_deg += float(delta)
        if self.step_deg < 0.0:
            self.step_deg = 0.0
        self._log(f"Step set to {self.step_deg} deg")

    def adjust_speed(self, delta):
        self.speed_rpm += float(delta)
        self._log(f"Speed set to {self.speed_rpm} rpm")

    def send_offset(self, signed_value):
        # signed_value is a float (can be negative)
        self.send_message(ACTION_TYPE_OFFSET, signed_value,
                          self.selected_motor)

    def send_return_to_origin(self):
        self.send_message(ACTION_TYPE_RETURN_TO_ORIGIN,
                          0.0, self.selected_motor)

    def send_confirm_set_origin(self):
        self.send_message(ACTION_TYPE_CONFIRM, 0.0, self.selected_motor)

    def send_stop(self):
        self.send_message(ACTION_TYPE_STOP, 0.0, self.selected_motor)

    def send_cancel(self):
        self.send_message(ACTION_TYPE_CANCEL, 0.0, self.selected_motor)

    # ----------------- Curses UI main loop -----------------
    def run_curses(self, stdscr):
        curses.curs_set(0)
        stdscr.nodelay(True)
        stdscr.timeout(100)

        key = None
        try:
            while True:
                stdscr.erase()
                h, w = stdscr.getmaxyx()

                # Header
                stdscr.addstr(0, 2, f"Calibrate Axis CLI - mode: {self.mode}")
                stdscr.addstr(1, 2, f"Selected motor button: {
                              self.selected_motor_button} (id={self.selected_motor})")
                stdscr.addstr(2, 2, f"Step (deg): {
                              self.step_deg:.3f}   Speed (rpm): {self.speed_rpm:.3f}")
                stdscr.addstr(3, 2, f"Periodic sending: {
                              'ON' if self.periodic_on else 'OFF'}  interval: {self.interval}s")

                stdscr.addstr(
                    5, 2, "Controls: TAB modes | 1-4 wheels | 5-0 manipulator axes")
                stdscr.addstr(
                    6, 2, "Arrows adjust/send | Space periodic | R origin | O confirm | S stop | C cancel | q quit")

                # history
                stdscr.addstr(8, 2, "Last messages:")
                for idx, item in enumerate(list(self.history)[:min(len(self.history), max(0, h - 11))]):
                    y = 9 + idx
                    if isinstance(item, tuple) and len(item) >= 3:
                        # two history formats used: (timestamp, action, text) or (timestamp, action, value, id, msgstr)
                        if item[1] == 'LOG':
                            stdscr.addstr(y, 4, f"[{item[0]}] {item[2]}")
                        else:
                            ts, act, val, idv, msgstr = item
                            stdscr.addstr(y, 4, f"[{ts}] act={
                                          act} val={val} id={idv}")
                    else:
                        stdscr.addstr(y, 4, str(item))

                stdscr.refresh()

                try:
                    key = stdscr.getch()
                except KeyboardInterrupt:
                    break

                if key == -1:
                    continue

                # handle keys
                if key in (9,):  # TAB
                    self.cycle_mode()
                    continue

                if key in (ord('q'), ord('Q')):
                    break

                if key in tuple(ord(str(button)) for button in MOTOR_MAP):
                    btn = int(chr(key))
                    self.set_motor_button(btn)
                    continue

                if key in (ord('r'), ord('R')):
                    self.send_return_to_origin()
                    continue

                if key in (ord('o'), ord('O')):
                    self.send_confirm_set_origin()
                    continue

                if key in (ord('s'), ord('S')):
                    self.stop_periodic()
                    self.periodic_on = False
                    self.speed_rpm = 0
                    self.send_stop()
                    continue

                if key in (ord('c'), ord('C')):
                    self.stop_periodic()
                    self.periodic_on = False
                    self.speed_rpm = 0
                    self.send_cancel()
                    continue

                if key == curses.KEY_UP:
                    if self.mode == 'offset':
                        self.adjust_step(STEP_ADJUST)
                    elif self.mode == 'speed':
                        self.adjust_speed(SPEED_ADJUST)
                    else:
                        # in idle, change nothing
                        pass
                    continue

                if key == curses.KEY_DOWN:
                    if self.mode == 'offset':
                        self.adjust_step(-STEP_ADJUST)
                    elif self.mode == 'speed':
                        self.adjust_speed(-SPEED_ADJUST)
                    continue

                if key == curses.KEY_LEFT:
                    if self.mode == 'offset':
                        self.send_offset(-self.step_deg)
                    elif self.mode == 'speed':
                        # decrease speed value
                        self.adjust_speed(-SPEED_ADJUST)
                    continue

                if key == curses.KEY_RIGHT:
                    if self.mode == 'offset':
                        self.send_offset(self.step_deg)
                    elif self.mode == 'speed':
                        # increase speed value
                        self.adjust_speed(SPEED_ADJUST)
                    continue

                if key == ord(' '):  # space
                    if self.mode == 'speed':
                        self.periodic_on = not self.periodic_on
                        if self.periodic_on:
                            self.start_periodic()
                        else:
                            # user requested DO NOTHING when toggling off (per instructions)
                            # we simply stop the periodic worker; do not send STOP.
                            self.stop_periodic()
                        self._log(f"Periodic toggled: {self.periodic_on}")
                    else:
                        # space has no effect in other modes
                        pass
                    continue

        finally:
            # cleanup
            self.stop_periodic()
            if self.use_rclpy and self.rclpy_ok and self.rclpy_node is not None:
                try:
                    import rclpy
                    rclpy.shutdown()
                except Exception:
                    pass

    # ----------------- helper to run curses main -----------------
    def run(self):
        curses.wrapper(self.run_curses)


# ----------------- CLI entrypoint -----------------

def main():
    parser = argparse.ArgumentParser(
        description='Calibration CLI for rex_interfaces/CalibrateAxis')
    parser.add_argument('--dry-run', action='store_true',
                        help='Log messages instead of publishing them')
    publisher_mode = parser.add_mutually_exclusive_group()
    publisher_mode.add_argument('--use-ros2-cli', dest='use_rclpy', action='store_false',
                                help='Use ros2 topic pub instead of direct rclpy publishing')
    parser.set_defaults(use_rclpy=True)
    parser.add_argument('--interval', type=float, default=PERIODIC_INTERVAL_DEFAULT,
                        help='Periodic send interval in seconds')
    args = parser.parse_args()

    cli = CalibCLI(use_rclpy=args.use_rclpy,
                   dry_run=args.dry_run, interval=args.interval)
    try:
        cli.run()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
