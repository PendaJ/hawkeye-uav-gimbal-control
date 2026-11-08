#!/usr/bin/env python3
"""Automate matched-initial-error Hawkeye Gazebo SS-PD repeatability trials.

Requires: running Gazebo + gimbal adapter + YOLO person tracker, ROS 2 Humble,
          and hawkeye_gazebo_sspd_controller_safe.py.
Only this runner may command /cam_vel during positioning; the controller owns
/cam_vel exclusively during the timed trial. In reproducible mode, Gazebo carrier is reset/armed and gimbal homed.
"""

import argparse
import csv
from datetime import datetime
import json
import math
import os
from pathlib import Path
import re
import signal
import sqlite3
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Point, Twist, Vector3


class ExperimentNode(Node):
    def __init__(self):
        super().__init__('hawkeye_repeatability_runner')
        self.target = None
        self.target_time = 0.0
        self.target_count = 0
        self.orientation = None
        self.orientation_time = 0.0
        # Keep only the newest feedback. The gimbal orientation stream is
        # approximately 1 kHz while vision is approximately 10 Hz; buffering
        # old orientations can starve the vision callback on a loaded CPU.
        self.create_subscription(Point, '/target_coord', self.on_target,
                                 QoSProfile(depth=2,
                                            history=HistoryPolicy.KEEP_LAST,
                                            reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.VOLATILE))
        self.create_subscription(Vector3, '/current_orientation', self.on_orientation,
                                 QoSProfile(depth=1,
                                            history=HistoryPolicy.KEEP_LAST,
                                            reliability=ReliabilityPolicy.BEST_EFFORT,
                                            durability=DurabilityPolicy.VOLATILE))
        self.cmd = self.create_publisher(Twist, '/cam_vel', 10)

    def on_target(self, msg):
        if all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
            self.target = (float(msg.x), float(msg.y), float(msg.z))
            self.target_time = time.monotonic()
            self.target_count += 1

    def on_orientation(self, msg):
        if all(math.isfinite(v) for v in (msg.x, msg.z)):
            self.orientation = (float(msg.x), float(msg.z))
            self.orientation_time = time.monotonic()

    def spin(self, duration=0.03):
        rclpy.spin_once(self, timeout_sec=duration)

    def feedback(self, max_age=0.7):
        now = time.monotonic()
        if (self.target is None or self.orientation is None
                or now - self.target_time > max_age
                or now - self.orientation_time > max_age
                or self.target[2] < 0.5):
            target_age = now - self.target_time if self.target is not None else float('inf')
            orientation_age = now - self.orientation_time if self.orientation is not None else float('inf')
            valid = self.target is not None and self.target[2] >= 0.5
            raise RuntimeError(
                f'Feedback unavailable: target_age={target_age:.3f}s, '
                f'orientation_age={orientation_age:.3f}s, '
                f'target_valid={valid}, target_count={self.target_count}'
            )
        return self.target, self.orientation

    def publish_vel(self, pitch=0.0, yaw=0.0):
        msg = Twist()
        msg.angular.x = float(max(-0.12, min(0.12, pitch)))
        msg.angular.y = float(max(-0.12, min(0.12, yaw)))
        self.cmd.publish(msg)

    def hold_zero(self, duration=0.5):
        until = time.monotonic() + duration
        while time.monotonic() < until:
            self.publish_vel()
            self.spin(0.02)
        self.publish_vel()


def prepare_pose(node, desired_x, desired_y, tolerance, max_seconds):
    """Low-rate visual servo to an OFF-CENTER initial condition, not to zero."""
    deadline = time.monotonic() + max_seconds
    last_report = 0.0
    settled_start = None
    stale_since = None
    # HAWKEYE_SAFE_SETUP_GAP_RECOVERY: setup only, not the scored control interval.
    # Brief YOLO stalls are allowed to recover, but gimbal motion is stopped
    # throughout stale feedback and sustained loss still invalidates the attempt.
    setup_stale_grace_s = 2.0
    while time.monotonic() < deadline:
        node.spin(0.035)
        try:
            (ex, ey, _), (pitch_deg, yaw_deg) = node.feedback()
        except RuntimeError as exc:
            node.publish_vel(0.0, 0.0)
            settled_start = None
            now = time.monotonic()
            if stale_since is None:
                stale_since = now
                print(f'  SETUP PAUSED: {exc}; commanding zero velocity', flush=True)
            elif now - stale_since > setup_stale_grace_s:
                node.publish_vel(0.0, 0.0)
                raise RuntimeError(
                    f'Setup feedback unavailable for >{setup_stale_grace_s:.1f}s; {exc}'
                ) from exc
            continue
        if stale_since is not None:
            print('  SETUP RESUMED: fresh target and orientation', flush=True)
            stale_since = None
        # Mechanical joint limits are wider; these conservative soft limits
        # prevent the automated setup from turning far outside the view.
        if abs(pitch_deg) > 65 or abs(yaw_deg) > 150:
            node.publish_vel()
            raise RuntimeError(f'Setup reached joint safety limit: pitch={pitch_deg:.1f}, yaw={yaw_deg:.1f}')
        err_x, err_y = desired_x - ex, desired_y - ey
        if abs(err_x) <= tolerance and abs(err_y) <= tolerance:
            node.publish_vel()
            if settled_start is None:
                settled_start = time.monotonic()
            if time.monotonic() - settled_start >= 0.65:
                node.hold_zero(0.45)
                try:
                    target, orientation = node.feedback()
                except RuntimeError as exc:
                    node.publish_vel(0.0, 0.0)
                    settled_start = None
                    print(f'  SETUP SETTLE RETRY: {exc}', flush=True)
                    continue
                if (abs(target[0] - desired_x) <= tolerance
                        and abs(target[1] - desired_y) <= tolerance):
                    return target, orientation
                settled_start = None
        else:
            settled_start = None
            # Calibrated in this Gazebo scene: positive yaw increases image dx;
            # positive pitch increases image dy. This is intentionally the
            # opposite purpose from the controller's corrective commands.
            yaw = max(-0.10, min(0.10, 0.0020 * err_x))
            pitch = max(-0.10, min(0.10, 0.0030 * err_y))
            if abs(err_x) < 2.0:
                yaw = 0.0
            if abs(err_y) < 2.0:
                pitch = 0.0
            if (yaw_deg >= 145 and yaw > 0) or (yaw_deg <= -145 and yaw < 0):
                yaw = 0.0
            if (pitch_deg >= 60 and pitch > 0) or (pitch_deg <= -60 and pitch < 0):
                pitch = 0.0
            node.publish_vel(pitch=pitch, yaw=yaw)
        now = time.monotonic()
        if now - last_report > 2.0:
            print(f'  positioning: dx={ex:+.1f} dy={ey:+.1f} px; pitch={pitch_deg:+.1f} yaw={yaw_deg:+.1f} deg', flush=True)
            last_report = now
    node.hold_zero()
    raise RuntimeError(f'Cannot reach ({desired_x:+.1f}, {desired_y:+.1f}) px within {max_seconds:.0f} seconds')


def wait_for_feedback(node, max_seconds=12.0):
    end = time.monotonic() + max_seconds
    while time.monotonic() < end:
        node.spin(0.05)
        try:
            return node.feedback()
        except RuntimeError:
            pass
    raise RuntimeError('No fresh person detection and gimbal orientation. Start Gazebo and YOLO first.')


def stop_process(proc, grace=8.0):
    if proc is None:
        return
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=grace)
        except subprocess.TimeoutExpired:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()


def bag_message_counts(bag_dir):
    counts = {}
    for db in sorted(Path(bag_dir).glob('*.db3')):
        con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
        try:
            rows = con.execute('''SELECT t.name, COUNT(*) FROM messages m
                                  JOIN topics t ON t.id=m.topic_id GROUP BY t.name''').fetchall()
            for topic, count in rows:
                counts[topic] = counts.get(topic, 0) + count
        finally:
            con.close()
    return counts


def bag_vision_gap(bag_dir, start_ns, end_ns):
    """Largest inter-message gap for /target_coord during controller process.

    Bag timestamps use wall clock in this setup (no --use-sim-time).
    This is independent of whether the runner's own callback was scheduled.
    """
    stamps = [start_ns, end_ns]
    for db in sorted(Path(bag_dir).glob('*.db3')):
        con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
        try:
            stamps.extend(row[0] for row in con.execute("SELECT m.timestamp FROM messages m "
                                  "JOIN topics t ON t.id=m.topic_id "
                                  "WHERE t.name = '/target_coord' "
                                  "AND m.timestamp BETWEEN ? AND ? "
                                  "ORDER BY m.timestamp",
                                  (int(start_ns), int(end_ns))))
        finally:
            con.close()
    stamps.sort()
    return (max((b-a for a,b in zip(stamps, stamps[1:])), default=end_ns-start_ns)/1e9,
            max(0,len(stamps)-2))


def save_json(path, data):
    Path(path).write_text(json.dumps(data, indent=2, sort_keys=True) + '\n')


def run_trial(node, args, session_dir, trial_index, attempt=1):
    trial_dir = session_dir / f'run_{trial_index:02d}_attempt_{attempt:02d}'
    trial_dir.mkdir()
    metadata = {'run': trial_index, 'attempt': attempt, 'desired_initial_error_px': [args.target_x, args.target_y],
                'tolerance_px': args.tolerance, 'control_duration_s': args.duration,
                'controller': str(args.controller), 'candidate_gains': {'kp': args.kp, 'kd': args.kd, 'omega_o': args.omega_o}, 'success': False}
    bag_proc = controller_proc = None
    bag_log = controller_log = None
    controller_wall_start_ns = controller_wall_end_ns = None
    seen_runner_vision_delay = False
    try:
        print(f'\n=== Run {trial_index}/{args.runs}, attempt {attempt}: positioning camera ===', flush=True)
        # Check no previous controller is still advertising the same node name.
        if any(name == 'gimbal_controller' for name in node.get_node_names()):
            raise RuntimeError('Another gimbal_controller is already running. Stop it before automation.')
        if getattr(args, 'repro', False):
            from repro_tools import reset as carrier_reset, home_gimbal
            node.hold_zero(.4)
            carrier = carrier_reset()
            expected = getattr(args, 'carrier_mode', None)
            if expected and carrier['mode'] != expected:
                raise RuntimeError(f'Carrier is {carrier["mode"]}, required {expected}')
            metadata['carrier_reset'] = carrier
            metadata['home_orientation_deg'] = home_gimbal(node, timeout=args.home_timeout)
            print('  repeatable carrier reset and gimbal home:',
                  carrier['epoch'], metadata['home_orientation_deg'], flush=True)
        # Reset/homing can coincide with a transient inference stall.
        # Wait for fresh feedback at zero gimbal velocity before setup begins.
        node.hold_zero(0.2)
        wait_for_feedback(node, max_seconds=8.0)
        target, orientation = prepare_pose(node, args.target_x, args.target_y,
                                           args.tolerance, args.position_timeout)
        metadata['initial_target'] = list(target)
        metadata['initial_orientation_deg'] = {'pitch': orientation[0], 'yaw': orientation[1]}
        print(f'  initial dx={target[0]:+.2f}, dy={target[1]:+.2f} px. Recording...', flush=True)

        bag_folder = trial_dir / 'bag'
        bag_log = open(trial_dir / 'rosbag.log', 'w')
        bag_cmd = ['ros2', 'bag', 'record', '--qos-profile-overrides-path', str(args.qos),
                   '-o', str(bag_folder), '/target_coord', '/current_orientation', '/cam_vel', '/pid_log', '/hawkeye/carrier/state']
        before_subs = node.count_subscribers('/cam_vel')
        bag_proc = subprocess.Popen(bag_cmd, stdout=bag_log, stderr=subprocess.STDOUT)
        # Let rosbag discover all topics and attach to the existing /cam_vel publisher.
        connected = False
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            node.spin(0.08)
            if bag_proc.poll() is not None:
                raise RuntimeError('ros2 bag exited early; inspect rosbag.log')
            if node.count_subscribers('/cam_vel') > before_subs:
                connected = True
                break
        if not connected:
            raise RuntimeError('Bag recorder did not subscribe to /cam_vel (check rosbag.log).')

        # One second of stationary baseline, all written into the same bag.
        node.hold_zero(1.0)
        pre_target, pre_orientation = wait_for_feedback(node, max_seconds=8.0)
        metadata['pre_control_target'] = list(pre_target)
        metadata['pre_control_orientation_deg'] = {'pitch': pre_orientation[0], 'yaw': pre_orientation[1]}
        if (abs(pre_target[0] - args.target_x) > args.tolerance
                or abs(pre_target[1] - args.target_y) > args.tolerance):
            raise RuntimeError('Initial error drifted during bag pre-roll; trial canceled.')
        if getattr(args, 'repro', False):
            from repro_tools import arm as carrier_arm
            metadata['carrier_armed'] = carrier_arm(metadata['carrier_reset']['epoch'])
            print('  carrier ARMED at y0, phase zero on first nonzero gimbal command', flush=True)
        print(f'  recording active. Starting {args.duration:.1f}-s SS-PD controller.', flush=True)
        controller_log = open(trial_dir / 'controller.log', 'w')
        ctrl_cmd = [sys.executable, str(args.controller), '--ros-args',
                    '-p', 'enable_motion:=true', '-p', f'run_seconds:={args.duration}',
                    '-p', f'kp_yaw:={args.kp}', '-p', f'kd_yaw:={args.kd}',
                    '-p', f'omega_o_yaw:={args.omega_o}',
                    '-p', f'kp_pitch:={args.kp}', '-p', f'kd_pitch:={args.kd}',
                    '-p', f'omega_o_pitch:={args.omega_o}']
        controller_wall_start_ns = time.time_ns()
        controller_proc = subprocess.Popen(ctrl_cmd, stdout=controller_log,
                                           stderr=subprocess.STDOUT)
        start_wait = time.monotonic()
        while controller_proc.poll() is None:
            node.spin(0.02)
            # Allow warm-up and 0.6-second explicit stop, but no indefinite hang.
            if time.monotonic() - start_wait > args.duration + 12.0:
                raise RuntimeError('Controller exceeded safety watchdog; terminating.')
            # Do not prematurely kill the safe controller because this monitor
            # missed one callback. The controller independently commands ZERO
            # after 0.6s stale target data; the Gazebo adapter also has a deadman.
            # Record this warning and verify the actual source timing in rosbag.
            age = time.monotonic() - node.target_time
            if age > 0.7 and not seen_runner_vision_delay:
                print(f'  WARNING: runner has not processed a target update for '
                      f'{age:.2f}s; checking rosbag timestamps after shutdown.', flush=True)
                seen_runner_vision_delay = True
        controller_wall_end_ns = time.time_ns()
        controller_log.flush()
        print(f'  controller exited with code {controller_proc.returncode}; checking shutdown.', flush=True)
        # Parent sends zero only AFTER controller exits; never compete for actuation.
        node.hold_zero(0.5)
        settle_end = time.monotonic() + 1.3
        while time.monotonic() < settle_end:
            node.spin(0.04)
        post_target, post_orientation = node.feedback()
        metadata['post_control_target'] = list(post_target)
        metadata['post_control_orientation_deg'] = {'pitch': post_orientation[0], 'yaw': post_orientation[1]}
        if getattr(args, 'repro', False):
            from repro_tools import state as carrier_state
            metadata['carrier_after_control'] = carrier_state()
            if (not metadata['carrier_after_control'].get('active') or
                metadata['carrier_after_control'].get('epoch') != metadata['carrier_reset']['epoch']):
                raise RuntimeError('Carrier did not trigger from closed-loop commands; invalid trial')
        metadata['post_control_radial_px'] = math.hypot(post_target[0], post_target[1])
        text_log = (trial_dir / 'controller.log').read_text()
        match = re.search(r'Control duration\s+([0-9.]+)\s+s completed', text_log)
        metadata['reported_control_duration_s'] = float(match.group(1)) if match else None
        metadata['controller_returncode'] = controller_proc.returncode
        print(f'  final dx={post_target[0]:+.2f}, dy={post_target[1]:+.2f} px; '
              f'radial={metadata["post_control_radial_px"]:.2f} px', flush=True)
        if (controller_proc.returncode != 0 or match is None
                or abs(float(match.group(1)) - args.duration) > 0.15):
            raise RuntimeError('Controller did not report completing the requested duration.')
        metadata['success'] = True
    except (Exception, KeyboardInterrupt) as exc:
        metadata['error'] = str(exc)
        print(f'  RUN FAILED: {exc}', file=sys.stderr, flush=True)
        if controller_proc and controller_proc.poll() is None:
            stop_process(controller_proc, grace=3.0)
    finally:
        # First stop the controller, THEN command zero from the runner.
        # Otherwise the two publishers race on /cam_vel and a live controller
        # could send nonzero motion after the runner's zero command.
        if controller_proc and controller_proc.poll() is None:
            stop_process(controller_proc, grace=3.0)
        node.hold_zero(0.65)
        if controller_log:
            controller_log.close()
        stop_process(bag_proc, grace=12.0)
        if bag_log:
            bag_log.close()
        try:
            counts = bag_message_counts(trial_dir / 'bag')
        except (sqlite3.Error, OSError) as exc:
            counts = {}
            metadata['success'] = False
            metadata['error'] = f'Could not inspect bag database: {exc}'
        metadata['recorded_message_counts'] = counts
        metadata['runner_vision_callback_delay_seen'] = seen_runner_vision_delay
        if controller_wall_start_ns is not None and controller_wall_end_ns is not None:
            try:
                gap, num = bag_vision_gap(trial_dir / 'bag',
                                          controller_wall_start_ns, controller_wall_end_ns)
                metadata['max_recorded_vision_gap_during_controller_s'] = round(gap, 4)
                metadata['vision_messages_during_controller'] = num
                print(f'  vision check: {num} samples during controller process; '
                      f'max recorded gap={gap:.3f}s', flush=True)
                if gap > args.max_vision_gap:
                    metadata['success'] = False
                    metadata['error'] = (f'Vision source gap {gap:.3f}s exceeded '
                                         f'{args.max_vision_gap:.2f}s threshold; '
                                         'trial excluded from matched comparison.')
            except (sqlite3.Error, OSError) as exc:
                metadata['success'] = False
                metadata['error'] = f'Cannot verify vision timestamps: {exc}'
        # Failure here means the trial MUST NOT be included in metrics.
        if metadata['success'] and (counts.get('/cam_vel', 0) < 200 or counts.get('/pid_log', 0) < 200):
            metadata['success'] = False
            metadata['error'] = 'Incomplete bag; check /cam_vel and /pid_log counts.'
        save_json(trial_dir / 'metadata.json', metadata)
        print('  recorded:', counts, flush=True)
        print(f'  status: {"VALID" if metadata["success"] else "INVALID"}', flush=True)
    return metadata


def main():
    parser = argparse.ArgumentParser(description='Automated Gazebo X3 SS-PD matched-error trials')
    parser.add_argument('--runs', type=int, default=3)
    parser.add_argument('--kp', type=float, default=9.6348)
    parser.add_argument('--kd', type=float, default=0.1508)
    parser.add_argument('--omega-o', type=float, default=600.2994)
    parser.add_argument('--target-x', type=float, default=80.0,
                        help='Desired initial horizontal image error (pixels)')
    parser.add_argument('--target-y', type=float, default=28.0,
                        help='Desired initial vertical image error (pixels)')
    parser.add_argument('--tolerance', type=float, default=5.0)
    parser.add_argument('--duration', type=float, default=3.0)
    parser.add_argument('--position-timeout', type=float, default=40.0)
    parser.add_argument('--max-vision-gap', type=float, default=0.75,
                        help='Largest allowed source image-error gap in bag, seconds')
    parser.add_argument('--max-retries', type=int, default=2,
                        help='Extra attempts per trial if source stalls or a run is invalid')
    parser.add_argument('--controller', type=Path,
                        default=Path.home() / 'Downloads/hawkeye_gazebo_sspd_controller_safe.py')
    parser.add_argument('--qos', type=Path,
                        default=Path.home() / 'hawkeye_x3_ws/rosbags/qos_override.yaml')
    parser.add_argument('--output', type=Path,
                        default=Path.home() / 'hawkeye_x3_ws/rosbags')
    args = parser.parse_args()
    if not (1 <= args.runs <= 20 and 0.5 <= args.duration <= 30 and
            0 < args.tolerance <= 20 and 5 <= args.position_timeout <= 120 and
            0.5 <= args.max_vision_gap <= 2.0 and 0 <= args.max_retries <= 5 and
            all(math.isfinite(v) for v in (args.target_x, args.target_y, args.duration))):
        parser.error('Invalid trial configuration.')
    args.controller = args.controller.expanduser().resolve()
    args.qos = args.qos.expanduser().resolve()
    if not args.controller.is_file():
        parser.error(f'Controller not found: {args.controller}')
    if not args.qos.is_file():
        parser.error(f'QoS file not found: {args.qos}')
    if not shutil_which('ros2'):
        parser.error('ros2 command not found; source /opt/ros/humble/setup.bash')
    output = args.output.expanduser()
    output.mkdir(parents=True, exist_ok=True)
    session_dir = output / ('auto_sspd_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
    session_dir.mkdir(exist_ok=False)
    save_json(session_dir / 'configuration.json',
              {'runs': args.runs, 'target_initial_px': [args.target_x, args.target_y],
               'tolerance_px': args.tolerance, 'duration_s': args.duration,
               'max_vision_gap_s': args.max_vision_gap, 'max_retries': args.max_retries,
               'controller': str(args.controller), 'qos': str(args.qos),
               'recorded_topics': ['/target_coord', '/current_orientation', '/cam_vel', '/pid_log'],
               'position_controller': 'bounded P, image feedback, maximum 0.10 rad/s',
               'notes': 'Controller observer and NPSO gains unchanged; matched error only, no drone reset.'})
    print('Session:', session_dir, flush=True)
    rclpy.init()
    node = ExperimentNode()
    summaries = []
    try:
        wait_for_feedback(node)
        if node.count_subscribers('/cam_vel') < 1:
            raise RuntimeError('No gimbal adapter subscriber on /cam_vel.')
        if any(name == 'gimbal_controller' for name in node.get_node_names()):
            raise RuntimeError('A gimbal_controller is already running; stop it first.')
        for index in range(1, args.runs + 1):
            valid = False
            for attempt in range(1, args.max_retries + 2):
                result = run_trial(node, args, session_dir, index, attempt)
                summaries.append(result)
                if result['success']:
                    valid = True
                    break
                if attempt <= args.max_retries:
                    print(f'  Retrying Run {index} with fresh, matched initial error...', flush=True)
                    node.hold_zero(0.5)
            if not valid:
                print(f'Stopping: Run {index} did not pass after '
                      f'{args.max_retries + 1} attempts.', flush=True)
                break
    except (Exception, KeyboardInterrupt) as exc:
        print('ABORTED:', exc, file=sys.stderr, flush=True)
    finally:
        try:
            node.hold_zero(0.65)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        with open(session_dir / 'summary.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['run', 'attempt', 'valid', 'initial_x_px', 'initial_y_px',
                             'final_x_px', 'final_y_px', 'final_radial_px',
                             'actual_control_s', 'cmd_messages', 'pid_log_messages', 'error'])
            for r in summaries:
                ini = r.get('pre_control_target', [None, None])
                end = r.get('post_control_target', [None, None])
                counts = r.get('recorded_message_counts', {})
                writer.writerow([r['run'], r['attempt'], r['success'], ini[0], ini[1], end[0], end[1],
                                 r.get('post_control_radial_px'), r.get('reported_control_duration_s'),
                                 counts.get('/cam_vel', 0), counts.get('/pid_log', 0), r.get('error', '')])
        print('Trial summary:', session_dir / 'summary.csv', flush=True)
        print('Data folder:', session_dir, flush=True)
    return 0 if sum(r['success'] for r in summaries) == args.runs else 1


def shutil_which(name):
    from shutil import which
    return which(name)


if __name__ == '__main__':
    sys.exit(main())
