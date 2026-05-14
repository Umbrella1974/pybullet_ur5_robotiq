import csv
import math
import os
import random
import socket
import sys
import traceback
from datetime import datetime

MISSING_DEPENDENCIES = []
IMPORT_FAILURES = []


def record_missing_dependency(exc):
    module_name = getattr(exc, "name", None) or str(exc)
    if module_name not in MISSING_DEPENDENCIES:
        MISSING_DEPENDENCIES.append(module_name)


try:
    import numpy as np
except ModuleNotFoundError as exc:
    np = None
    record_missing_dependency(exc)

try:
    import pybullet as p
except ModuleNotFoundError as exc:
    p = None
    record_missing_dependency(exc)

ClutteredPushGrasp = None
UR5Robotiq85 = None
YCBModels = None

if np is not None and p is not None:
    try:
        from env import ClutteredPushGrasp
        from robot import UR5Robotiq85

        try:
            from utilities import YCBModels
        except ImportError:
            from utilities import Models as YCBModels
    except ModuleNotFoundError as exc:
        record_missing_dependency(exc)
    except Exception as exc:
        IMPORT_FAILURES.append(str(exc))

try:
    import yaml
except ImportError:
    yaml = None


SIM_DT = 1.0 / 240.0
MAGIC = b"\xAA\x55\xAA\x55"

STATE_INIT_PREPARE = "INIT_PREPARE"
STATE_INIT = "INIT"
STATE_USER_CONTROL = "USER_CONTROL"
STATE_AUTO_WAIT = "AUTO_WAIT"
STATE_AUTO_PERTURB_CLOSE = "AUTO_PERTURB_CLOSE"
STATE_AUTO_PERTURB_OPEN = "AUTO_PERTURB_OPEN"

AUTO_NONE = "none"
AUTO_CLOSE = "close"
AUTO_OPEN = "open"

VALID_EVENT_TYPES = {
    "EXPERIMENT_START",
    "INITIAL_POSE_START",
    "INITIAL_POSE_DONE",
    "MANUAL_START",
    "AUTO_PERTURB_START_CLOSE",
    "AUTO_PERTURB_START_OPEN",
    "AUTO_PERTURB_END",
    "USER_INPUT_Z",
    "USER_INPUT_X",
    "FORCE_ABOVE_HIGH",
    "FORCE_BELOW_LOW",
    "FORCE_BACK_IN_RANGE",
    "EMERGENCY_HIGH_FORCE",
    "EXPERIMENT_END",
}

KEY_IS_DOWN = getattr(p, "KEY_IS_DOWN", 1) if p is not None else 1
KEY_WAS_TRIGGERED = getattr(p, "KEY_WAS_TRIGGERED", 2) if p is not None else 2

TIME_SERIES_FIELDS = [
    "timestamp_iso",
    "time_sec",
    "trial_elapsed_sec",
    "participant_id",
    "condition",
    "formal_experiment",
    "manual_start",
    "init_phase",
    "initial_pose_reached",
    "state",
    "event_id",
    "auto_direction",
    "auto_active",
    "auto_delta",
    "x",
    "y",
    "z",
    "roll",
    "pitch",
    "yaw",
    "gripper_opening",
    "user_key_z",
    "user_key_x",
    "user_input",
    "tf_force_raw",
    "if_force_raw",
    "tf_force_smooth",
    "if_force_smooth",
    "tf_contact_count",
    "if_contact_count",
    "contact_force",
    "contact_frames",
    "no_contact_frames",
    "contact_hold_sec",
    "no_contact_hold_sec",
    "stable_contact",
    "stable_no_contact",
    "tf_level",
    "if_level",
    "overall_level",
    "target_low",
    "target_high",
    "target_mid",
    "force_error",
    "in_target_range",
    "esp32_tf_sent",
    "esp32_if_sent",
]

EVENT_FIELDS = [
    "timestamp_iso",
    "time_sec",
    "trial_elapsed_sec",
    "participant_id",
    "condition",
    "formal_experiment",
    "event_id",
    "event_type",
    "state",
    "auto_direction",
    "contact_force",
    "gripper_opening",
    "detail",
]


def key_down(keys, key):
    return bool(keys.get(key, 0) & KEY_IS_DOWN)


def key_triggered(keys, key):
    return bool(keys.get(key, 0) & KEY_WAS_TRIGGERED)


def safe_step_simulation(env):
    if hasattr(env, "step_simulation"):
        env.step_simulation()
    else:
        p.stepSimulation()


def get_link_name(robot_id, link_index):
    if link_index < 0:
        return "base"
    try:
        return p.getJointInfo(robot_id, link_index)[12].decode("utf-8")
    except Exception:
        return f"unknown_link_{link_index}"


def print_current_contacts(env):
    contacts = p.getContactPoints(bodyA=env.robot.id, bodyB=env.boxID)

    if len(contacts) == 0:
        print("\nNo contact.")
        return

    print("\nCurrent contacts:")
    for contact in contacts:
        robot_link_index = contact[3]
        object_link_index = contact[4]
        normal_force = contact[9]
        link_name = get_link_name(env.robot.id, robot_link_index)
        print(
            f"robot link index={robot_link_index}, "
            f"link name={link_name}, "
            f"object link index={object_link_index}, "
            f"normal force={normal_force:.3f}"
        )


def read_separate_fingertip_forces(env, tf_link, if_link):
    contacts = p.getContactPoints(bodyA=env.robot.id, bodyB=env.boxID)

    tf_forces = []
    if_forces = []

    for contact in contacts:
        robot_link_index = contact[3]
        normal_force = contact[9]
        if robot_link_index == tf_link:
            tf_forces.append(normal_force)
        elif robot_link_index == if_link:
            if_forces.append(normal_force)

    tf_force_sum = float(sum(tf_forces)) if tf_forces else 0.0
    if_force_sum = float(sum(if_forces)) if if_forces else 0.0

    tf_max_force = float(max(tf_forces)) if tf_forces else 0.0
    if_max_force = float(max(if_forces)) if if_forces else 0.0

    tf_contact_count = len(tf_forces)
    if_contact_count = len(if_forces)

    return (
        tf_force_sum,
        tf_max_force,
        tf_contact_count,
        if_force_sum,
        if_max_force,
        if_contact_count,
    )


def force_to_level(force, thresholds):
    if len(thresholds) != 3:
        raise ValueError("level_thresholds must contain exactly three values.")
    if force < thresholds[0]:
        return 0
    if force < thresholds[1]:
        return 1
    if force < thresholds[2]:
        return 2
    return 3


def generate_packet(payload):
    payload = bytearray(payload)
    checksum = sum(payload) & 0xFF

    packet = bytearray(MAGIC)
    packet.append(len(payload))
    packet.extend(payload)
    packet.append(checksum)
    return packet


def build_action_from_config(cfg):
    initial = cfg["initial_action"]
    return np.array(
        [
            initial["x"],
            initial["y"],
            initial["z"],
            initial["roll"],
            initial["pitch"],
            initial["yaw"],
            initial["gripper_opening"],
        ],
        dtype=float,
    )


def step_simulation_for_steps(env, steps, sim_dt=SIM_DT):
    executed_steps = 0
    for _ in range(int(steps)):
        if not p.isConnected():
            break
        env.step_simulation()
        executed_steps += 1
    return executed_steps * sim_dt, executed_steps


def move_to_initial_pose(env, action, steps, sim_dt=SIM_DT):
    executed_steps = 0
    for _ in range(int(steps)):
        if not p.isConnected():
            break
        env.robot.move_ee(action[:6], control_method="end")
        env.robot.move_gripper(action[6])
        env.step_simulation()
        executed_steps += 1
    return executed_steps * sim_dt, executed_steps


def get_link_world_pose(robot_id, link_index):
    state = p.getLinkState(robot_id, link_index, computeForwardKinematics=True)
    return np.array(state[4], dtype=float), np.array(state[5], dtype=float)


def rotate_local_offset(local_offset, orientation_quat):
    rotation = np.array(p.getMatrixFromQuaternion(orientation_quat), dtype=float).reshape(3, 3)
    return rotation.dot(np.array(local_offset, dtype=float))


def create_centered_contact_object(env, tf_link, if_link, center_cfg):
    left_pos, _left_quat = get_link_world_pose(env.robot.id, tf_link)
    right_pos, _right_quat = get_link_world_pose(env.robot.id, if_link)
    center_pos = 0.5 * (left_pos + right_pos)

    reference_link = getattr(env.robot, "eef_id", tf_link)
    _reference_pos, reference_quat = get_link_world_pose(env.robot.id, reference_link)

    offset_local = center_cfg.get("position_offset_local")
    if offset_local is None:
        offset_local = center_cfg.get("position_offset", [0.0, 0.0, 0.0])
    offset_world = rotate_local_offset(offset_local, reference_quat)
    center_pos = center_pos + offset_world

    half_extents = [float(value) for value in center_cfg["half_extents"]]

    original_box_id = getattr(env, "boxID", None)
    if center_cfg.get("replace_env_box", True) and original_box_id is not None:
        p.removeBody(original_box_id)

    collision_shape_id = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=half_extents,
    )
    visual_shape_id = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=half_extents,
        rgbaColor=[0.20, 0.80, 0.35, 1.0],
    )
    body_id = p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=collision_shape_id,
        baseVisualShapeIndex=visual_shape_id,
        basePosition=center_pos.tolist(),
        baseOrientation=reference_quat.tolist(),
    )
    p.changeDynamics(body_id, -1, lateralFriction=1.0, spinningFriction=0.001, restitution=0.0)

    env.original_boxID = original_box_id
    env.boxID = body_id

    return {
        "body_id": body_id,
        "center_pos": center_pos.tolist(),
        "orientation_quat": reference_quat.tolist(),
        "half_extents": half_extents,
        "replace_env_box": bool(center_cfg.get("replace_env_box", True)),
    }


def create_environment():
    ycb_models = YCBModels(os.path.join("./data/ycb", "**", "textured-decmp.obj"))
    robot = UR5Robotiq85((0, 0.5, 0), (0, 0, 0))
    env = ClutteredPushGrasp(robot, ycb_models, camera=None, vis=True)
    env.reset()
    return env


def now_iso():
    return datetime.now().isoformat()


def to_int_flag(value):
    return 1 if value else 0


def ensure_runtime_dependencies():
    if yaml is None and "yaml" not in MISSING_DEPENDENCIES:
        MISSING_DEPENDENCIES.append("yaml")

    dependency_alias = {
        "attrdict": "attrdict",
        "cv2": "opencv-python",
        "numpy": "numpy",
        "pybullet": "pybullet",
        "pybullet_data": "pybullet",
        "scipy": "scipy",
        "tqdm": "tqdm",
        "yaml": "pyyaml",
    }

    if not MISSING_DEPENDENCIES and not IMPORT_FAILURES:
        return

    print("Experiment 2 cannot start because Python dependencies are missing or failed to import.")

    if MISSING_DEPENDENCIES:
        print("Missing modules:")
        for name in sorted(set(MISSING_DEPENDENCIES)):
            print(f"  - {name}")

        packages = sorted(
            {dependency_alias.get(name, name) for name in MISSING_DEPENDENCIES}
        )
        print(f"Install them with: pip install {' '.join(packages)}")

    if IMPORT_FAILURES:
        print("Import failures:")
        for message in IMPORT_FAILURES:
            print(f"  - {message}")

    sys.exit(1)


class ESP32FeedbackClient:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = None
        self.last_payload = None
        self.connect()

    def connect(self):
        try:
            if self.sock is not None:
                try:
                    self.sock.close()
                except Exception:
                    pass

            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.sock.settimeout(2.0)
            self.sock.connect((self.ip, self.port))
            self.sock.settimeout(None)
            print(f"Connected to ESP32 at {self.ip}:{self.port}")
        except Exception as exc:
            print(f"Failed to connect to ESP32: {exc}")
            self.sock = None

    def send_levels(self, tf_level, if_level):
        tf_level = int(np.clip(tf_level, 0, 3))
        if_level = int(np.clip(if_level, 0, 3))
        payload = [1, tf_level, 2, if_level]

        if payload == self.last_payload:
            return

        self.last_payload = payload
        packet = generate_packet(payload)

        try:
            if self.sock is None:
                self.connect()
            if self.sock is not None:
                self.sock.sendall(packet)
        except Exception as exc:
            print(f"ESP32 send failed: {exc}")
            self.sock = None

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None


class FeedbackSender:
    def __init__(self, client, feedback_period_sec, keepalive_period_sec):
        self.client = client
        self.feedback_period_sec = float(feedback_period_sec)
        self.keepalive_period_sec = float(keepalive_period_sec)
        self.last_sent_levels = None
        self.last_send_time_sec = None

    def _send(self, levels, sim_time_sec, force_duplicate=False):
        if self.client is None:
            self.last_sent_levels = levels
            self.last_send_time_sec = sim_time_sec
            return levels

        if force_duplicate and hasattr(self.client, "last_payload"):
            self.client.last_payload = None

        self.client.send_levels(levels[0], levels[1])
        self.last_sent_levels = levels
        self.last_send_time_sec = sim_time_sec
        return levels

    def maybe_send(self, tf_level, if_level, sim_time_sec):
        levels = (int(tf_level), int(if_level))

        if self.last_sent_levels is None:
            return self._send(levels, sim_time_sec)

        if self.last_send_time_sec is None:
            return self._send(levels, sim_time_sec)

        elapsed_sec = sim_time_sec - self.last_send_time_sec

        if levels != self.last_sent_levels:
            if elapsed_sec >= self.feedback_period_sec:
                return self._send(levels, sim_time_sec)
            return self.last_sent_levels

        if elapsed_sec >= self.keepalive_period_sec:
            return self._send(levels, sim_time_sec, force_duplicate=True)

        return self.last_sent_levels

    def shutdown(self, sim_time_sec):
        levels = (0, 0)
        if self.client is not None:
            self._send(levels, sim_time_sec, force_duplicate=True)
            self.client.close()
        else:
            self.last_sent_levels = levels
            self.last_send_time_sec = sim_time_sec
        return levels


class ExperimentLogger:
    def __init__(
        self,
        time_series_path,
        event_path,
        save_time_series=True,
        save_event=True,
    ):
        self.time_series_path = time_series_path
        self.event_path = event_path
        self.save_time_series = save_time_series
        self.save_event = save_event
        self._time_series_file = None
        self._event_file = None
        self._time_series_writer = None
        self._event_writer = None
        self.pending_sample_event_ids = []
        self.event_index = 0
        self.event_counts = {}

        if self.save_time_series:
            self._time_series_file = open(
                self.time_series_path, "w", newline="", encoding="utf-8"
            )
            self._time_series_writer = csv.DictWriter(
                self._time_series_file, fieldnames=TIME_SERIES_FIELDS
            )
            self._time_series_writer.writeheader()
            self._time_series_file.flush()

        if self.save_event:
            self._event_file = open(
                self.event_path, "w", newline="", encoding="utf-8"
            )
            self._event_writer = csv.DictWriter(
                self._event_file, fieldnames=EVENT_FIELDS
            )
            self._event_writer.writeheader()
            self._event_file.flush()

    def record_event(
        self,
        event_type,
        timestamp_iso,
        time_sec,
        trial_elapsed_sec,
        participant_id,
        condition,
        formal_experiment,
        state,
        auto_direction,
        contact_force,
        gripper_opening,
        detail="",
    ):
        if event_type not in VALID_EVENT_TYPES:
            raise ValueError(f"Unsupported event type: {event_type}")

        self.event_index += 1
        event_id = f"E{self.event_index:05d}"
        self.pending_sample_event_ids.append(event_id)
        self.event_counts[event_type] = self.event_counts.get(event_type, 0) + 1

        if self._event_writer is not None:
            self._event_writer.writerow(
                {
                    "timestamp_iso": timestamp_iso,
                    "time_sec": f"{time_sec:.6f}",
                    "trial_elapsed_sec": f"{trial_elapsed_sec:.6f}",
                    "participant_id": participant_id,
                    "condition": condition,
                    "formal_experiment": int(bool(formal_experiment)),
                    "event_id": event_id,
                    "event_type": event_type,
                    "state": state,
                    "auto_direction": auto_direction,
                    "contact_force": f"{contact_force:.6f}",
                    "gripper_opening": f"{gripper_opening:.6f}",
                    "detail": detail,
                }
            )
            self._event_file.flush()

        return event_id

    def consume_pending_event_ids(self):
        if not self.pending_sample_event_ids:
            return ""
        event_ids = "|".join(self.pending_sample_event_ids)
        self.pending_sample_event_ids.clear()
        return event_ids

    def write_time_series(self, row):
        if self._time_series_writer is None:
            self.pending_sample_event_ids.clear()
            return
        self._time_series_writer.writerow(row)
        self._time_series_file.flush()

    def close(self):
        if self._time_series_file is not None:
            self._time_series_file.close()
            self._time_series_file = None
        if self._event_file is not None:
            self._event_file.close()
            self._event_file = None


def load_config(config_path):
    if yaml is None:
        print("PyYAML is required for Experiment 2.")
        print("Install it with: pip install pyyaml")
        sys.exit(1)

    with open(config_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def validate_config(config):
    condition = config["experiment"]["condition"]
    if condition not in ("visual_only", "vision_haptic"):
        raise ValueError("experiment.condition must be visual_only or vision_haptic.")

    thresholds = config["force"]["level_thresholds"]
    if len(thresholds) != 3:
        raise ValueError("force.level_thresholds must contain exactly three values.")

    if config["force"]["combine_method"] not in ("max", "sum"):
        raise ValueError("force.combine_method must be max or sum.")

    if config["state_machine"]["perturb_interval_min_sec"] > config["state_machine"]["perturb_interval_max_sec"]:
        raise ValueError("perturb_interval_min_sec must be <= perturb_interval_max_sec.")

    limits = config["limits"]
    if limits["safe_grip_min"] < limits["grip_min"] or limits["safe_grip_max"] > limits["grip_max"]:
        raise ValueError("safe grip limits must stay within grip_min and grip_max.")

    for axis in ("x", "y", "z", "roll", "pitch", "yaw"):
        min_key = f"{axis}_min"
        max_key = f"{axis}_max"
        if limits[min_key] > limits[max_key]:
            raise ValueError(f"{min_key} must be <= {max_key}.")

    initial = config["initial_action"]
    initial_to_limits = {
        "x": ("x_min", "x_max"),
        "y": ("y_min", "y_max"),
        "z": ("z_min", "z_max"),
        "roll": ("roll_min", "roll_max"),
        "pitch": ("pitch_min", "pitch_max"),
        "yaw": ("yaw_min", "yaw_max"),
        "gripper_opening": ("grip_min", "grip_max"),
    }
    for key, (min_key, max_key) in initial_to_limits.items():
        value = initial[key]
        if value < limits[min_key] or value > limits[max_key]:
            raise ValueError(
                f"initial_action.{key} must stay within {min_key} and {max_key}."
            )

    if config["experiment"]["trial_duration_sec"] <= 0:
        raise ValueError("trial_duration_sec must be positive.")

    if config["experiment"]["init_timeout_sec"] < 0:
        raise ValueError("init_timeout_sec must be >= 0.")

    if config["experiment"]["initial_settle_steps"] < 0:
        raise ValueError("initial_settle_steps must be >= 0.")

    center_cfg = config.get("center_object", {})
    if center_cfg.get("enabled", False):
        half_extents = center_cfg.get("half_extents", [])
        if len(half_extents) != 3 or any(float(value) <= 0.0 for value in half_extents):
            raise ValueError("center_object.half_extents must contain three positive values.")

        offset_local = center_cfg.get("position_offset_local")
        if offset_local is None:
            offset_local = center_cfg.get("position_offset", [0.0, 0.0, 0.0])
        if len(offset_local) != 3:
            raise ValueError("center_object.position_offset_local must contain exactly three values.")

        if abs(float(center_cfg.get("mass", 0.0))) > 1e-9:
            raise ValueError("center_object.mass must be 0.0 so the centered object stays immovable.")

        settle_steps = int(center_cfg.get("settle_steps", 30))
        if settle_steps < 0:
            raise ValueError("center_object.settle_steps must be >= 0.")


class ExperimentRunner:
    def __init__(self, config, config_path):
        self.config = config
        self.config_path = os.path.abspath(config_path)
        self.config_dir = os.path.dirname(self.config_path)

        self.exp_cfg = config["experiment"]
        self.esp_cfg = config["esp32"]
        self.link_cfg = config["links"]
        self.initial_cfg = config["initial_action"]
        self.keyboard_cfg = config["keyboard"]
        self.limit_cfg = config["limits"]
        self.force_cfg = config["force"]
        self.state_cfg = config["state_machine"]
        self.logging_cfg = config["logging"]
        self.center_cfg = config.get("center_object", {})
        self.initial_settle_steps = int(self.exp_cfg.get("initial_settle_steps", 0))
        self.auto_move_to_initial = bool(self.exp_cfg.get("auto_move_to_initial", False))
        self.manual_start_enabled = bool(self.exp_cfg.get("manual_start_enabled", True))
        self.formal_experiment = bool(self.exp_cfg.get("formal_experiment", False))
        self.trial_duration_sec = float(
            self.exp_cfg.get(
                "trial_duration_sec",
                self.state_cfg.get("trial_duration_sec", 120.0),
            )
        )
        self.init_timeout_sec = float(
            self.exp_cfg.get(
                "init_timeout_sec",
                self.state_cfg.get("init_timeout_sec", 60.0),
            )
        )
        self.log_init_phase = bool(self.logging_cfg.get("log_init_phase", False))
        self.center_object_enabled = bool(self.center_cfg.get("enabled", False))
        self.center_object_settle_steps = int(self.center_cfg.get("settle_steps", 30))

        self.condition = self.exp_cfg["condition"]
        self.rng = random.Random(self.exp_cfg["random_seed"])

        self.output_dir = self.exp_cfg["output_dir"]
        if not os.path.isabs(self.output_dir):
            self.output_dir = os.path.join(self.config_dir, self.output_dir)
        os.makedirs(self.output_dir, exist_ok=True)

        self.timestamp_tag = datetime.now().strftime("%Y%m%d_%H%M%S")
        prefix = (
            f"experiment2_{self.exp_cfg['participant_id']}_"
            f"{self.condition}_{self.timestamp_tag}"
        )
        self.time_series_path = os.path.join(self.output_dir, f"{prefix}_timeseries.csv")
        self.event_path = os.path.join(self.output_dir, f"{prefix}_events.csv")

        self.logger = ExperimentLogger(
            self.time_series_path,
            self.event_path,
            save_time_series=self.logging_cfg["save_time_series_csv"],
            save_event=self.logging_cfg["save_event_csv"],
        )

        self.env = None
        self.feedback_sender = None
        self.center_object_info = None

        self.state = STATE_INIT_PREPARE if self.auto_move_to_initial else STATE_INIT
        self.action = build_action_from_config(config)

        self.sim_time_sec = 0.0
        self.trial_started = False
        self.manual_start = False
        self.trial_start_sim_time_sec = None
        self.initial_pose_reached = not self.auto_move_to_initial
        self.init_start_sim_time_sec = 0.0
        self.init_timeout_prompted = False

        self.next_perturb_time_sec = None
        self.perturb_start_time_sec = None
        self.auto_direction = AUTO_NONE
        self.auto_delta = 0.0

        self.last_user_input_time_sec = None
        self.tf_force_raw = 0.0
        self.if_force_raw = 0.0
        self.tf_force_smooth = 0.0
        self.if_force_smooth = 0.0
        self.tf_contact_count = 0
        self.if_contact_count = 0
        self.contact_force = 0.0
        self.contact_frames = 0
        self.no_contact_frames = 0
        self.contact_hold_time_sec = 0.0
        self.no_contact_hold_time_sec = 0.0
        self.stable_contact = False
        self.stable_no_contact = False
        self.has_contact = False

        self.tf_level = 0
        self.if_level = 0
        self.overall_level = 0
        self.force_error = 0.0
        self.in_target_range = False

        self.prev_range_zone = None
        self.prev_emergency_high = False
        self.prev_user_key_z = False
        self.prev_user_key_x = False
        self.last_sent_levels = (0, 0)

        self.next_sample_time_sec = 0.0
        self.next_print_time_sec = 0.0
        self.debug_text_id = None

        self.stop_requested = False
        self.end_reason = "unknown"
        self.end_event_recorded = False

        self.trial_step_count = 0
        self.contact_force_total = 0.0
        self.force_error_total = 0.0
        self.in_target_step_count = 0
        self.trial_event_counts = {}

    def schedule_next_perturb(self):
        delay = self.rng.uniform(
            self.state_cfg["perturb_interval_min_sec"],
            self.state_cfg["perturb_interval_max_sec"],
        )
        return self.sim_time_sec + max(self.state_cfg["auto_wait_sec"], delay)

    def current_trial_elapsed_sec(self):
        if not self.trial_started or self.trial_start_sim_time_sec is None:
            return -1.0
        return max(0.0, self.sim_time_sec - self.trial_start_sim_time_sec)

    def current_timestamp_iso(self):
        return now_iso()

    def in_init_phase(self):
        return self.state in (STATE_INIT_PREPARE, STATE_INIT)

    def record_event(self, event_type, detail=""):
        event_id = self.logger.record_event(
            event_type=event_type,
            timestamp_iso=self.current_timestamp_iso(),
            time_sec=self.sim_time_sec,
            trial_elapsed_sec=self.current_trial_elapsed_sec(),
            participant_id=self.exp_cfg["participant_id"],
            condition=self.condition,
            formal_experiment=self.formal_experiment,
            state=self.state,
            auto_direction=self.auto_direction,
            contact_force=self.contact_force,
            gripper_opening=float(self.action[6]),
            detail=detail,
        )
        if self.trial_started:
            self.trial_event_counts[event_type] = self.trial_event_counts.get(event_type, 0) + 1
        return event_id

    def start_trial(self):
        self.manual_start = True
        self.trial_started = True
        self.trial_start_sim_time_sec = self.sim_time_sec
        self.state = STATE_AUTO_WAIT
        self.next_perturb_time_sec = self.schedule_next_perturb()
        self.record_event(
            "MANUAL_START",
            detail="manual start from INIT after operator confirmed initial pose",
        )
        self.record_trial_start_baseline_events()

    def record_trial_start_baseline_events(self):
        if self.contact_force < self.force_cfg["target_low"]:
            self.prev_range_zone = "low"
            self.record_event("FORCE_BELOW_LOW")
        elif self.contact_force > self.force_cfg["target_high"]:
            self.prev_range_zone = "high"
            self.record_event("FORCE_ABOVE_HIGH")
        else:
            self.prev_range_zone = "in"

        emergency_high = self.contact_force >= self.force_cfg["emergency_high"]
        if emergency_high:
            self.record_event("EMERGENCY_HIGH_FORCE")
        self.prev_emergency_high = emergency_high

    def enter_auto_wait(self, detail=""):
        if self.state in (STATE_AUTO_PERTURB_CLOSE, STATE_AUTO_PERTURB_OPEN):
            self.record_event("AUTO_PERTURB_END", detail=detail)
        self.state = STATE_AUTO_WAIT
        self.auto_direction = AUTO_NONE
        self.auto_delta = 0.0
        self.perturb_start_time_sec = None
        self.next_perturb_time_sec = self.schedule_next_perturb()

    def start_auto_perturb(self, direction):
        self.auto_direction = direction
        self.auto_delta = 0.0
        self.perturb_start_time_sec = self.sim_time_sec
        if direction == AUTO_CLOSE:
            self.state = STATE_AUTO_PERTURB_CLOSE
            self.record_event("AUTO_PERTURB_START_CLOSE")
        else:
            self.state = STATE_AUTO_PERTURB_OPEN
            self.record_event("AUTO_PERTURB_START_OPEN")

    def choose_auto_direction(self):
        options = []
        if (
            self.state_cfg["allow_auto_close"]
            and self.contact_force < self.force_cfg["emergency_high"]
            and self.action[6] > self.limit_cfg["safe_grip_min"]
        ):
            options.append(AUTO_CLOSE)

        if (
            self.state_cfg["allow_auto_open"]
            and self.action[6] < self.limit_cfg["safe_grip_max"]
            and not self.stable_no_contact
        ):
            options.append(AUTO_OPEN)

        if not options:
            return AUTO_NONE

        return self.rng.choice(options)

    def setup(self):
        self.env = create_environment()

        client = None
        if self.esp_cfg["send_to_esp32"]:
            client = ESP32FeedbackClient(self.esp_cfg["ip"], self.esp_cfg["port"])

        self.feedback_sender = FeedbackSender(
            client=client,
            feedback_period_sec=self.esp_cfg["feedback_send_period_sec"],
            keepalive_period_sec=max(
                self.esp_cfg["feedback_send_period_sec"],
                self.esp_cfg["keepalive_send_period_sec"],
            ),
        )

        self.record_event("EXPERIMENT_START", detail=f"condition={self.condition}")
        self.last_sent_levels = self.feedback_sender.maybe_send(0, 0, self.sim_time_sec)

        if self.auto_move_to_initial:
            self.state = STATE_INIT_PREPARE
            self.record_event("INITIAL_POSE_START", detail="move_to_initial_pose")
            elapsed_sec, executed_steps = move_to_initial_pose(
                self.env,
                self.action,
                self.initial_settle_steps,
                sim_dt=SIM_DT,
            )
            self.sim_time_sec += elapsed_sec
            self.last_sent_levels = self.feedback_sender.maybe_send(0, 0, self.sim_time_sec)
            self.initial_pose_reached = True
        else:
            self.initial_pose_reached = True

        if self.center_object_enabled:
            self.center_object_info = create_centered_contact_object(
                self.env,
                self.link_cfg["tf_link"],
                self.link_cfg["if_link"],
                self.center_cfg,
            )
            elapsed_sec, settle_steps = step_simulation_for_steps(
                self.env,
                self.center_object_settle_steps,
                sim_dt=SIM_DT,
            )
            self.sim_time_sec += elapsed_sec
            self.last_sent_levels = self.feedback_sender.maybe_send(0, 0, self.sim_time_sec)
            print(
                "Centered contact object created: "
                f"id={self.center_object_info['body_id']}, "
                f"center={self.center_object_info['center_pos']}, "
                f"half_extents={self.center_object_info['half_extents']}, "
                f"settle_steps={settle_steps}"
            )

        self.state = STATE_INIT
        if self.auto_move_to_initial:
            detail = f"executed_steps={executed_steps}"
            if self.center_object_info is not None:
                detail += f"; centered_box_id={self.center_object_info['body_id']}"
            self.record_event(
                "INITIAL_POSE_DONE",
                detail=detail,
            )

        self.init_start_sim_time_sec = self.sim_time_sec
        print(f"Config: {self.config_path}")
        print(f"Time-series CSV: {self.time_series_path}")
        print(f"Event CSV: {self.event_path}")
        print(f"Formal experiment: {self.formal_experiment}")
        self.print_instructions()

    def print_instructions(self):
        print("\nKeyboard controls:")
        print("  I / K   : x + / x -")
        print("  J / L   : y - / y +")
        print("  U / O   : z + / z -")
        print("  1 / 2   : roll - / roll +")
        print("  3 / 4   : pitch - / pitch +")
        print("  5 / 6   : yaw - / yaw +")
        print("  Z / X   : step close / step open gripper")
        print("  P       : print current contact details")
        if self.keyboard_cfg["allow_condition_toggle"]:
            print("  C       : toggle condition")
        if self.manual_start_enabled:
            print("  S       : start formal trial from INIT")
        print("  Q       : quit")
        print()

    def apply_motion_keys(self, keys):
        pos_step = self.keyboard_cfg["pos_step"]
        rot_step = self.keyboard_cfg["rot_step"]

        if key_down(keys, ord("i")) or key_down(keys, ord("I")):
            self.action[0] += pos_step
        if key_down(keys, ord("k")) or key_down(keys, ord("K")):
            self.action[0] -= pos_step

        if key_down(keys, ord("l")) or key_down(keys, ord("L")):
            self.action[1] += pos_step
        if key_down(keys, ord("j")) or key_down(keys, ord("J")):
            self.action[1] -= pos_step

        if key_down(keys, ord("u")) or key_down(keys, ord("U")):
            self.action[2] += pos_step
        if key_down(keys, ord("o")) or key_down(keys, ord("O")):
            self.action[2] -= pos_step

        if key_down(keys, ord("1")):
            self.action[3] -= rot_step
        if key_down(keys, ord("2")):
            self.action[3] += rot_step

        if key_down(keys, ord("3")):
            self.action[4] -= rot_step
        if key_down(keys, ord("4")):
            self.action[4] += rot_step

        if key_down(keys, ord("5")):
            self.action[5] -= rot_step
        if key_down(keys, ord("6")):
            self.action[5] += rot_step

    def clip_action(self):
        self.action[0] = np.clip(self.action[0], self.limit_cfg["x_min"], self.limit_cfg["x_max"])
        self.action[1] = np.clip(self.action[1], self.limit_cfg["y_min"], self.limit_cfg["y_max"])
        self.action[2] = np.clip(self.action[2], self.limit_cfg["z_min"], self.limit_cfg["z_max"])
        self.action[3] = np.clip(self.action[3], self.limit_cfg["roll_min"], self.limit_cfg["roll_max"])
        self.action[4] = np.clip(self.action[4], self.limit_cfg["pitch_min"], self.limit_cfg["pitch_max"])
        self.action[5] = np.clip(self.action[5], self.limit_cfg["yaw_min"], self.limit_cfg["yaw_max"])
        self.action[6] = np.clip(self.action[6], self.limit_cfg["grip_min"], self.limit_cfg["grip_max"])

    def update_contact_metrics(self):
        (
            tf_force_sum,
            _tf_max_force,
            tf_contact_count,
            if_force_sum,
            _if_max_force,
            if_contact_count,
        ) = read_separate_fingertip_forces(
            self.env,
            self.link_cfg["tf_link"],
            self.link_cfg["if_link"],
        )

        alpha = self.force_cfg["smoothing_alpha"]
        self.tf_force_raw = tf_force_sum
        self.if_force_raw = if_force_sum
        self.tf_contact_count = tf_contact_count
        self.if_contact_count = if_contact_count

        self.tf_force_smooth = (1.0 - alpha) * self.tf_force_smooth + alpha * self.tf_force_raw
        self.if_force_smooth = (1.0 - alpha) * self.if_force_smooth + alpha * self.if_force_raw

        if self.force_cfg["combine_method"] == "max":
            self.contact_force = max(self.tf_force_smooth, self.if_force_smooth)
        else:
            self.contact_force = self.tf_force_smooth + self.if_force_smooth

        self.has_contact = self.contact_force > self.force_cfg["contact_threshold"]

        if self.has_contact:
            self.contact_frames += 1
            self.no_contact_frames = 0
            self.contact_hold_time_sec += SIM_DT
            self.no_contact_hold_time_sec = 0.0
        else:
            self.no_contact_frames += 1
            self.contact_frames = 0
            self.no_contact_hold_time_sec += SIM_DT
            self.contact_hold_time_sec = 0.0

        self.stable_contact = self.contact_hold_time_sec >= self.force_cfg["contact_hold_sec"]
        self.stable_no_contact = self.no_contact_hold_time_sec >= self.force_cfg["no_contact_hold_sec"]

        thresholds = self.force_cfg["level_thresholds"]
        self.tf_level = force_to_level(self.tf_force_smooth, thresholds)
        self.if_level = force_to_level(self.if_force_smooth, thresholds)
        self.overall_level = max(self.tf_level, self.if_level)

        self.in_target_range = (
            self.force_cfg["target_low"]
            <= self.contact_force
            <= self.force_cfg["target_high"]
        )
        target_mid = (self.force_cfg["target_low"] + self.force_cfg["target_high"]) / 2.0
        self.force_error = abs(self.contact_force - target_mid)

    def update_range_events(self):
        if not self.trial_started:
            return

        if self.contact_force < self.force_cfg["target_low"]:
            current_zone = "low"
        elif self.contact_force > self.force_cfg["target_high"]:
            current_zone = "high"
        else:
            current_zone = "in"

        if self.prev_range_zone is None:
            self.prev_range_zone = current_zone
        elif current_zone != self.prev_range_zone:
            if current_zone == "high":
                self.record_event("FORCE_ABOVE_HIGH")
            elif current_zone == "low":
                self.record_event("FORCE_BELOW_LOW")
            elif current_zone == "in" and self.prev_range_zone in ("low", "high"):
                self.record_event("FORCE_BACK_IN_RANGE")
            self.prev_range_zone = current_zone

        emergency_high = self.contact_force >= self.force_cfg["emergency_high"]
        if emergency_high and not self.prev_emergency_high:
            self.record_event("EMERGENCY_HIGH_FORCE")
        self.prev_emergency_high = emergency_high

    def update_init_state(self, manual_start_requested):
        if self.state != STATE_INIT:
            return

        init_elapsed_sec = self.sim_time_sec - self.init_start_sim_time_sec
        if (
            not self.init_timeout_prompted
            and init_elapsed_sec >= self.init_timeout_sec
        ):
            print(
                "\nINIT timeout. Press S to start manually after confirming pose, "
                "or Q to quit."
            )
            self.init_timeout_prompted = True

        if manual_start_requested and self.manual_start_enabled:
            self.start_trial()

    def apply_user_or_auto_grip(self, user_key_z, user_key_x):
        grip_delta = 0.0

        if user_key_z:
            grip_delta -= self.keyboard_cfg["user_grip_step"]
        if user_key_x:
            grip_delta += self.keyboard_cfg["user_grip_step"]

        user_input = user_key_z or user_key_x

        if self.trial_started and user_input:
            self.last_user_input_time_sec = self.sim_time_sec
            if self.state != STATE_USER_CONTROL:
                if self.state in (STATE_AUTO_PERTURB_CLOSE, STATE_AUTO_PERTURB_OPEN):
                    self.enter_auto_wait(detail="user_input")
                self.state = STATE_USER_CONTROL
        elif self.trial_started and not user_input and self.state == STATE_USER_CONTROL:
            self.state = STATE_AUTO_WAIT
            self.next_perturb_time_sec = self.schedule_next_perturb()

        if self.trial_started and not user_input:
            if self.state == STATE_AUTO_WAIT:
                if (
                    self.next_perturb_time_sec is not None
                    and self.sim_time_sec >= self.next_perturb_time_sec
                ):
                    direction = self.choose_auto_direction()
                    if direction == AUTO_NONE:
                        self.next_perturb_time_sec = self.schedule_next_perturb()
                    else:
                        self.start_auto_perturb(direction)

            if self.state == STATE_AUTO_PERTURB_CLOSE:
                if (
                    self.contact_force >= self.force_cfg["emergency_high"]
                    or self.action[6] <= self.limit_cfg["safe_grip_min"]
                    or not self.state_cfg["allow_auto_close"]
                ):
                    self.enter_auto_wait(detail="close_safety_stop")
                else:
                    grip_delta -= self.state_cfg["auto_grip_speed"] * SIM_DT

            elif self.state == STATE_AUTO_PERTURB_OPEN:
                if (
                    self.action[6] >= self.limit_cfg["safe_grip_max"]
                    or not self.state_cfg["allow_auto_open"]
                    or self.stable_no_contact
                ):
                    self.enter_auto_wait(detail="open_safety_stop")
                else:
                    grip_delta += self.state_cfg["auto_grip_speed"] * SIM_DT

        old_grip = float(self.action[6])
        self.action[6] += grip_delta
        self.clip_action()
        self.auto_delta += float(self.action[6]) - old_grip

        if self.state in (STATE_AUTO_PERTURB_CLOSE, STATE_AUTO_PERTURB_OPEN):
            if (
                self.perturb_start_time_sec is not None
                and (self.sim_time_sec - self.perturb_start_time_sec) >= self.state_cfg["perturb_duration_sec"]
            ):
                self.enter_auto_wait(detail="perturb_duration_reached")
            elif abs(self.auto_delta) >= self.state_cfg["max_auto_delta"]:
                self.enter_auto_wait(detail="max_auto_delta_reached")

        return user_input

    def build_desired_feedback(self):
        if self.condition == "visual_only":
            return (0, 0)
        return (self.tf_level, self.if_level)

    def update_gui_text(self):
        if not self.logging_cfg["show_gui_force_text"]:
            return

        text = (
            f"state: {self.state}\n"
            f"trial_elapsed: {self.current_trial_elapsed_sec():.2f}\n"
            f"contact_force: {self.contact_force:.3f}\n"
            f"in_target: {self.in_target_range}\n"
            f"tf_level: {self.tf_level}\n"
            f"if_level: {self.if_level}\n"
            f"condition: {self.condition}"
        )

        if self.debug_text_id is None:
            self.debug_text_id = p.addUserDebugText(
                text,
                textPosition=[-0.45, -0.45, 0.7],
                textColorRGB=[1, 0, 0],
                textSize=1.2,
            )
        else:
            self.debug_text_id = p.addUserDebugText(
                text,
                textPosition=[-0.45, -0.45, 0.7],
                textColorRGB=[1, 0, 0],
                textSize=1.2,
                replaceItemUniqueId=self.debug_text_id,
            )

    def print_status(self):
        if self.sim_time_sec < self.next_print_time_sec:
            return

        self.next_print_time_sec = self.sim_time_sec + self.logging_cfg["print_period_sec"]
        print(
            f"\rstate={self.state:>18} "
            f"trial={self.current_trial_elapsed_sec():6.2f}s "
            f"force={self.contact_force:7.3f} "
            f"target=[{self.force_cfg['target_low']:.1f},{self.force_cfg['target_high']:.1f}] "
            f"grip={self.action[6]:.4f} "
            f"cond={self.condition:>13} ",
            end="",
            flush=True,
        )

    def write_time_series_row(self, user_key_z, user_key_x, user_input, force=False):
        if not force and self.sim_time_sec < self.next_sample_time_sec:
            return

        if self.in_init_phase() and not self.log_init_phase:
            self.logger.consume_pending_event_ids()
            return

        self.next_sample_time_sec = self.sim_time_sec + self.logging_cfg["sample_period_sec"]
        target_mid = (self.force_cfg["target_low"] + self.force_cfg["target_high"]) / 2.0
        event_ids = self.logger.consume_pending_event_ids()

        row = {
            "timestamp_iso": self.current_timestamp_iso(),
            "time_sec": f"{self.sim_time_sec:.6f}",
            "trial_elapsed_sec": f"{self.current_trial_elapsed_sec():.6f}",
            "participant_id": self.exp_cfg["participant_id"],
            "condition": self.condition,
            "formal_experiment": to_int_flag(self.formal_experiment),
            "manual_start": to_int_flag(self.manual_start),
            "init_phase": to_int_flag(self.in_init_phase()),
            "initial_pose_reached": to_int_flag(self.initial_pose_reached),
            "state": self.state,
            "event_id": event_ids,
            "auto_direction": self.auto_direction,
            "auto_active": to_int_flag(
                self.state in (STATE_AUTO_PERTURB_CLOSE, STATE_AUTO_PERTURB_OPEN)
            ),
            "auto_delta": f"{self.auto_delta:.6f}",
            "x": f"{self.action[0]:.6f}",
            "y": f"{self.action[1]:.6f}",
            "z": f"{self.action[2]:.6f}",
            "roll": f"{self.action[3]:.6f}",
            "pitch": f"{self.action[4]:.6f}",
            "yaw": f"{self.action[5]:.6f}",
            "gripper_opening": f"{self.action[6]:.6f}",
            "user_key_z": to_int_flag(user_key_z),
            "user_key_x": to_int_flag(user_key_x),
            "user_input": to_int_flag(user_input),
            "tf_force_raw": f"{self.tf_force_raw:.6f}",
            "if_force_raw": f"{self.if_force_raw:.6f}",
            "tf_force_smooth": f"{self.tf_force_smooth:.6f}",
            "if_force_smooth": f"{self.if_force_smooth:.6f}",
            "tf_contact_count": self.tf_contact_count,
            "if_contact_count": self.if_contact_count,
            "contact_force": f"{self.contact_force:.6f}",
            "contact_frames": self.contact_frames,
            "no_contact_frames": self.no_contact_frames,
            "contact_hold_sec": f"{self.contact_hold_time_sec:.6f}",
            "no_contact_hold_sec": f"{self.no_contact_hold_time_sec:.6f}",
            "stable_contact": to_int_flag(self.stable_contact),
            "stable_no_contact": to_int_flag(self.stable_no_contact),
            "tf_level": self.tf_level,
            "if_level": self.if_level,
            "overall_level": self.overall_level,
            "target_low": f"{self.force_cfg['target_low']:.6f}",
            "target_high": f"{self.force_cfg['target_high']:.6f}",
            "target_mid": f"{target_mid:.6f}",
            "force_error": f"{self.force_error:.6f}",
            "in_target_range": to_int_flag(self.in_target_range),
            "esp32_tf_sent": self.last_sent_levels[0],
            "esp32_if_sent": self.last_sent_levels[1],
        }
        self.logger.write_time_series(row)

    def update_summary_accumulators(self):
        if not self.trial_started:
            return
        self.trial_step_count += 1
        self.contact_force_total += self.contact_force
        self.force_error_total += self.force_error
        if self.in_target_range:
            self.in_target_step_count += 1

    def finalize_if_trial_complete(self):
        if not self.trial_started:
            return
        if self.current_trial_elapsed_sec() >= self.trial_duration_sec:
            self.stop_requested = True
            self.end_reason = "trial_duration_reached"

    def maybe_toggle_condition(self, keys):
        if not self.keyboard_cfg["allow_condition_toggle"]:
            return
        if key_triggered(keys, ord("c")) or key_triggered(keys, ord("C")):
            self.condition = (
                "vision_haptic" if self.condition == "visual_only" else "visual_only"
            )
            print(f"\nCondition switched to: {self.condition}\n")

    def run(self):
        self.setup()
        try:
            while p.isConnected() and not self.stop_requested:
                keys = p.getKeyboardEvents()

                if key_triggered(keys, ord("q")) or key_triggered(keys, ord("Q")):
                    self.end_reason = "user_quit"
                    self.stop_requested = True
                    break

                if key_triggered(keys, ord("p")) or key_triggered(keys, ord("P")):
                    print_current_contacts(self.env)

                self.maybe_toggle_condition(keys)

                manual_start_requested = key_triggered(keys, ord("s")) or key_triggered(
                    keys, ord("S")
                )

                self.apply_motion_keys(keys)

                user_key_z_held = key_down(keys, ord("z")) or key_down(keys, ord("Z"))
                user_key_x_held = key_down(keys, ord("x")) or key_down(keys, ord("X"))
                user_key_z = user_key_z_held and not self.prev_user_key_z
                user_key_x = user_key_x_held and not self.prev_user_key_x

                if user_key_z:
                    self.record_event("USER_INPUT_Z")
                if user_key_x:
                    self.record_event("USER_INPUT_X")

                user_input = self.apply_user_or_auto_grip(user_key_z, user_key_x)
                self.prev_user_key_z = user_key_z_held
                self.prev_user_key_x = user_key_x_held

                self.clip_action()
                self.env.robot.move_ee(self.action[:6], control_method="end")
                self.env.robot.move_gripper(self.action[6])
                safe_step_simulation(self.env)
                self.sim_time_sec += SIM_DT

                self.update_contact_metrics()

                if not self.trial_started:
                    self.update_init_state(manual_start_requested)

                self.update_range_events()

                if self.state == STATE_AUTO_PERTURB_CLOSE and self.contact_force >= self.force_cfg["emergency_high"]:
                    self.enter_auto_wait(detail="emergency_high_force")

                if self.state == STATE_AUTO_PERTURB_OPEN and self.stable_no_contact:
                    self.enter_auto_wait(detail="stable_no_contact")

                desired_levels = self.build_desired_feedback()
                self.last_sent_levels = self.feedback_sender.maybe_send(
                    desired_levels[0],
                    desired_levels[1],
                    self.sim_time_sec,
                )

                self.update_summary_accumulators()
                self.write_time_series_row(user_key_z, user_key_x, user_input)
                self.update_gui_text()
                self.print_status()
                self.finalize_if_trial_complete()

            if not p.isConnected():
                self.end_reason = "pybullet_disconnected"

        finally:
            self.shutdown()

    def print_summary(self):
        active_duration = self.current_trial_elapsed_sec()
        if active_duration < 0:
            active_duration = 0.0
        avg_contact_force = (
            self.contact_force_total / self.trial_step_count
            if self.trial_step_count > 0
            else 0.0
        )
        avg_force_error = (
            self.force_error_total / self.trial_step_count
            if self.trial_step_count > 0
            else 0.0
        )
        in_target_ratio = (
            self.in_target_step_count / self.trial_step_count
            if self.trial_step_count > 0
            else 0.0
        )

        print("\n\nExperiment 2 summary")
        print(f"  end_reason: {self.end_reason}")
        print(f"  active_trial_duration_sec: {active_duration:.3f}")
        print(f"  avg_contact_force: {avg_contact_force:.3f}")
        print(f"  avg_force_error: {avg_force_error:.3f}")
        print(f"  in_target_range_ratio: {in_target_ratio:.3f}")
        print(
            f"  FORCE_ABOVE_HIGH count: "
            f"{self.trial_event_counts.get('FORCE_ABOVE_HIGH', 0)}"
        )
        print(
            f"  FORCE_BELOW_LOW count: "
            f"{self.trial_event_counts.get('FORCE_BELOW_LOW', 0)}"
        )
        print(
            f"  USER_INPUT_Z count: "
            f"{self.trial_event_counts.get('USER_INPUT_Z', 0)}"
        )
        print(
            f"  USER_INPUT_X count: "
            f"{self.trial_event_counts.get('USER_INPUT_X', 0)}"
        )
        print(
            f"  auto_close_count: "
            f"{self.trial_event_counts.get('AUTO_PERTURB_START_CLOSE', 0)}"
        )
        print(
            f"  auto_open_count: "
            f"{self.trial_event_counts.get('AUTO_PERTURB_START_OPEN', 0)}"
        )

    def shutdown(self):
        if not self.end_event_recorded:
            try:
                self.record_event("EXPERIMENT_END", detail=self.end_reason)
                self.end_event_recorded = True
            except Exception:
                pass

        if self.feedback_sender is not None:
            try:
                self.last_sent_levels = self.feedback_sender.shutdown(self.sim_time_sec)
            except Exception as exc:
                print(f"\nFeedback shutdown error: {exc}")

        self.write_time_series_row(False, False, False, force=True)

        if self.logger is not None:
            self.logger.close()

        if self.env is not None and p.isConnected():
            try:
                self.env.close()
            except Exception:
                try:
                    p.disconnect()
                except Exception:
                    pass

        self.print_summary()


def main():
    ensure_runtime_dependencies()

    if len(sys.argv) < 2:
        print("Usage: python experiment2_force_maintenance.py experiment2_config.yaml")
        sys.exit(1)

    config_path = sys.argv[1]
    if not os.path.isabs(config_path):
        config_path = os.path.join(os.getcwd(), config_path)
    if not os.path.exists(config_path):
        print(f"Config file not found: {config_path}")
        sys.exit(1)

    project_root = os.path.dirname(os.path.abspath(__file__))
    if os.getcwd() != project_root:
        print(f"Switching working directory to project root: {project_root}")
        os.chdir(project_root)

    try:
        config = load_config(config_path)
        validate_config(config)
    except Exception as exc:
        print(f"Failed to load config: {exc}")
        sys.exit(1)

    print("Starting Experiment 2...")
    print(f"Project root: {project_root}")
    print(f"Config path: {config_path}")

    try:
        runner = ExperimentRunner(config, config_path)
        runner.run()
    except Exception as exc:
        print(f"Experiment 2 crashed before startup completed: {exc}")
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
