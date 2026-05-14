"""
Experiment 1 — Contact Judgment Task
====================================
Participants judge whether the gripper is contacting the object under two
conditions: visual-only vs. visual + haptic (ESP32 vibrotactile feedback).

Usage
-----
  python experiment1_contact_judgment.py <participant_id> <condition>

  participant_id : any string (e.g. P01, P02)
  condition      : "visual_only"  → ESP32 always receives (0, 0)
                   "vision_haptic" → ESP32 receives real fingertip levels

During the experiment
---------------------
  - Position / orientation / gripper keys  →  same as forcelevel_simulation.py
  - SPACE  →  trigger a trial (record contact judgment)
  - C      →  toggle condition on-the-fly
  - P      →  print current contact links (debug)
  - Q      →  quit
"""

import csv
import math
import os
import socket
import sys
import time
from datetime import datetime

import numpy as np
import pybullet as p

from env import ClutteredPushGrasp
from robot import UR5Robotiq85

# ---------------------------------------------------------------------------
# Different versions of this repository may use different class names.
# ---------------------------------------------------------------------------
try:
    from utilities import YCBModels
except ImportError:
    from utilities import Models as YCBModels

# ---------------------------------------------------------------------------
#  Constants
# ---------------------------------------------------------------------------
TF_LINK = 12  # left_inner_finger_pad
IF_LINK = 17  # right_inner_finger_pad

ESP32_IP = "192.168.1.16"
ESP32_PORT = 12345
SEND_TO_ESP32 = True

MAGIC = b"\xAA\x55\xAA\x55"

KEY_IS_DOWN = getattr(p, "KEY_IS_DOWN", 1)
KEY_WAS_TRIGGERED = getattr(p, "KEY_WAS_TRIGGERED", 2)

# ---------------------------------------------------------------------------
#  Keyboard helpers
# ---------------------------------------------------------------------------

def key_down(keys, key):
    return keys.get(key, 0) & KEY_IS_DOWN


def key_triggered(keys, key):
    return keys.get(key, 0) & KEY_WAS_TRIGGERED

# ---------------------------------------------------------------------------
#  Simulation helpers
# ---------------------------------------------------------------------------

def safe_step_simulation(env):
    """Step the simulation using env.step_simulation() or p.stepSimulation()."""
    if hasattr(env, "step_simulation"):
        env.step_simulation()
    else:
        p.stepSimulation()


def get_link_name(robot_id, link_index):
    """Return the link name for a given robot link index."""
    if link_index < 0:
        return "base"
    try:
        return p.getJointInfo(robot_id, link_index)[12].decode("utf-8")
    except Exception:
        return f"unknown_link_{link_index}"


def print_current_contacts(env):
    """Print all current contact points between the robot and the target object."""
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


def read_separate_fingertip_forces(env):
    """
    Read left and right fingertip contact forces separately.

    Returns
    -------
    tf_force_sum, tf_max_force, tf_contact_count,
    if_force_sum, if_max_force, if_contact_count
    """
    contacts = p.getContactPoints(bodyA=env.robot.id, bodyB=env.boxID)

    tf_forces = []
    if_forces = []

    for contact in contacts:
        robot_link_index = contact[3]
        normal_force = contact[9]
        if robot_link_index == TF_LINK:
            tf_forces.append(normal_force)
        elif robot_link_index == IF_LINK:
            if_forces.append(normal_force)

    tf_force_sum = float(sum(tf_forces)) if tf_forces else 0.0
    if_force_sum = float(sum(if_forces)) if if_forces else 0.0

    tf_max_force = float(max(tf_forces)) if tf_forces else 0.0
    if_max_force = float(max(if_forces)) if if_forces else 0.0

    tf_contact_count = len(tf_forces)
    if_contact_count = len(if_forces)

    return (
        tf_force_sum, tf_max_force, tf_contact_count,
        if_force_sum, if_max_force, if_contact_count,
    )


def force_to_level(force):
    """
    Convert simulated fingertip contact force to four feedback levels: 0–3.
    """
    if force < 5.0:
        return 0
    elif force < 50.0:
        return 1
    elif force < 100.0:
        return 2
    else:
        return 3

# ---------------------------------------------------------------------------
#  Environment factory
# ---------------------------------------------------------------------------

def create_environment():
    """Create the PyBullet UR5 Robotiq environment."""
    ycb_models = YCBModels(
        os.path.join("./data/ycb", "**", "textured-decmp.obj")
    )
    camera = None
    robot = UR5Robotiq85((0, 0.5, 0), (0, 0, 0))
    env = ClutteredPushGrasp(robot, ycb_models, camera, vis=True)
    env.reset()
    return env

# ---------------------------------------------------------------------------
#  ESP32 communication
# ---------------------------------------------------------------------------

def generate_packet(payload):
    """
    Packet format:
    [AA 55 AA 55] [payload_len] [payload...] [checksum]
    """
    payload = bytearray(payload)
    checksum = sum(payload) & 0xFF
    packet = bytearray(MAGIC)
    packet.append(len(payload))
    packet.extend(payload)
    packet.append(checksum)
    return packet


class ESP32FeedbackClient:
    """TCP client that sends vibrotactile feedback levels to an ESP32."""

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
        except Exception as e:
            print(f"Failed to connect to ESP32: {e}")
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
        except Exception as e:
            print(f"ESP32 send failed: {e}")
            self.sock = None

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

# ---------------------------------------------------------------------------
#  Trial recorder
# ---------------------------------------------------------------------------

class TrialRecorder:
    """
    Appends one row per trial to a CSV file.

    Columns
    -------
    participant_id, trial_id, condition, true_contact, response, correct,
    tf_force_smooth, if_force_smooth, tf_level, if_level, overall_level,
    timestamp
    """

    def __init__(self, csv_path: str):
        self.csv_path = csv_path
        self.trial_count = 0
        file_exists = os.path.exists(csv_path)
        self._file = open(csv_path, "a", newline="")
        self._writer = csv.writer(self._file)
        if not file_exists:
            self._writer.writerow([
                "participant_id", "trial_id", "condition",
                "true_contact", "response", "correct",
                "tf_force_smooth", "if_force_smooth",
                "tf_level", "if_level", "overall_level",
                "timestamp",
            ])
            self._file.flush()

    def record_trial(
        self,
        participant_id: str,
        condition: str,
        true_contact: int,
        response: int,
        tf_force_smooth: float,
        if_force_smooth: float,
        tf_level: int,
        if_level: int,
    ):
        self.trial_count += 1
        correct = 1 if response == true_contact else 0
        overall_level = max(tf_level, if_level)
        timestamp = datetime.now().isoformat()

        self._writer.writerow([
            participant_id,
            self.trial_count,
            condition,
            true_contact,
            response,
            correct,
            f"{tf_force_smooth:.3f}",
            f"{if_force_smooth:.3f}",
            tf_level,
            if_level,
            overall_level,
            timestamp,
        ])
        self._file.flush()

    def close(self):
        self._file.close()

# ---------------------------------------------------------------------------
#  Main experiment loop
# ---------------------------------------------------------------------------

def experiment1(participant_id: str, initial_condition: str):
    # --- create the simulation ------------------------------------------------
    env = create_environment()

    # --- ESP32 ----------------------------------------------------------------
    esp_client = None
    if SEND_TO_ESP32:
        esp_client = ESP32FeedbackClient(ESP32_IP, ESP32_PORT)

    # --- CSV recorder ---------------------------------------------------------
    csv_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        f"experiment1_{participant_id}.csv",
    )
    recorder = TrialRecorder(csv_path)
    print(f"Trial data will be saved to: {csv_path}")

    # --- experiment state -----------------------------------------------------
    condition = initial_condition

    # --- initial action -------------------------------------------------------
    # x, y, z, roll, pitch, yaw, gripper_opening_length
    action = np.array([
        0.0,            # x
        0.0,            # y
        0.5,            # z
        0.0,            # roll
        math.pi / 2,    # pitch
        math.pi / 2,    # yaw
        0.04,           # gripper opening length
    ], dtype=float)

    # --- control step sizes ---------------------------------------------------
    pos_step = 0.003
    rot_step = 0.02
    grip_step = 0.0015

    # --- safety limits --------------------------------------------------------
    x_min, x_max = -0.224, 0.224
    y_min, y_max = -0.224, 0.224
    z_min, z_max = 0.05, 1.0
    grip_min, grip_max = 0.0, 0.085

    # --- force smoothing ------------------------------------------------------
    tf_force_smooth = 0.0
    if_force_smooth = 0.0
    smoothing_alpha = 0.15

    # --- periodic print state -------------------------------------------------
    last_print_time = time.time()

    # --- print instructions ---------------------------------------------------
    print(f"\n{'='*60}")
    print(f"Experiment 1 — Contact Judgment Task")
    print(f"{'='*60}")
    print(f"Participant: {participant_id}")
    print(f"Condition:   {condition}")
    print(f"{'='*60}")
    print("\nKeyboard controls:")
    print("  I / K   :  x + / x -")
    print("  J / L   :  y - / y +")
    print("  U / O   :  z + / z -")
    print("  1 / 2   :  roll - / roll +")
    print("  3 / 4   :  pitch - / pitch +")
    print("  5 / 6   :  yaw - / yaw +")
    print("  Z / X   :  close / open gripper")
    print("  SPACE   :  trigger a trial (record contact judgment)")
    print("  C       :  toggle condition (visual_only <-> vision_haptic)")
    print("  P       :  print current contact details (debug)")
    print("  Q       :  quit")
    print()

    try:
        while True:
            keys = p.getKeyboardEvents()

            # --- quit ---------------------------------------------------------
            if key_triggered(keys, ord("q")) or key_triggered(keys, ord("Q")):
                break

            # --- print contacts (debug) ---------------------------------------
            if key_triggered(keys, ord("p")) or key_triggered(keys, ord("P")):
                print_current_contacts(env)

            # --- toggle condition ---------------------------------------------
            if key_triggered(keys, ord("c")) or key_triggered(keys, ord("C")):
                if condition == "visual_only":
                    condition = "vision_haptic"
                else:
                    condition = "visual_only"
                print(f"\n>>> Condition switched to: {condition} <<<\n")

            # --- position control ---------------------------------------------
            if key_down(keys, ord("i")) or key_down(keys, ord("I")):
                action[0] += pos_step
            if key_down(keys, ord("k")) or key_down(keys, ord("K")):
                action[0] -= pos_step

            if key_down(keys, ord("l")) or key_down(keys, ord("L")):
                action[1] += pos_step
            if key_down(keys, ord("j")) or key_down(keys, ord("J")):
                action[1] -= pos_step

            if key_down(keys, ord("u")) or key_down(keys, ord("U")):
                action[2] += pos_step
            if key_down(keys, ord("o")) or key_down(keys, ord("O")):
                action[2] -= pos_step

            # --- orientation control ------------------------------------------
            if key_down(keys, ord("1")):
                action[3] -= rot_step
            if key_down(keys, ord("2")):
                action[3] += rot_step

            if key_down(keys, ord("3")):
                action[4] -= rot_step
            if key_down(keys, ord("4")):
                action[4] += rot_step

            if key_down(keys, ord("5")):
                action[5] -= rot_step
            if key_down(keys, ord("6")):
                action[5] += rot_step

            # --- gripper control ----------------------------------------------
            if key_down(keys, ord("z")) or key_down(keys, ord("Z")):
                action[6] -= grip_step
            if key_down(keys, ord("x")) or key_down(keys, ord("X")):
                action[6] += grip_step

            # --- clip ---------------------------------------------------------
            action[0] = np.clip(action[0], x_min, x_max)
            action[1] = np.clip(action[1], y_min, y_max)
            action[2] = np.clip(action[2], z_min, z_max)
            action[6] = np.clip(action[6], grip_min, grip_max)

            # --- send command -------------------------------------------------
            env.robot.move_ee(action[:6], control_method="end")
            env.robot.move_gripper(action[6])

            # --- step simulation ----------------------------------------------
            safe_step_simulation(env)

            # --- read forces --------------------------------------------------
            (
                tf_force, tf_max_force, tf_contact_count,
                if_force, if_max_force, if_contact_count,
            ) = read_separate_fingertip_forces(env)

            tf_force_smooth = (
                (1.0 - smoothing_alpha) * tf_force_smooth
                + smoothing_alpha * tf_force
            )
            if_force_smooth = (
                (1.0 - smoothing_alpha) * if_force_smooth
                + smoothing_alpha * if_force
            )

            tf_level = force_to_level(tf_force_smooth)
            if_level = force_to_level(if_force_smooth)

            # --- ESP32 feedback (depends on condition) ------------------------
            if esp_client is not None:
                if condition == "visual_only":
                    esp_client.send_levels(0, 0)
                else:
                    esp_client.send_levels(tf_level, if_level)

            # --- trial trigger (SPACE) ----------------------------------------
            b3g_space = getattr(p, "B3G_SPACE", None)
            trial_triggered = key_triggered(keys, ord(" ")) or (
                b3g_space is not None and key_triggered(keys, b3g_space)
            )

            if trial_triggered:
                true_contact = (
                    1 if (tf_contact_count > 0 or if_contact_count > 0) else 0
                )

                print("\n" + "-" * 40)
                print("Ask participant: contact or no contact?")
                print("-" * 40)

                # Block until the experimenter enters a valid response.
                while True:
                    try:
                        raw = input(
                            "Enter 1 (contact) or 0 (no contact): "
                        ).strip()
                        response = int(raw)
                        if response in (0, 1):
                            break
                        print("  Invalid — please enter 1 or 0.")
                    except ValueError:
                        print("  Invalid — please enter 1 or 0.")

                recorder.record_trial(
                    participant_id=participant_id,
                    condition=condition,
                    true_contact=true_contact,
                    response=response,
                    tf_force_smooth=tf_force_smooth,
                    if_force_smooth=if_force_smooth,
                    tf_level=tf_level,
                    if_level=if_level,
                )

                verdict = "CORRECT" if response == true_contact else "INCORRECT"
                print(
                    f"  true_contact={true_contact}, "
                    f"response={response}  →  {verdict}"
                )
                print(
                    f"  trial #{recorder.trial_count} recorded "
                    f"(condition={condition})"
                )
                print("-" * 40 + "\n")

            # --- periodic status in command window ----------------------------
            now = time.time()
            if now - last_print_time > 0.2:
                last_print_time = now
                contact_flag = (
                    "YES"
                    if (tf_contact_count > 0 or if_contact_count > 0)
                    else "no"
                )
                print(
                    f"\r"
                    f"x={action[0]: .3f} "
                    f"y={action[1]: .3f} "
                    f"z={action[2]: .3f} "
                    f"grip={action[6]: .3f} | "
                    f"contact={contact_flag} | "
                    f"{condition} | "
                    f"trials={recorder.trial_count}   ",
                    end="",
                    flush=True,
                )

    finally:
        print("\n\nExperiment finished.")
        print(f"Total trials recorded: {recorder.trial_count}")
        print(f"Data saved to: {csv_path}")

        recorder.close()

        if esp_client is not None:
            esp_client.send_levels(0, 0)
            esp_client.close()

        if hasattr(env, "close"):
            env.close()
        else:
            p.disconnect()

# ---------------------------------------------------------------------------
#  Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(
            "Usage: python experiment1_contact_judgment.py "
            "<participant_id> <condition>"
        )
        print("  participant_id : e.g. P01, P02, ...")
        print("  condition      : visual_only  or  vision_haptic")
        sys.exit(1)

    participant_id = sys.argv[1]
    condition = sys.argv[2]

    if condition not in ("visual_only", "vision_haptic"):
        print(
            f"Error: unknown condition '{condition}'. "
            "Must be 'visual_only' or 'vision_haptic'."
        )
        sys.exit(1)

    experiment1(participant_id, condition)
