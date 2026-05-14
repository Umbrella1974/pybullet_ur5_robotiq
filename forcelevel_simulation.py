import os
import math
import time
import socket

import numpy as np
import pybullet as p

from env import ClutteredPushGrasp
from robot import UR5Robotiq85

# Different versions of this repository may use different class names.
# Try YCBModels first. If it is not available, use Models instead.
try:
    from utilities import YCBModels
except ImportError:
    from utilities import Models as YCBModels


TF_LINK = 12  # left_inner_finger_pad
IF_LINK = 17  # right_inner_finger_pad

ESP32_IP = "192.168.1.16"  # 改成你已知的 ESP32 IP
ESP32_PORT = 12345
SEND_TO_ESP32 = True

MAGIC = b'\xAA\x55\xAA\x55'

KEY_IS_DOWN = getattr(p, "KEY_IS_DOWN", 1)
KEY_WAS_TRIGGERED = getattr(p, "KEY_WAS_TRIGGERED", 2)


def key_down(keys, key):
    return keys.get(key, 0) & KEY_IS_DOWN


def key_triggered(keys, key):
    return keys.get(key, 0) & KEY_WAS_TRIGGERED


def safe_step_simulation(env):
    """
    Step the PyBullet simulation.

    Some versions of this project define env.step_simulation().
    If not, fall back to p.stepSimulation().
    """
    if hasattr(env, "step_simulation"):
        env.step_simulation()
    else:
        p.stepSimulation()


def get_link_name(robot_id, link_index):
    """
    Return the link name for a given robot link index.
    """
    if link_index < 0:
        return "base"

    try:
        return p.getJointInfo(robot_id, link_index)[12].decode("utf-8")
    except Exception:
        return f"unknown_link_{link_index}"


def print_current_contacts(env):
    """
    Print all current contact points between the robot and the target object.

    Press P during the simulation to call this function.
    It is useful for confirming which robot links are touching the object.
    """
    contacts = p.getContactPoints(
        bodyA=env.robot.id,
        bodyB=env.boxID
    )

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

    TF_LINK = 12: left_inner_finger_pad
    IF_LINK = 17: right_inner_finger_pad

    Returns:
        tf_force_sum, tf_max_force, tf_contact_count,
        if_force_sum, if_max_force, if_contact_count
    """
    contacts = p.getContactPoints(
        bodyA=env.robot.id,
        bodyB=env.boxID
    )

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
        if_force_sum, if_max_force, if_contact_count
    )


def force_to_level(force):
    """
    Convert simulated fingertip contact force to four feedback levels:
    0, 1, 2, 3.

    These thresholds are initial values. You should adjust them after observing
    the fingertip-only contact force range in your own simulation.
    """
    if force < 5.0:
        return 0
    elif force < 50.0:
        return 1
    elif force < 100.0:
        return 2
    else:
        return 3


def create_environment():
    """
    Create the PyBullet UR5 Robotiq environment.
    """
    ycb_models = YCBModels(
        os.path.join("./data/ycb", "**", "textured-decmp.obj")
    )

    camera = None

    robot = UR5Robotiq85((0, 0.5, 0), (0, 0, 0))
    env = ClutteredPushGrasp(robot, ycb_models, camera, vis=True)
    env.reset()

    return env


def keyboard_control_demo():
    env = create_environment()

    esp_client = None

    if SEND_TO_ESP32:
        esp_client = ESP32FeedbackClient(ESP32_IP, ESP32_PORT)

    # Initial action:
    # x, y, z, roll, pitch, yaw, gripper_opening_length
    action = np.array([
        0.0,            # x
        0.0,            # y
        0.5,            # z
        0.0,            # roll
        math.pi / 2,    # pitch
        math.pi / 2,    # yaw
        0.04            # gripper opening length
    ], dtype=float)

    # Control step sizes
    pos_step = 0.003       # metres per keyboard update
    rot_step = 0.02        # radians per keyboard update
    grip_step = 0.0015     # metres per keyboard update

    # Position and gripper safety limits
    x_min, x_max = -0.224, 0.224
    y_min, y_max = -0.224, 0.224
    z_min, z_max = 0.05, 1.0
    grip_min, grip_max = 0.0, 0.085

    # Contact force smoothing
    tf_force_smooth = 0.0
    if_force_smooth = 0.0
    smoothing_alpha = 0.15

    last_print_time = time.time()
    debug_text_id = None

    print("\nKeyboard control started.")
    print("Click inside the PyBullet GUI window first, then use the keyboard.")
    print("\nPosition control:")
    print("  I / K : x + / x -")
    print("  J / L : y - / y +")
    print("  U / O : z + / z -")
    print("\nOrientation control:")
    print("  1 / 2 : roll - / roll +")
    print("  3 / 4 : pitch - / pitch +")
    print("  5 / 6 : yaw - / yaw +")
    print("\nGripper control:")
    print("  Z / X : close / open")
    print("\nDebug:")
    print("  P     : print current contact links")
    print("  Q     : quit\n")

    try:
        while True:
            keys = p.getKeyboardEvents()

            # Quit
            if key_triggered(keys, ord("q")) or key_triggered(keys, ord("Q")):
                break

            # Print current contact details
            if key_triggered(keys, ord("p")) or key_triggered(keys, ord("P")):
                print_current_contacts(env)

            # Position control
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

            # Orientation control
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

            # Gripper control
            if key_down(keys, ord("z")) or key_down(keys, ord("Z")):
                action[6] -= grip_step
            if key_down(keys, ord("x")) or key_down(keys, ord("X")):
                action[6] += grip_step

            # Clip command values
            action[0] = np.clip(action[0], x_min, x_max)
            action[1] = np.clip(action[1], y_min, y_max)
            action[2] = np.clip(action[2], z_min, z_max)
            action[6] = np.clip(action[6], grip_min, grip_max)

            # Send command to robot
            env.robot.move_ee(action[:6], control_method="end")
            env.robot.move_gripper(action[6])

            # Step simulation
            safe_step_simulation(env)

            (
                tf_force, tf_max_force, tf_contact_count,
                if_force, if_max_force, if_contact_count
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
            
            if esp_client is not None:
                esp_client.send_levels(tf_level, if_level)

            # Print at lower frequency
            now = time.time()
            if now - last_print_time > 0.2:
                last_print_time = now

                print(
                    f"\r"
                    f"x={action[0]: .3f}, "
                    f"y={action[1]: .3f}, "
                    f"z={action[2]: .3f}, "
                    f"grip={action[6]: .3f} | "
                    f"TF_contact={tf_contact_count}, "
                    f"TF_force={tf_force: .3f}, "
                    f"TF_smooth={tf_force_smooth: .3f}, "
                    f"TF_level={tf_level} | "
                    f"IF_contact={if_contact_count}, "
                    f"IF_force={if_force: .3f}, "
                    f"IF_smooth={if_force_smooth: .3f}, "
                    f"IF_level={if_level}",
                    end="",
                    flush=True
                )

                # Show force information in GUI
                text = (
                    f"TF contacts: {tf_contact_count}\n"
                    f"TF force: {tf_force:.3f}\n"
                    f"TF smooth: {tf_force_smooth:.3f}\n"
                    f"TF level: {tf_level}\n"
                    f"IF contacts: {if_contact_count}\n"
                    f"IF force: {if_force:.3f}\n"
                    f"IF smooth: {if_force_smooth:.3f}\n"
                    f"IF level: {if_level}"
                )

                if debug_text_id is None:
                    debug_text_id = p.addUserDebugText(
                        text,
                        textPosition=[-0.45, -0.45, 0.7],
                        textColorRGB=[1, 0, 0],
                        textSize=1.2
                    )
                else:
                    debug_text_id = p.addUserDebugText(
                        text,
                        textPosition=[-0.45, -0.45, 0.7],
                        textColorRGB=[1, 0, 0],
                        textSize=1.2,
                        replaceItemUniqueId=debug_text_id
                    )

    finally:
        print("\nKeyboard control stopped.")

        if hasattr(env, "close"):
            env.close()
        else:
            p.disconnect()

        if esp_client is not None:
            esp_client.send_levels(0, 0)
            esp_client.close()


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

        # 避免重复发送完全相同的数据
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


if __name__ == "__main__":
    keyboard_control_demo()