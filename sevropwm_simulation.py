import network
import socket
import time
from machine import Pin, PWM

# ==========================================
# 1. 硬件校准参数 (MG90S @ ESP32-S3)
# ==========================================
TF_PIN = 13
IF_PIN = 14
PWM_FREQ = 50
RESOLUTION = 1023  # 10-bit分辨率


# 脉宽定义
MIN_US = 500   # 0度
MAX_US = 1800  # 180度

# 初始化PWM，创建两个独立的舵机对象
servo_tf = PWM(Pin(TF_PIN), freq=PWM_FREQ, duty=0)
servo_if = PWM(Pin(IF_PIN), freq=PWM_FREQ, duty=0)

# 角度映射函数：角度——脉宽——占空比值——函数
def set_servo_angle(pwm_obj, angle):
    """根据线性映射计算占空比并控制舵机"""
    # 计算目标脉宽 (微秒)
    pulse_us = MIN_US + (angle / 180) * (MAX_US - MIN_US)
    # 转换为 0-1023 的占空比值
    # 公式: (pulse_us / 周期总时长20000us) * 1023
    duty_val = int((pulse_us / 20000) * RESOLUTION)
    pwm_obj.duty(duty_val)  # 更新占空比，舵机转动到对应角度

# ==========================================
# 2. 档位动作定义：持续往返运动
# ==========================================
# 每个档位对应往返运动的最大角度
# Level 0: 停止并回到 0°
# Level 1: 0° ↔ 60°
# Level 2: 0° ↔ 120°
# Level 3: 0° ↔ 180°
LEVEL_MAX_ANGLES = {
    0: 0,
    1: 60,
    2: 120,
    3: 180
}

# 半周期时间：每隔 1 秒切换一次目标角度
# 例如 Level 2 时：0° → 120° 等 1 秒 → 0° 等 1 秒 → 120° ...
SWEEP_HALF_PERIOD_MS = 1000

# 最新接收到的反馈档位
latest_levels = {
    "TF": 0,
    "IF": 0
}

# 当前输出角度记录，避免每 5ms 重复写 PWM
last_angles = {
    "TF": None,
    "IF": None
}

# 当前往返方向：0 表示当前目标为 0°，1 表示当前目标为最大角度
sweep_phase = {
    "TF": 0,
    "IF": 0
}

# 下一次切换目标角度的时间
next_toggle_time = {
    "TF": 0,
    "IF": 0
}

# 上一次档位，用于检测档位变化
last_levels = {
    "TF": 0,
    "IF": 0
}

# ==========================================
# 3. 核心逻辑：数据处理与同步驱动
# ==========================================
# 数据解析模块
def parse_tcp_data(data):
    """
    Parse protocol:
    [AA 55 AA 55] [Len] [Payload...] [Checksum]

    New payload format:
    [1, TF_level, 2, IF_level]
    """
    idx = data.find(b'\xAA\x55\xAA\x55')

    if idx == -1:
        return

    if len(data) < idx + 6:
        return

    payload_len = data[idx + 4]
    packet_end = idx + 5 + payload_len + 1

    if payload_len > 128:
        return

    if len(data) < packet_end:
        return

    payload = data[idx + 5 : idx + 5 + payload_len]
    recv_checksum = data[idx + 5 + payload_len]
    calc_checksum = sum(payload) & 0xFF

    if recv_checksum != calc_checksum:
        print("Checksum error")
        return

    # New compact format: [1, TF_level, 2, IF_level]
    if payload_len >= 4:
        if payload[0] == 1:
            latest_levels["TF"] = min(max(payload[1], 0), 3)

        if payload[2] == 2:
            latest_levels["IF"] = min(max(payload[3], 0), 3)

        print("Levels:", latest_levels["TF"], latest_levels["IF"])

def set_part_angle(part, angle):
    """
    Set servo angle for TF or IF channel.
    Only update PWM when the target angle changes.
    """
    pwm = servo_tf if part == "TF" else servo_if

    angle = max(0, min(180, angle))

    if last_angles[part] != angle:
        set_servo_angle(pwm, angle)
        last_angles[part] = angle


# 舵机管理器
def manage_servos():
    """
    Non-blocking continuous sweep control.

    If level > 0:
        the corresponding servo continuously sweeps between 0°
        and the maximum angle defined by LEVEL_MAX_ANGLES[level].

    If level == 0:
        the corresponding servo returns to 0° and stops.
    """
    now = time.ticks_ms()

    for part in ["TF", "IF"]:
        current_lvl = latest_levels[part]
        max_angle = LEVEL_MAX_ANGLES.get(current_lvl, 0)

        # Case 1: no feedback, return to 0° and stop
        if current_lvl == 0:
            set_part_angle(part, 0)
            sweep_phase[part] = 0
            next_toggle_time[part] = now + SWEEP_HALF_PERIOD_MS
            last_levels[part] = 0
            continue

        # Case 2: level has changed
        # Immediately start a new sweep from the maximum angle
        if current_lvl != last_levels[part]:
            sweep_phase[part] = 1
            set_part_angle(part, max_angle)
            next_toggle_time[part] = now + SWEEP_HALF_PERIOD_MS
            last_levels[part] = current_lvl
            continue

        # Case 3: level remains active
        # Toggle between 0° and max_angle every SWEEP_HALF_PERIOD_MS
        if time.ticks_diff(now, next_toggle_time[part]) >= 0:
            if sweep_phase[part] == 1:
                sweep_phase[part] = 0
                set_part_angle(part, 0)
            else:
                sweep_phase[part] = 1
                set_part_angle(part, max_angle)

            next_toggle_time[part] = now + SWEEP_HALF_PERIOD_MS

# ==========================================
# 4. 主程序入口
# ==========================================
def main():
    # WiFi 连接部分 (请自行补充 SSID/PWD)
    # ...

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(('0.0.0.0', 12345))
    s.listen(1)
    s.setblocking(False) # 关键：非阻塞Socket

    print("TCP Server Ready...")

    conn = None
    last_servo_time = time.ticks_ms()

    while True:
        current_time = time.ticks_ms()

        # 定期执行舵机控制，至少每5ms一次
        if time.ticks_diff(current_time, last_servo_time) > 5:
            manage_servos()
            last_servo_time = current_time

        try:
            if conn is None:
                # 尝试接受新连接
                conn, addr = s.accept()
                conn.setblocking(False)  # 设为非阻塞
                print("Connected by:", addr)
            else:
                # 尝试接收数据
                try:
                    data = conn.recv(128)
                    if data:
                        parse_tcp_data(data)
                    else:
                        # 连接关闭
                        conn.close()
                        conn = None
                except OSError:
                    # 无数据可用，继续
                    pass
        except OSError:
            # 无连接等待，继续
            pass

if __name__ == "__main__":
    main()