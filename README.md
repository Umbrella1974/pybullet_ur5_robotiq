# pybullet_ur5_robotiq

这个仓库当前主要用于在 PyBullet 中运行 UR5 + Robotiq 夹爪实验，重点是两个实验脚本：

- `experiment1_contact_judgment.py`
  接触判断实验，对比 `visual_only` 和 `vision_haptic`
- `experiment2_force_maintenance.py`
  力维持实验，支持自动扰动、日志记录、结果绘图

如果你只是想快速上手，优先看“快速开始”和“Experiment 2 操作流程”这两节。

## 1. 环境准备

建议使用 Python 3，并从项目根目录运行脚本。

安装当前代码实际会用到的依赖：

```bash
pip install pybullet numpy pyyaml attrdict tqdm opencv-python scipy matplotlib
```

说明：

- `torch` 在当前实验主流程里不是必需依赖。
- 如果你没有连接 ESP32，请把 `experiment2_config.yaml` 中的 `esp32.send_to_esp32` 设为 `false`。
- PowerShell 启动时如果看到 `profile.ps1` 的执行策略警告，可以忽略，它不是项目错误。

## 2. 当前文件用途

- `main.py`
  最基础的 PyBullet slider demo，用来确认环境能否启动
- `forcelevel_simulation.py`
  手动调姿态、看接触力、导出 `initial_action` 的辅助脚本
- `experiment1_contact_judgment.py`
  实验 1：接触判断
- `experiment2_force_maintenance.py`
  实验 2：力维持主脚本
- `experiment2_config.yaml`
  实验 2 配置文件
- `plot_experiment2_results.py`
  读取实验 2 的 time-series CSV 并画图
- `sevropwm_simulation.py`
  ESP32 端参考代码
- `experiment2_data/`
  实验 2 结果输出目录

## 3. 快速开始

先进入项目目录：

```bash
cd /path/to/pybullet_ur5_robotiq
```

确认 PyBullet 能正常打开：

```bash
python main.py
```

运行实验 2：

```bash
python experiment2_force_maintenance.py experiment2_config.yaml
```

画实验 2 结果图：

```bash
python plot_experiment2_results.py experiment2_data
```

如果你只想先找一个合适的初始位姿：

```bash
python forcelevel_simulation.py
```

## 4. 推荐操作顺序

建议按下面的顺序使用这个仓库：

1. 先运行 `forcelevel_simulation.py`，手动把机械臂调到一个合适的抓取准备位姿。
2. 在该脚本里按 `V`，终端会打印当前 `initial_action` 的 YAML 片段。
3. 把这段 `initial_action` 复制到 `experiment2_config.yaml`。
4. 根据需要调整 `center_object.half_extents`，决定实验中的标准木块大小。
5. 运行 `experiment2_force_maintenance.py experiment2_config.yaml`。
6. 在 `INIT` 阶段微调姿态和夹爪，确认起始状态合适后按 `S` 开始正式 trial。
7. 实验结束后，用 `plot_experiment2_results.py` 查看 time-series 曲线。

## 5. Experiment 2 操作流程

当前 `experiment2` 的真实行为是：

1. 启动脚本并读取 `experiment2_config.yaml`
2. 创建 PyBullet 环境
3. 如果 `experiment.auto_move_to_initial: true`
   机械臂会先自动移动到 `initial_action`
4. 如果 `center_object.enabled: true`
   会在当前夹爪姿态下生成一个居中的、不可移动的标准方块，并把它作为后续接触对象
5. 进入 `INIT` 阶段
6. 你在 `INIT` 阶段微调位置和夹爪
7. 按 `S` 后，正式 trial 立刻开始计时
8. trial 中会进入 `AUTO_WAIT`、`AUTO_PERTURB_CLOSE`、`AUTO_PERTURB_OPEN`、`USER_CONTROL` 等状态
9. 按 `Q` 退出，脚本会输出 summary，并写 CSV

### `INIT` 阶段和正式 trial 的关系

- `INIT` 是准备阶段，不是正式实验。
- 只有按下 `S` 才会开始正式 trial。
- `init_timeout_sec` 现在只负责提醒，不会自动开始 trial。
- 正式 trial 开始后，`trial_elapsed_sec` 从 `0` 开始累计。

### Experiment 2 键位

- `I / K`：`x + / x -`
- `J / L`：`y - / y +`
- `U / O`：`z + / z -`
- `1 / 2`：`roll - / roll +`
- `3 / 4`：`pitch - / pitch +`
- `5 / 6`：`yaw - / yaw +`
- `Z / X`：夹爪固定步长闭合 / 打开
- `P`：打印当前接触信息
- `S`：在 `INIT` 中开始正式 trial
- `Q`：退出

重要说明：

- 在 `experiment2` 中，`Z / X` 已经改成“每按一次走固定一步”，不是按住连续变化。
- 在 `forcelevel_simulation.py` 中，`Z / X` 仍然是“按住连续变化”。
- 所以这两个脚本里的 `Z / X` 手感不同，这是当前设计，不是 bug。

### Experiment 2 中自动扰动的基本逻辑

- 正式 trial 开始后，系统先进入 `AUTO_WAIT`
- 到达随机等待时间后，程序会在允许的方向中选择一次自动扰动
- 自动扰动方向可能是：
  - `AUTO_PERTURB_CLOSE`
  - `AUTO_PERTURB_OPEN`
- 用户按 `Z / X` 时，手动输入优先，会进入 `USER_CONTROL`

当前自动扰动的停止条件主要包括：

- 达到 `perturb_duration_sec`
- 达到 `max_auto_delta`
- 自动夹紧时达到 `emergency_high`
- 自动松开时进入 `stable_no_contact`
- 到达安全夹爪边界

## 6. `experiment2_config.yaml` 常用参数

### 实验基本信息

```yaml
experiment:
  participant_id: "P00"
  condition: "visual_only"
```

- `participant_id`
  参与者编号，会写入输出文件名
- `condition`
  只能是 `visual_only` 或 `vision_haptic`

### 是否向 ESP32 发送反馈

```yaml
esp32:
  send_to_esp32: false
```

- 没有连接 ESP32 时，建议固定为 `false`
- `visual_only` 模式下，即使启用了发送，程序也会发送 `(0, 0)` 级别

### 初始位姿

```yaml
initial_action:
  x: 0.0
  y: 0.0
  z: 0.5
  roll: 0.0
  pitch: 1.57079632679
  yaw: 1.57079632679
  gripper_opening: 0.04
```

- 这是 `auto_move_to_initial` 使用的目标位姿
- 推荐先用 `forcelevel_simulation.py` 调到满意位置，再按 `V` 导出

### 居中标准木块

```yaml
center_object:
  enabled: true
  replace_env_box: true
  mass: 0.0
  half_extents: [0.015, 0.015, 0.020]
  position_offset_local: [0.0, 0.0, 0.0]
  settle_steps: 30
```

- `enabled`
  是否生成新的标准接触方块
- `mass`
  当前要求必须为 `0.0`，这样物体不可移动
- `half_extents`
  方块半尺寸，单位是米
- `position_offset_local`
  相对当前夹爪局部坐标系的偏移
- `settle_steps`
  创建方块后额外稳定仿真的步数

尺寸计算方式：

- 完整尺寸 = `2 * half_extents`
- 例如 `half_extents: [0.015, 0.015, 0.020]`
  对应完整尺寸约 `0.03 x 0.03 x 0.04 m`

### 键盘控制步长

```yaml
keyboard:
  pos_step: 0.003
  rot_step: 0.02
  user_grip_step: 0.0010
```

- `pos_step`
  每次位置微调的步长
- `rot_step`
  每次姿态微调的步长
- `user_grip_step`
  在 `experiment2` 中，每按一次 `Z / X`，夹爪开合变化的固定长度

如果你觉得 `Z / X` 还是容易按过头，可以先试：

- `0.0005`

如果你觉得太慢，可以试：

- `0.0015`

### 自动扰动参数

```yaml
state_machine:
  perturb_interval_min_sec: 1.0
  perturb_interval_max_sec: 3.0
  perturb_duration_sec: 2.0
  auto_grip_speed: 0.012
  max_auto_delta: 0.024
```

- `perturb_interval_min_sec` / `perturb_interval_max_sec`
  自动扰动之间的随机等待时间范围
- `perturb_duration_sec`
  一次自动扰动的最长持续时间
- `auto_grip_speed`
  自动扰动时夹爪速度，单位 `m/s`
- `max_auto_delta`
  一次自动扰动允许的最大总位移

实际每帧自动变化量是：

```text
auto_grip_speed * (1 / 240)
```

## 7. `forcelevel_simulation.py` 的用途

这个脚本更适合做两件事：

- 手动找 `initial_action`
- 观察接触力、确认哪几个 link 在接触

运行：

```bash
python forcelevel_simulation.py
```

键位和 `experiment2` 大体类似，但有一个重要区别：

- 在这个脚本里，`Z / X` 是按住连续开合，不是单步触发

常用按键：

- `P`
  打印当前接触信息
- `V`
  把当前 `action` 打印成一段 YAML，可直接拷到 `experiment2_config.yaml`
- `Q`
  退出

## 8. Experiment 1 用法

运行格式：

```bash
python experiment1_contact_judgment.py P01 visual_only
python experiment1_contact_judgment.py P01 vision_haptic
```

实验 1 的核心按键：

- `SPACE`
  触发一轮接触判断试次
- `C`
  在 `visual_only` 和 `vision_haptic` 之间切换
- 其余位置、姿态、夹爪按键基本沿用 `forcelevel_simulation.py`

## 9. 输出文件

实验 2 默认输出到：

```text
experiment2_data/
```

每次运行通常会生成两类文件：

- `*_timeseries.csv`
  连续时序数据
- `*_events.csv`
  事件日志

`plot_experiment2_results.py` 主要读取 `*_timeseries.csv`：

- 如果传入单个 CSV，会画这一轮 trial 的曲线
- 如果传入一个目录或多个 CSV，也会尝试按 `condition` 画汇总图

## 10. 常见问题

### 1. 运行后没有窗口

优先检查：

- 当前 Python 环境是否装了 `pybullet`
- 是否从终端运行脚本，而不是双击 `.py`
- 是否从项目根目录运行

推荐命令：

```bash
python experiment2_force_maintenance.py experiment2_config.yaml
```

### 2. 改了 YAML 之后要不要重新 `py_compile`

不用。

- 改 `.yaml`：直接重新运行实验脚本即可
- 改 `.py`：才有必要额外做语法检查

### 3. 为什么 `Z / X` 手感和以前不一样

因为现在 `experiment2` 已经改成：

- 每按一次只移动一个固定步长

这样更适合实验操作，能减少因为按住时间不同造成的 overshoot。

### 4. 为什么有时看不到自动扰动

常见原因：

- 正式 trial 开始后还没等到随机等待时间
- 你中途频繁按了 `Z / X`，程序会优先进入 `USER_CONTROL`
- 当前接触状态不满足某个自动方向的触发条件

### 5. `FORCE_ABOVE_HIGH / FORCE_BELOW_LOW / FORCE_BACK_IN_RANGE` 是不是核心控制逻辑

不是。

当前它们主要是事件日志，用来做后处理和可视化参考，不是状态机的核心驱动条件。

## 11. 建议的最小工作流

如果你只是想尽快开始做实验，推荐直接按下面做：

1. 安装依赖
2. 把 `experiment2_config.yaml` 里的 `esp32.send_to_esp32` 设为 `false`
3. 用 `forcelevel_simulation.py` 找到一个满意的 `initial_action`
4. 调整 `center_object.half_extents` 到你想要的木块大小
5. 运行 `experiment2_force_maintenance.py experiment2_config.yaml`
6. 在 `INIT` 阶段微调后按 `S`
7. 实验结束后用 `plot_experiment2_results.py experiment2_data` 看结果
