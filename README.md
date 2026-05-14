# pybullet_ur5_robotiq

这个仓库当前主要用于在 PyBullet 中运行 UR5 + Robotiq 夹爪实验，当前最重要的两个脚本是：

- `experiment1_contact_judgment.py`
  接触判断实验，对比 `visual_only` 和 `vision_haptic`
- `experiment2_force_maintenance.py`
  力维持实验，支持自动扰动、ESP32 反馈、日志记录和后处理绘图

如果你只是想快速开始，优先看：

1. “快速开始”
2. “Experiment 2 操作流程”
3. “绘图与结果输出”

## 1. 环境准备

建议使用 Python 3，并从项目根目录运行脚本。

安装当前代码实际用到的依赖：

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
  实验 2 后处理与绘图脚本
- `sevropwm_simulation.py`
  ESP32 端参考代码
- `experiment2_data/`
  实验 2 原始输出目录
- `experiment2_figures/`
  推荐的绘图输出目录

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

绘制实验 2 结果：

```bash
python plot_experiment2_results.py --data-dir experiment2_data --out-dir experiment2_figures
```

如果你想先找一个合适的初始位姿：

```bash
python forcelevel_simulation.py
```

## 4. 推荐操作顺序

建议按下面顺序使用这个仓库：

1. 先运行 `forcelevel_simulation.py`，手动把机械臂调到一个合适的抓取准备位姿。
2. 在该脚本里按 `V`，终端会打印当前 `initial_action` 的 YAML 片段。
3. 把这段 `initial_action` 复制到 `experiment2_config.yaml`。
4. 根据需要调整 `center_object.half_extents`，决定实验中的标准木块大小。
5. 运行 `experiment2_force_maintenance.py experiment2_config.yaml`。
6. 在 `INIT` 阶段微调姿态和夹爪，确认起始状态合适后按 `S` 开始正式 trial。
7. 实验结束后，用 `plot_experiment2_results.py` 生成 summary 和图。

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
  participant_id: "P01"
  condition: "visual_only"
  formal_experiment: false
```

- `participant_id`
  参与者编号，会写入输出文件名和 CSV
- `condition`
  只能是 `visual_only` 或 `vision_haptic`
- `formal_experiment`
  `false` 表示调试运行
  `true` 表示正式实验
  绘图脚本默认只统计 `formal_experiment: true` 的数据，并只为正式实验自动生成 `trial_id`

### 是否向 ESP32 发送反馈

```yaml
esp32:
  send_to_esp32: false
  ip: "192.168.1.16"
  port: 12345
  feedback_send_period_sec: 0.10
  keepalive_send_period_sec: 1.0
```

- `send_to_esp32`
  是否真正建立 TCP 连接并发送反馈
- `ip` / `port`
  ESP32 的地址和端口
- `feedback_send_period_sec`
  正常反馈发送节流周期
- `keepalive_send_period_sec`
  payload 不变时的保活发送周期

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

## 7. 如何启用 `vision_haptic`

### 最小配置

如果你只想让实验进入 `vision_haptic` 条件，需要改这两处：

```yaml
experiment:
  condition: "vision_haptic"

esp32:
  send_to_esp32: true
```

### 推荐完整配置

```yaml
experiment:
  participant_id: "P01"
  condition: "vision_haptic"
  formal_experiment: true

esp32:
  send_to_esp32: true
  ip: "192.168.1.16"
  port: 12345
  feedback_send_period_sec: 0.10
  keepalive_send_period_sec: 1.0
```

### 真实行为说明

- `visual_only`
  脚本会把目标反馈固定成 `(0, 0)`，即使已连接 ESP32 也是这样
- `vision_haptic`
  脚本会把当前左右指尖等级发送给 ESP32
  具体来自：
  - 左指尖：`tf_level`
  - 右指尖：`if_level`

在代码里这一点由 [experiment2_force_maintenance.py](</d:/research_history/first_one/research_code/pybullet_ur5_robotiq/experiment2_force_maintenance.py:1237>) 的 `build_desired_feedback()` 控制。

### 如果只想测试条件切换，但不真的发硬件

你也可以这样配置：

```yaml
experiment:
  condition: "vision_haptic"

esp32:
  send_to_esp32: false
```

这样程序内部仍按 `vision_haptic` 逻辑运行，但不会真正建立 TCP 发送。

### ESP32 端

- `sevropwm_simulation.py` 是 ESP32 端参考代码
- 你需要保证 ESP32 的 IP / 端口和 YAML 一致
- 运行时如果连接成功，终端会打印  
  `Connected to ESP32 at ...`
- 如果失败，实验不会直接崩溃，但会打印连接或发送错误

## 8. `forcelevel_simulation.py` 的用途

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

## 9. Experiment 1 用法

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

## 10. 绘图与结果输出

实验 2 默认把原始结果写到：

```text
experiment2_data/
```

每次运行通常会生成两类文件：

- `*_timeseries.csv`
  连续时序数据
- `*_events.csv`
  事件日志

### 推荐绘图命令

```bash
python plot_experiment2_results.py --data-dir experiment2_data --out-dir experiment2_figures
```

### 常用选项

- `--summary`
  生成 `experiment2_summary_by_trial.csv` 和按 `condition` 的汇总图
- `--single "*170911_timeseries.csv"`
  只为匹配到的单个 trial 生成详细图
- `--all-trials`
  为所有纳入分析的 trial 生成单 trial 图
- `--include-nonformal`
  连调试数据一起纳入；默认只统计 `formal_experiment: true` 的正式实验
- `--paper-mode`
  使用更适合论文的分辨率和字号，并额外导出 `SVG`
- `--include-threshold-reaction`
  额外计算并输出 `threshold_reaction_time` 相关图
- `--recovery-hold-sec 0.2`
  控制 `recovery_time` 所需的目标区间持续保持时间

### `--paper-mode` 会输出什么

`--paper-mode` 不改变“画哪些图”的逻辑，只改变：

- 分辨率和字号
- 每张图除 `PNG` 外额外导出 `SVG`

如果你用的是 summary 模式，它会导出例如：

- `mean_force_error_by_condition.png/.svg`
- `in_target_range_ratio_by_condition.png/.svg`
- `disturbance_response_time_by_condition.png/.svg`
- `recovery_time_by_condition.png/.svg`
- `target_range_ratio_stack.png/.svg`
- `user_workload_by_condition.png/.svg`

如果还加了 `--include-threshold-reaction`，会额外导出：

- `threshold_reaction_time_by_condition.png/.svg`

如果你用 `--single` 或 `--all-trials`，则每个 trial 还会导出：

- `single_trial_overview_*.png/.svg`
- `force_with_events_*.png/.svg`
- `force_gripper_alignment_*.png/.svg`
- `target_range_band_*.png/.svg`
- `left_right_fingertip_forces_*.png/.svg`

### 当前绘图脚本会生成哪些图

单 trial 图：

- `single_trial_overview_*`
- `force_with_events_*`
- `force_gripper_alignment_*`
- `target_range_band_*`
- `left_right_fingertip_forces_*`

汇总图：

- `mean_force_error_by_condition`
- `in_target_range_ratio_by_condition`
- `disturbance_response_time_by_condition`
- `recovery_time_by_condition`
- `target_range_ratio_stack`
- `user_workload_by_condition`

补充图：

- `disturbance_response_time_close_only_by_condition`
- `disturbance_response_time_open_only_by_condition`
- `recovery_time_close_only_by_condition`
- `recovery_time_open_only_by_condition`

同时会输出：

- `experiment2_summary_by_trial.csv`

## 11. 如果想改曲线颜色、图例位置，怎么改

这些改动都集中在 [plot_experiment2_results.py](</d:/research_history/first_one/research_code/pybullet_ur5_robotiq/plot_experiment2_results.py:1>)，不需要改实验主脚本。

### 1. 改全局分辨率、字号、导出风格

看 `configure_plot_style(paper_mode)`：

- [plot_experiment2_results.py](</d:/research_history/first_one/research_code/pybullet_ur5_robotiq/plot_experiment2_results.py:95>)

这里控制：

- `figure.dpi`
- `savefig.dpi`
- `font.size`
- `axes.titlesize`
- `legend.fontsize`

如果你想统一把字变大、图变高清，就改这一段。

### 2. 改按 condition 的颜色

看 `plot_paired_metric()`：

- [plot_experiment2_results.py](</d:/research_history/first_one/research_code/pybullet_ur5_robotiq/plot_experiment2_results.py:645>)

当前是：

```python
colors = {"visual_only": "tab:gray", "vision_haptic": "tab:cyan"}
```

你可以直接改成，比如：

```python
colors = {"visual_only": "#666666", "vision_haptic": "#D55E00"}
```

### 3. 改单 trial 曲线颜色

几个主要函数的位置：

- `plot_single_trial_overview()`
  [plot_experiment2_results.py](</d:/research_history/first_one/research_code/pybullet_ur5_robotiq/plot_experiment2_results.py:972>)
- `plot_force_with_events()`
  [plot_experiment2_results.py](</d:/research_history/first_one/research_code/pybullet_ur5_robotiq/plot_experiment2_results.py:1014>)
- `plot_force_gripper_alignment()`
  [plot_experiment2_results.py](</d:/research_history/first_one/research_code/pybullet_ur5_robotiq/plot_experiment2_results.py:1045>)
- `plot_target_range_band()`
  [plot_experiment2_results.py](</d:/research_history/first_one/research_code/pybullet_ur5_robotiq/plot_experiment2_results.py:1085>)
- `plot_left_right_fingertip_forces()`
  [plot_experiment2_results.py](</d:/research_history/first_one/research_code/pybullet_ur5_robotiq/plot_experiment2_results.py:1131>)

你会看到很多类似：

```python
color="tab:red"
color="tab:blue"
color="tab:green"
color="tab:orange"
color="tab:purple"
```

直接改这些值就可以。

### 4. 改事件标记颜色

看这几个函数：

- `event_legend_handles()`
  [plot_experiment2_results.py](</d:/research_history/first_one/research_code/pybullet_ur5_robotiq/plot_experiment2_results.py:913>)
- `add_event_markers()`
  [plot_experiment2_results.py](</d:/research_history/first_one/research_code/pybullet_ur5_robotiq/plot_experiment2_results.py:945>)
- `add_user_scatter()`
  [plot_experiment2_results.py](</d:/research_history/first_one/research_code/pybullet_ur5_robotiq/plot_experiment2_results.py:955>)

这里分别控制：

- 自动夹紧开始竖线颜色
- 自动松开开始竖线颜色
- 用户 `Z/X` 标记颜色

### 5. 改图例位置

最直接的方法是改 `legend(loc="...")` 里的 `loc`。

当前脚本里常见位置例如：

- `loc="upper right"`
  [plot_experiment2_results.py](</d:/research_history/first_one/research_code/pybullet_ur5_robotiq/plot_experiment2_results.py:994>)
  [plot_experiment2_results.py](</d:/research_history/first_one/research_code/pybullet_ur5_robotiq/plot_experiment2_results.py:1035>)
  [plot_experiment2_results.py](</d:/research_history/first_one/research_code/pybullet_ur5_robotiq/plot_experiment2_results.py:1072>)
  [plot_experiment2_results.py](</d:/research_history/first_one/research_code/pybullet_ur5_robotiq/plot_experiment2_results.py:1110>)
  [plot_experiment2_results.py](</d:/research_history/first_one/research_code/pybullet_ur5_robotiq/plot_experiment2_results.py:1143>)

你可以改成：

- `"upper left"`
- `"lower right"`
- `"lower left"`
- `"best"`

例如：

```python
ax.legend(loc="upper left")
```

### 6. 颜色可以用什么格式

Matplotlib 常见写法都可以：

- 命名色
  `tab:red`
- 十六进制
  `#D55E00`
- RGB 元组
  `(0.2, 0.4, 0.8)`

如果你只是想快速换一套论文配色，最省事的方法就是把脚本里所有 `tab:*` 系列替换掉。

## 12. 常见问题

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

### 5. 为什么默认跑绘图时没有任何正式数据

因为绘图脚本默认只统计：

- `formal_experiment: true`

如果你现在目录里全是调试数据，就会被默认跳过。  
这时有两种做法：

- 正式采数据时，把 YAML 里的 `formal_experiment` 改成 `true`
- 或者在绘图时显式加：

```bash
python plot_experiment2_results.py --data-dir experiment2_data --out-dir experiment2_figures --include-nonformal
```

### 6. `FORCE_ABOVE_HIGH / FORCE_BELOW_LOW / FORCE_BACK_IN_RANGE` 是不是核心控制逻辑

不是。

当前它们主要是事件日志，用来做后处理和可视化参考，不是状态机的核心驱动条件。

## 13. 建议的最小工作流

如果你只是想尽快开始做实验，推荐直接按下面做：

1. 安装依赖
2. 如果没有 ESP32，把 `experiment2_config.yaml` 里的 `esp32.send_to_esp32` 设为 `false`
3. 用 `forcelevel_simulation.py` 找到一个满意的 `initial_action`
4. 调整 `center_object.half_extents` 到你想要的木块大小
5. 调试阶段保持：
   - `formal_experiment: false`
6. 正式采集前改成：
   - `formal_experiment: true`
   - `participant_id: "P01"` 之类的正式编号
7. 运行：

```bash
python experiment2_force_maintenance.py experiment2_config.yaml
```

8. 画正式 summary：

```bash
python plot_experiment2_results.py --data-dir experiment2_data --out-dir experiment2_figures
```

9. 如果想连调试数据一起看：

```bash
python plot_experiment2_results.py --data-dir experiment2_data --out-dir experiment2_figures --include-nonformal
```
