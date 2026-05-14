9. TARGET_RANGE_STABLE 是事件，不是状态：
   在 INIT 状态中，当 contact_force 连续 stable_target_hold_sec 位于 target_low 和 target_high 之间时，触发 TARGET_RANGE_STABLE 事件。
   触发该事件时：
   - trial_start_time = now
   - trial_elapsed_sec 从 0 开始计时
   - state 从 INIT 进入 AUTO_WAIT
   - 每轮 active trial 固定运行 trial_duration_sec 后自动结束

10. dt 定义：
   自动扰动中的 dt 使用固定仿真步长 1/240 秒，不使用 wall-clock delta。
   即每帧扰动量为 auto_grip_speed * (1/240)。
   实验分析用的 time_sec 建议使用仿真累计时间。

11. 接触稳定日志字段：
   接触稳定逻辑使用按秒判断：
   - contact_hold_sec
   - no_contact_hold_sec
   同时日志中保留 contact_frames 和 no_contact_frames 作为调试字段。
   time_series CSV 需要新增：
   - contact_hold_sec
   - no_contact_hold_sec

12. 手动开始：
   如果 INIT 超过 init_timeout_sec，允许实验者按 S 手动开始。
   manual_start=true 时，记录 MANUAL_START 事件。
   触发 MANUAL_START 时：
   - trial_start_time = now
   - trial_elapsed_sec 从 0 开始
   - state 从 INIT 进入 AUTO_WAIT