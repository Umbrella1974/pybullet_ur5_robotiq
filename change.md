1. 实验结束条件：
   INIT 不计入正式时间；进入 TARGET_RANGE_STABLE 后正式开始；
   每轮 active trial 固定 120 秒，到时自动结束。

2. 扰动等待关系：
   perturb_interval_min/max_sec 是从“用户停止输入或上次扰动结束”到“下一次扰动开始”的总等待时间；
   auto_wait_sec 是最小保护等待时间；
   实际等待 = max(auto_wait_sec, random interval)。

3. 扰动更新方式：
   不使用每帧 auto_grip_step；
   改为 auto_grip_speed，单位 m/s；
   每帧变化量 = auto_grip_speed * dt；
   同时保留 max_auto_delta 和 perturb_duration_sec 作为停止条件。

4. contact_force 定义：
   实验二用 sum，即 tf_force_smooth + if_force_smooth；
   后续 target_low/high、emergency_high 和 level_thresholds 都按 sum 标定。

5. keepalive：
   接受在 experiment2_force_maintenance.py 里单独实现 sender wrapper；
   不修改已有 ESP32FeedbackClient 类。

6. 事件日志：
   FORCE_ABOVE_HIGH、FORCE_BELOW_LOW、FORCE_BACK_IN_RANGE、USER_INPUT_Z/X 等事件全部按边沿触发，只在状态变化时记录一次。

7. 接触稳定判断：
   优先改成按秒判断，例如 contact_hold_sec=0.10、no_contact_hold_sec=0.20；
   如果暂时不改，也要意识到 frame-based 会受帧率影响。

8. INIT：
   INIT 阶段可以有 init_timeout_sec=60；
   超时后允许实验者按 S 手动开始，但正式数据中要标记 manual_start=true。