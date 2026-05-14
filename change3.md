请在 experiment2_force_maintenance.py 中增加“指尖中心标准接触物体”机制，用于实验二初始化。

目标：
为了避免每次手动移动机械臂到物体附近造成碰撞、左右指尖距离不一致、接触力偏置等问题，在 reset 后、move_to_initial_pose 后、INIT 前，读取左右指尖 link 的世界坐标，并在两指尖中点生成一个标准接触物体。实验二的接触力读取对象使用这个新生成物体。

配置：
在 experiment2_config.yaml 中新增：

center_object:
  enabled: true
  replace_env_box: true
  mass: 0.0
  half_extents: [0.018, 0.018, 0.025]
  position_offset: [0.0, 0.0, 0.0]

实现要求：
1. move_to_initial_pose(env, action, steps) 完成后，再创建 centered contact object。
2. 使用 p.getLinkState(env.robot.id, TF_LINK) 和 p.getLinkState(env.robot.id, IF_LINK) 获取左右指尖世界坐标。
3. center_pos = (left_pos + right_pos) / 2。
4. center_pos += position_offset。
5. 用 p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents) 创建方块碰撞体。
6. 用 p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents) 创建可视体。
7. 用 p.createMultiBody(baseMass=mass, ...) 创建目标物体。
8. mass 默认为 0.0，表示固定物体，避免掉落或被夹飞。
9. 如果 replace_env_box=true 且 env.boxID 存在，则 removeBody(env.boxID)，然后将 env.boxID 替换为新物体 id。
10. 后续 read_separate_fingertip_forces(env) 仍然使用 p.getContactPoints(bodyA=env.robot.id, bodyB=env.boxID)。
11. 该 centered contact object 只在每轮实验初始化时创建一次，不要每帧更新。
12. 创建完成后进入 INIT，允许用户微调；按 S 后正式开始 trial。

注意：
- 不要改变物体大小来制造扰动。
- 扰动仍然通过 gripper_opening 的自动夹紧/自动松开实现。
- 物体生成前应确保 initial_action.gripper_opening 足够大，避免初始穿模或力尖峰。