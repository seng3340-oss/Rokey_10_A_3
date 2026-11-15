# 사전에 측정한 BASE 기준 고정 위치다. geometry_msgs/Pose 위치는 m 단위다.
# 이 메시지에 실제 이동 가능한 좌표를 자동으로 채우지 않는다.
geometry_msgs/Pose cake_pickup_pose
geometry_msgs/Pose workstation_pose
geometry_msgs/Pose tool_holder_pose
geometry_msgs/Pose delivery_pose
geometry_msgs/Point cake_center
float64 cake_radius
float64 cake_height
# 평면 방향은 rad, 접근·접촉 오프셋은 m다.
float64 cake_orientation
float64 approach_height
float64 drawing_z_offset
