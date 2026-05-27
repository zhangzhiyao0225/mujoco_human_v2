# custom_msgs/msg/ToSimMsg.msg

# 是否开始仿真 / 是否执行初始化动作
bool start_simulation

# 需要重置的执行器/关节名字（还是 actuator 名字）
string[] actuators_name

# 对应每个 actuator（或关节）的初始关节角
float64[] initial_joint_positions
