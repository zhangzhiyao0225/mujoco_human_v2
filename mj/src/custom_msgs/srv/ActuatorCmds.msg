# custom_msgs/msg/ActuatorCmds.msg

# 每个执行器的名字，对应 MuJoCo 里面 actuator 的 name
string[] actuators_name

# 目标关节位置/速度
float64[] pos
float64[] vel

# PD 参数
float64[] kp
float64[] kd

# 额外扭矩偏置
float64[] torque

# 力矩限幅（逐关节）
float64[] torque_limit
