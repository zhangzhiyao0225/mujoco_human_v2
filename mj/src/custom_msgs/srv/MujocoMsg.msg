# custom_msgs/msg/MujocoMsg.msg

# 地面反力向量（12 维：4 条腿 * 每条腿 3 维力）(humanoid 下肢：6+6 上肢 6+6 腰部 1 头部 1)
float64[] ground_reaction_force

# 每条腿的接触状态（0/1），长度固定为 4
int32[4] contact_state
