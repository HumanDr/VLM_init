# 决策/执行层对外发布的自检状态，供 rqt/rviz/debug 订阅。
std_msgs/Header header

float32 linear_cmd
float32 angular_cmd

# CRUISE | CAUTION | AVOID_LEFT | AVOID_RIGHT | STOP | RECOVERY | ESTOP
string state

float32 risk
float32 clearance_center
float32 yaw
bool vlm_stale

# 看门狗降速系数（1.0 = 正常，0.0 = 完全停车）
float32 watchdog_scale
