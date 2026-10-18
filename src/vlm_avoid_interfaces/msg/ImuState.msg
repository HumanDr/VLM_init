# 由 IMU 互补滤波得到的姿态估计（相机/IMU 融合链路中的状态估计环节）。
std_msgs/Header header

# 弧度
float32 roll
float32 pitch
float32 yaw          # 未校正的连续偏航角，[-pi, pi] 包裹前的积分值会累积
float32 yaw_rate     # rad/s

# false 表示 IMU 数据不足或已超时
bool valid
