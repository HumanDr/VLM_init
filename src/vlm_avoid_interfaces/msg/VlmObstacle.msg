# 视觉语言模型（Qwen-VL）对当前画面的障碍物语义判定结果。
std_msgs/Header header

# 画面中是否存在需要主动规避的障碍物
bool obstacle

# 0.0 ~ 1.0 的碰撞风险（模型输出，经时间衰减后的值）
float32 risk

# 障碍物相对机器人前进方向的方位: left | center | right | none
string direction

# 模型建议动作: go_straight | turn_left | turn_right | stop
string action

# 语义标签，例如 "红色的箱子" / "行人" / "垃圾筒"
string label

# 0.0 ~ 1.0 模型置信度
float32 confidence

# true 表示该结果已过期（推理超时 / 服务不可用），下游应按降级策略处理
bool stale

# 本次推理端到端耗时（毫秒）
float64 latency_ms

# 模型原始返回文本，仅用于调试
string raw_text
