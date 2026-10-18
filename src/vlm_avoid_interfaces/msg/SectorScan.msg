# 由深度图（或单目几何方法）算出的左/中/右三个扇区的最近相对距离。
# 该话题是 VLM 低频语义决策的高频安全网：VLM 1~2Hz，扇区扫描 15~20Hz。
std_msgs/Header header

# false 表示本帧深度图无效（全零 / 超出量程 / 无数据）
bool valid

# 各扇区距离 [m]，越大越安全；无有效点时填 max_range
float32 left
float32 center
float32 right

# 三扇区中的最小值 [m]
float32 min_range
