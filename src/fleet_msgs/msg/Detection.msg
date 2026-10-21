# 视觉检测结果（感知 -> 调度）
# header.stamp      : 图像时间戳
# header.frame_id   : world_pose 所在坐标系，固定为 "map"
std_msgs/Header header

uint8 COLOR_RED=1
uint8 COLOR_GREEN=2
uint8 COLOR_BLUE=3

string source_robot_id
uint8 color
float32 confidence           # 轮廓面积 / 参考面积，>1 表示目标偏大（更近）

# 像素域：基于与 header.stamp 同一帧的图像
sensor_msgs/RegionOfInterest bbox

# 世界域：map 系下的目标估计位置（z 为估计高度）
geometry_msgs/Point world_pose
float32 range                # 单目测距结果（米）
float32 bearing              # 相对相机光轴的水平偏角（弧度）
float32 range_sigma          # 本次测距的不确定度估计（米）
