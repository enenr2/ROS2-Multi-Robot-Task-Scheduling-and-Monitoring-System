// 感知节点：订阅任务相机的图像与内参，输出 map 系下的三色目标位置。
//
// 本节点只做"接线"：颜色分割在 ColorDetector、测距在 Ranging、时序去抖在
// DetectionDebouncer，三者都是纯 C++ 且已有单测覆盖。这里负责
// 取图 -> 检测 -> 测距 -> 去抖 -> TF 变换 -> 发布 fleet_msgs/Detection。
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "fleet_msgs/msg/detection.hpp"
#include "fleet_perception/color_detector.hpp"
#include "fleet_perception/debouncer.hpp"
#include "fleet_perception/ranging.hpp"

namespace fleet {

namespace {

constexpr const char* kColorNames[4] = {"UNKNOWN", "RED", "GREEN", "BLUE"};

uint8_t ToMsgColor(Color c)
{
  switch (c) {
    case Color::kRed:   return fleet_msgs::msg::Detection::COLOR_RED;
    case Color::kGreen: return fleet_msgs::msg::Detection::COLOR_GREEN;
    case Color::kBlue:  return fleet_msgs::msg::Detection::COLOR_BLUE;
    case Color::kUnknown: break;
  }
  return 0;
}

}  // namespace

class PerceptionNode : public rclcpp::Node
{
public:
  PerceptionNode()
  : Node("perception_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_, this, false)
  {
    robot_id_ = declare_parameter<std::string>("robot_id", "");
    if (robot_id_.empty()) {
      // 空则退化为命名空间名（/robot1 -> robot1），保证一键启动时不用重复配置
      const std::string ns = get_namespace();
      robot_id_ = ns.size() > 1 ? ns.substr(1) : "robot";
    }

    world_frame_ = declare_parameter<std::string>("world_frame", "map");
    camera_frame_param_ = declare_parameter<std::string>("camera_frame", "");
    hfov_deg_ = declare_parameter<double>("hfov_deg", 60.0);
    publish_debug_ = declare_parameter<bool>("publish_debug_image", false);

    RangeConfig range_cfg;
    range_cfg.target_width = declare_parameter<double>("target_width", 0.2);
    range_cfg.min_range = declare_parameter<double>("min_range", 0.4);
    range_cfg.max_range = declare_parameter<double>("max_range", 5.0);
    range_cfg.min_bbox_width_px = declare_parameter<int>("min_bbox_width_px", 10);
    range_cfg.edge_sigma_px = declare_parameter<double>("edge_sigma_px", 2.0);
    ranging_ = std::make_unique<Ranging>(range_cfg);

    DebounceConfig db_cfg;
    db_cfg.required_frames = declare_parameter<int>("debounce_frames", 3);
    db_cfg.centroid_tolerance_px = declare_parameter<double>("debounce_centroid_px", 10.0);
    db_cfg.range_tolerance_m = declare_parameter<double>("debounce_range_m", 0.15);
    db_cfg.lost_after_missing_frames = declare_parameter<int>("lost_frames", 15);

    DetectorConfig det_cfg = ColorDetector::DefaultConfig();
    det_cfg.min_area_px = declare_parameter<int>("min_area_px", 60);
    detector_ = std::make_unique<ColorDetector>(det_cfg);

    for (Color c : {Color::kRed, Color::kGreen, Color::kBlue}) {
      debouncers_[c] = std::make_unique<DetectionDebouncer>(db_cfg);
    }

    det_pub_ = create_publisher<fleet_msgs::msg::Detection>("detections", rclcpp::QoS(10));
    if (publish_debug_) {
      debug_pub_ = create_publisher<sensor_msgs::msg::Image>("perception/debug_image", rclcpp::QoS(2));
    }

    // 仿真里图像与内参都是传感器数据流，用 best effort 订阅兼容性好
    // （reliable 的 gz 桥接发布者向下兼容 best effort 订阅者）
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera/camera_info", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) { OnCameraInfo(*msg); });

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "camera/image", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::SharedPtr msg) { OnImage(msg); });

    RCLCPP_INFO(get_logger(),
                "感知节点就绪: robot=%s 图像=%s 输出=%s world_frame=%s target_width=%.3fm",
                robot_id_.c_str(), image_sub_->get_topic_name(), det_pub_->get_topic_name(),
                world_frame_.c_str(), ranging_->config().target_width);
  }

private:
  void OnCameraInfo(const sensor_msgs::msg::CameraInfo& msg)
  {
    if (msg.k[0] <= 0.0) {
      return;
    }
    CameraModel cam;
    cam.fx = msg.k[0];
    cam.fy = msg.k[4];
    cam.cx = msg.k[2];
    cam.cy = msg.k[5];
    cam.width = static_cast<int>(msg.width);
    cam.height = static_cast<int>(msg.height);
    camera_model_ = cam;
    has_camera_model_ = true;

    if (!msg.header.frame_id.empty()) {
      frame_from_camera_info_ = msg.header.frame_id;
    }

    if (!logged_camera_info_) {
      logged_camera_info_ = true;
      RCLCPP_INFO(get_logger(), "相机内参: fx=%.2f fy=%.2f cx=%.2f cy=%.2f (%dx%d) frame=%s",
                  cam.fx, cam.fy, cam.cx, cam.cy, cam.width, cam.height,
                  frame_from_camera_info_.c_str());
    }
  }

  // 内参缺失时按水平视场角推出一个等效内参，保证节点能先跑起来，
  // 但会打一次警告——测距精度依赖真实内参，正式验收必须看到内参日志
  CameraModel FallbackCameraModel(int width, int height) const
  {
    CameraModel cam;
    const double hfov = hfov_deg_ * M_PI / 180.0;
    cam.fx = (static_cast<double>(width) / 2.0) / std::tan(hfov / 2.0);
    cam.fy = cam.fx;
    cam.cx = static_cast<double>(width) / 2.0;
    cam.cy = static_cast<double>(height) / 2.0;
    cam.width = width;
    cam.height = height;
    return cam;
  }

  std::string CameraFrame() const
  {
    if (!camera_frame_param_.empty()) {
      return camera_frame_param_;
    }
    if (!frame_from_camera_info_.empty()) {
      return frame_from_camera_info_;
    }
    return robot_id_ + "_camera_link";
  }

  void PublishLost(Color color, const std_msgs::msg::Header& header)
  {
    fleet_msgs::msg::Detection det;
    det.header = header;
    det.header.frame_id = world_frame_;
    det.source_robot_id = robot_id_;
    det.color = ToMsgColor(color);
    det.confidence = 0.0f;
    det.lost = true;
    det.range = -1.0f;
    det_pub_->publish(det);
    RCLCPP_WARN(get_logger(), "%s 目标丢失（连续丢帧超阈值），已上报丢失事件", kColorNames[
      static_cast<int>(color)]);
  }

  void PublishDetection(Color color, const Blob& blob, const RangeEstimate& est,
                        const std_msgs::msg::Header& header)
  {
    const std::string camera_frame = CameraFrame();

    geometry_msgs::msg::PointStamped cam_pt;
    cam_pt.header = header;
    cam_pt.header.frame_id = camera_frame;
    cam_pt.point.x = est.point_cam.x;
    cam_pt.point.y = est.point_cam.y;
    cam_pt.point.z = est.point_cam.z;

    geometry_msgs::msg::PointStamped world_pt;
    try {
      // 用图像时间戳查 TF：仿真下 /clock 与图像戳同源，能得到与该帧严格对应的位姿。
      // 刻意不传超时：TF 数据由本节点的执行器喂给 Buffer，在回调里带超时等待
      // 等于自己等自己（tf2 会直接报错），拿不到就丢这一帧，去抖器会兜住。
      world_pt = tf_buffer_.transform(cam_pt, world_frame_);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "TF %s -> %s 查询失败，本帧检测丢弃: %s",
                           camera_frame.c_str(), world_frame_.c_str(), ex.what());
      dropped_frames_++;
      return;
    }

    fleet_msgs::msg::Detection det;
    det.header = header;
    det.header.frame_id = world_frame_;
    det.source_robot_id = robot_id_;
    det.color = ToMsgColor(color);
    det.confidence = static_cast<float>(blob.confidence);
    det.lost = false;
    det.bbox.x_offset = static_cast<uint32_t>(std::max(0, blob.bbox.x));
    det.bbox.y_offset = static_cast<uint32_t>(std::max(0, blob.bbox.y));
    det.bbox.width = static_cast<uint32_t>(std::max(0, blob.bbox.width));
    det.bbox.height = static_cast<uint32_t>(std::max(0, blob.bbox.height));
    det.world_pose = world_pt.point;
    det.range = static_cast<float>(est.range);
    det.bearing = static_cast<float>(est.bearing);
    det.range_sigma = static_cast<float>(est.sigma);
    det_pub_->publish(det);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                         "%s: 像素框 %dx%d @(%d,%d) 距离 %.2f±%.2fm 偏角 %.1f° "
                         "world=(%.2f, %.2f, %.2f)",
                         kColorNames[static_cast<int>(color)], blob.bbox.width, blob.bbox.height,
                         blob.bbox.x, blob.bbox.y, est.range, est.sigma,
                         est.bearing * 180.0 / M_PI, world_pt.point.x, world_pt.point.y,
                         world_pt.point.z);
  }

  void OnImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (const cv_bridge::Exception& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "图像转换失败: %s", ex.what());
      return;
    }

    const CameraModel cam = has_camera_model_ ? camera_model_
                                               : FallbackCameraModel(msg->width, msg->height);
    if (!has_camera_model_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "尚未收到 camera_info，暂用 %.0f° 视场角推导的内参（测距仅作参考）",
                           hfov_deg_);
    }

    const std::vector<Blob> blobs = detector_->Detect(cv_ptr->image);
    std::map<Color, const Blob*> by_color;
    for (const Blob& b : blobs) {
      by_color[b.color] = &b;
    }

    cv::Mat debug;
    if (publish_debug_) {
      debug = cv_ptr->image.clone();
    }

    for (Color color : {Color::kRed, Color::kGreen, Color::kBlue}) {
      const Blob* blob = nullptr;
      const auto it = by_color.find(color);
      if (it != by_color.end()) {
        blob = it->second;
      }

      RangeEstimate est;
      bool usable = false;
      if (blob != nullptr) {
        est = ranging_->Compute(cam, blob->bbox);
        usable = est.valid;
      }

      const DebounceState st = debouncers_[color]->Update(
        usable, blob != nullptr ? blob->bbox.CenterU() : 0.0,
        blob != nullptr ? blob->bbox.CenterV() : 0.0, usable ? est.range : 0.0);

      if (st.lost) {
        PublishLost(color, msg->header);
      }
      if (st.emit && usable && blob != nullptr) {
        PublishDetection(color, *blob, est, msg->header);
      }

      if (publish_debug_ && blob != nullptr) {
        const cv::Scalar draw_color = color == Color::kRed ? cv::Scalar(0, 0, 255)
                                     : color == Color::kGreen ? cv::Scalar(0, 255, 0)
                                                              : cv::Scalar(255, 0, 0);
        cv::rectangle(debug,
                      cv::Rect(blob->bbox.x, blob->bbox.y, blob->bbox.width, blob->bbox.height),
                      draw_color, 1);
        std::string text = std::string(kColorNames[static_cast<int>(color)]);
        if (usable) {
          char buf[32];
          std::snprintf(buf, sizeof(buf), " %.2fm", est.range);
          text += buf;
        }
        cv::putText(debug, text, cv::Point(blob->bbox.x, std::max(10, blob->bbox.y - 3)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.35, draw_color, 1);
      }
    }

    if (publish_debug_ && !debug.empty()) {
      debug_pub_->publish(*cv_bridge::CvImage(msg->header, "bgr8", debug).toImageMsg());
    }
  }

  std::string robot_id_;
  std::string world_frame_;
  std::string camera_frame_param_;
  std::string frame_from_camera_info_;
  double hfov_deg_{60.0};
  bool publish_debug_{false};
  bool has_camera_model_{false};
  bool logged_camera_info_{false};
  int dropped_frames_{0};

  CameraModel camera_model_;
  std::unique_ptr<Ranging> ranging_;
  std::unique_ptr<ColorDetector> detector_;
  std::map<Color, std::unique_ptr<DetectionDebouncer>> debouncers_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<fleet_msgs::msg::Detection>::SharedPtr det_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
};

}  // namespace fleet

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fleet::PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
