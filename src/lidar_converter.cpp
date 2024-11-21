#include "lidar_converter.hpp"

LidarConverter::LidarConverter() 
  : Node("lidar_converter")
{
  // Initialize the subscription to PointCloud2
  pointcloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/nonground" /*ground_filter output topic name*/,
    rclcpp::QoS(10),
    std::bind(&LidarConverter::callbackPointCloud, this, std::placeholders::_1)
  );

  // Initialize the publisher for CustomMsg
  custommsg_publisher_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>(
    "/livox/lidar" /*fast_lio expected input topic name*/,
    rclcpp::QoS(10)
  );
}

void LidarConverter::callbackPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // Create an empty CustomMsg
  auto custom_msg = livox_ros_driver2::msg::CustomMsg();
  std::vector<livox_ros_driver2::msg::CustomPoint> custom_points;

  // Conversion logic from PointCloud2 to CustomMsg
  custom_msg.header = msg->header;
  custom_msg.timebase = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
  custom_msg.point_num = msg->width;
  custom_msg.lidar_id = 0;
  for (unsigned int i=0; i < msg->width; i++) {
    uint8_t* point_ptr = &msg->data[i * msg->point_step];
    float x, y, z, intensity;
    uint8_t tag, line;
    std::memcpy(&x, point_ptr + 0, sizeof(float));
    std::memcpy(&y, point_ptr + 4, sizeof(float));
    std::memcpy(&z, point_ptr + 8, sizeof(float));
    std::memcpy(&intensity, point_ptr + 12, sizeof(float));
    intensity = std::clamp(intensity, 0.0f, 255.0f) /*optional: clamp intensity values within [0, 255]*/;
    tag = *(point_ptr + 16);
    line = *(point_ptr + 17);
    livox_ros_driver2::msg::CustomPoint custom_point;
    custom_point.offset_time = 0;
    custom_point.x = x;
    custom_point.y = y;
    custom_point.z = z;
    custom_point.reflectivity = static_cast<uint8_t>(intensity + 0.5f/*rounding*/);
    custom_point.tag = tag;
    custom_point.line = line;
    custom_points.push_back(custom_point);
  }
  custom_msg.points = custom_points;

  // Publish the CustomMsg
  custommsg_publisher_->publish(custom_msg);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarConverter>());
  rclcpp::shutdown();
  return 0;
}
