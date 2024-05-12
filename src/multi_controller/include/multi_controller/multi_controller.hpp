#ifndef MULTI_CONTROLLER
#define MULTI_CONTROLLER

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>

#include "multi_controller_interfaces/msg/pointcloud_with_offset.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace multi_controller
{
class MultiControllerNode : public rclcpp::Node
{
public:
  MultiControllerNode(const rclcpp::NodeOptions& options);

private:
  int robot_id;

  void TerrainMapOffsetCallBack(
    const multi_controller_interfaces::msg::PointcloudWithOffset::ConstSharedPtr
      terrain_map_offset_msg);
  void TerrainMapExtOffsetCallBack(
    const multi_controller_interfaces::msg::PointcloudWithOffset::ConstSharedPtr
      terrain_map_ext_offset_msg);

  rclcpp::Subscription<multi_controller_interfaces::msg::PointcloudWithOffset>::SharedPtr
    terrain_map_offset_sub_;
  rclcpp::Subscription<multi_controller_interfaces::msg::PointcloudWithOffset>::SharedPtr
    terrain_map_ext_offset_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr total_terrain_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr total_terrain_map_ext_pub_;
};
} // namespace multi_controller

#endif