#include <rclcpp/node_options.hpp>

#include "multi_controller/multi_controller.hpp"

namespace multi_controller
{
MultiControllerNode::MultiControllerNode(const rclcpp::NodeOptions& options)
  : Node("multi_controller", options)
{
  this->declare_parameter<int>("robot_id", 0);
  this->get_parameter("robot_id", robot_id);

  terrain_map_offset_sub_ = this->create_subscription<multi_controller_interfaces::msg::PointcloudWithOffset>("terrain_map_offset", 2, &MultiControllerNode::TerrainMapOffsetCallBack);
  terrain_map_ext_offset_sub_ = this->create_subscription<multi_controller_interfaces::msg::PointcloudWithOffset>("terrain_map_ext_offset", 2, &MultiControllerNode::TerrainMapExtOffsetCallBack);
  total_terrain_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/total_terrain_map", 2);
  total_terrain_map_ext_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/total_terrain_map_ext", 2);
}

void MultiControllerNode::TerrainMapOffsetCallBack(const multi_controller_interfaces::msg::PointcloudWithOffset::ConstSharedPtr terrain_map_offset_msg){

}

void MultiControllerNode::TerrainMapExtOffsetCallBack(const multi_controller_interfaces::msg::PointcloudWithOffset::ConstSharedPtr terrain_map_ext_offset_msg){

}
} // namespace multi_controller
