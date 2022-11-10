#ifndef MAP_LOADER__LANELET2_MAP_VISUALIZATION_NODE_HPP_
#define MAP_LOADER__LANELET2_MAP_VISUALIZATION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <avyana_map_messages/msg/hd_map_bin.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>

class Lanelet2MapVisualizationNode : public rclcpp::Node
{
public:
  explicit Lanelet2MapVisualizationNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<avyana_map_messages::msg::HDMapBin>::SharedPtr sub_map_bin_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;

  bool viz_lanelets_centerline_;

  void onMapBin(const avyana_map_messages::msg::HDMapBin::ConstSharedPtr msg);
};

#endif  // MAP_LOADER__LANELET2_MAP_VISUALIZATION_NODE_HPP_
