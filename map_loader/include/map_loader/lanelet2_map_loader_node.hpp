#ifndef MAP_LOADER__LANELET2_MAP_LOADER_NODE_HPP_
#define MAP_LOADER__LANELET2_MAP_LOADER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <avyana_map_messages/msg/hd_map_bin.hpp>

#include <lanelet2_projection/UTM.h>

#include <memory>
#include <string>

using avyana_map_messages::msg::HDMapBin;

class Lanelet2MapLoaderNode : public rclcpp::Node
{
public:
  explicit Lanelet2MapLoaderNode(const rclcpp::NodeOptions & options);

  static lanelet::LaneletMapPtr load_map(
    rclcpp::Node & node, const std::string & lanelet2_filename,
    const std::string & lanelet2_map_projector_type);
  static HDMapBin create_map_bin_msg(
    const lanelet::LaneletMapPtr map, const std::string & lanelet2_filename,
    const rclcpp::Time & now);

private:
  rclcpp::Publisher<HDMapBin>::SharedPtr pub_map_bin_;
};

#endif  // MAP_LOADER__LANELET2_MAP_LOADER_NODE_HPP_
