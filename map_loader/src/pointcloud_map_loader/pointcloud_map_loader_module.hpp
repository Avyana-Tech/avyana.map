#ifndef POINTCLOUD_MAP_LOADER__POINTCLOUD_MAP_LOADER_MODULE_HPP_
#define POINTCLOUD_MAP_LOADER__POINTCLOUD_MAP_LOADER_MODULE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <boost/optional.hpp>

#include <string>
#include <vector>

class PointcloudMapLoaderModule
{
public:
  explicit PointcloudMapLoaderModule(
    rclcpp::Node * node, const std::vector<std::string> & pcd_paths,
    const std::string publisher_name);

private:
  rclcpp::Logger logger_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_map_;

  sensor_msgs::msg::PointCloud2 loadPCDFiles(const std::vector<std::string> & pcd_paths) const;
};

#endif  // POINTCLOUD_MAP_LOADER__POINTCLOUD_MAP_LOADER_MODULE_HPP_
