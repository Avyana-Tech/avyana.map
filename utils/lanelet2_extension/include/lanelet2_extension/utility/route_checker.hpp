#ifndef LANELET2_EXTENSION__UTILITY__ROUTE_CHECKER_HPP_
#define LANELET2_EXTENSION__UTILITY__ROUTE_CHECKER_HPP_

// NOLINTBEGIN(readability-identifier-naming)

#include <lanelet2_extension/utility/query.hpp>
#include <rclcpp/rclcpp.hpp>

#include <avyana_map_messages/msg/hd_map_bin.hpp>
#include <avyana_map_messages/msg/hd_map_route.hpp>

namespace lanelet::utils::route
{
using avyana_map_messages::msg::HDMapBin;
using avyana_map_messages::msg::HDMapRoute;

bool isRouteValid(const HDMapRoute & route, const lanelet::LaneletMapPtr lanelet_map_ptr_);
}  // namespace lanelet::utils::route

// NOLINTEND(readability-identifier-naming)

#endif  // LANELET2_EXTENSION__UTILITY__ROUTE_CHECKER_HPP_
