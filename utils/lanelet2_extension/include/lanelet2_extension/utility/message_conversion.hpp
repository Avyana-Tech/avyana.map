#ifndef LANELET2_EXTENSION__UTILITY__MESSAGE_CONVERSION_HPP_
#define LANELET2_EXTENSION__UTILITY__MESSAGE_CONVERSION_HPP_

// NOLINTBEGIN(readability-identifier-naming)

#include <avyana_map_messages/msg/hd_map_bin.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

namespace lanelet::utils::conversion
{
/**
 * [toBinMsg converts lanelet2 map to ROS message. Similar implementation to
 * lanelet::io_handlers::BinHandler::write()]
 * @param map [lanelet map data]
 * @param msg [converted ROS message. Only "data" field is filled]
 */
void toBinMsg(const lanelet::LaneletMapPtr & map, avyana_map_messages::msg::HDMapBin * msg);

/**
 * [fromBinMsg converts ROS message into lanelet2 data. Similar implementation
 * to lanelet::io_handlers::BinHandler::parse()]
 * @param msg [ROS message for lanelet map]
 * @param map [Converted lanelet2 data]
 */
void fromBinMsg(const avyana_map_messages::msg::HDMapBin & msg, lanelet::LaneletMapPtr map);
void fromBinMsg(
  const avyana_map_messages::msg::HDMapBin & msg, lanelet::LaneletMapPtr map,
  lanelet::traffic_rules::TrafficRulesPtr * traffic_rules,
  lanelet::routing::RoutingGraphPtr * routing_graph);

/**
 * [toGeomMsgPt converts various point types to geometry_msgs point]
 * @param src [input point(geometry_msgs::msg::Point3,
 * Eigen::VEctor3d=lanelet::BasicPoint3d, lanelet::Point3d, lanelet::Point2d) ]
 * @param dst [converted geometry_msgs point]
 */
void toGeomMsgPt(const geometry_msgs::msg::Point32 & src, geometry_msgs::msg::Point * dst);
void toGeomMsgPt(const Eigen::Vector3d & src, geometry_msgs::msg::Point * dst);
void toGeomMsgPt(const lanelet::ConstPoint3d & src, geometry_msgs::msg::Point * dst);
void toGeomMsgPt(const lanelet::ConstPoint2d & src, geometry_msgs::msg::Point * dst);

/**
 * [toGeomMsgPt converts various point types to geometry_msgs point]
 * @param src [input point(geometry_msgs::msg::Point3,
 * Eigen::VEctor3d=lanelet::BasicPoint3d, lanelet::Point3d, lanelet::Point2d) ]
 * @return    [converted geometry_msgs point]
 */
geometry_msgs::msg::Point toGeomMsgPt(const geometry_msgs::msg::Point32 & src);
geometry_msgs::msg::Point toGeomMsgPt(const Eigen::Vector3d & src);
geometry_msgs::msg::Point toGeomMsgPt(const lanelet::ConstPoint3d & src);
geometry_msgs::msg::Point toGeomMsgPt(const lanelet::ConstPoint2d & src);

lanelet::ConstPoint3d toLaneletPoint(const geometry_msgs::msg::Point & src);
void toLaneletPoint(const geometry_msgs::msg::Point & src, lanelet::ConstPoint3d * dst);

/**
 * [toGeomMsgPoly converts lanelet polygon to geometry_msgs polygon]
 * @param ll_poly   [input polygon]
 * @param geom_poly [converted geometry_msgs point]
 */
void toGeomMsgPoly(
  const lanelet::ConstPolygon3d & ll_poly, geometry_msgs::msg::Polygon * geom_poly);

/**
 * [toGeomMsgPt32 converts Eigen::Vector3d(lanelet:BasicPoint3d to
 * geometry_msgs::msg::Point32)]
 * @param src [input point]
 * @param dst [converted point]
 */
void toGeomMsgPt32(const Eigen::Vector3d & src, geometry_msgs::msg::Point32 * dst);

}  // namespace lanelet::utils::conversion

// NOLINTEND(readability-identifier-naming)

#endif  // LANELET2_EXTENSION__UTILITY__MESSAGE_CONVERSION_HPP_
