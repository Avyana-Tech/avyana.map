#include "lanelet2_extension/utility/route_checker.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>

namespace lanelet::utils
{
bool route::isRouteValid(
  const HDMapRoute & route_msg, const lanelet::LaneletMapPtr lanelet_map_ptr_)
{
  for (const auto & route_section : route_msg.segments) {
    for (const auto & primitive : route_section.primitives) {
      const auto id = primitive.id;
      try {
        lanelet_map_ptr_->laneletLayer.get(id);
      } catch (const std::exception & e) {
        std::cerr
          << e.what()
          << ". Maybe the loaded route was created on a different Map from the current one. "
             "Try to load the other Route again."
          << std::endl;
        return false;
      }
    }
  }
  return true;
}

}  // namespace lanelet::utils

// NOLINTEND(readability-identifier-naming)
