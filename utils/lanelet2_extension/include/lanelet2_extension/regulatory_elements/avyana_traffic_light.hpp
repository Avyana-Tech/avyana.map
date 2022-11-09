#ifndef LANELET2_EXTENSION__REGULATORY_ELEMENTS__AVYANA_TRAFFIC_LIGHT_HPP_
#define LANELET2_EXTENSION__REGULATORY_ELEMENTS__AVYANA_TRAFFIC_LIGHT_HPP_

// NOLINTBEGIN(readability-identifier-naming)

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <memory>
#include <vector>

namespace lanelet::avyana
{
struct AvyanaRoleNameString
{
  static constexpr const char LightBulbs[] = "light_bulbs";
};

class AvyanaTrafficLight : public lanelet::TrafficLight
{
public:
  using Ptr = std::shared_ptr<AvyanaTrafficLight>;
  static constexpr char RuleName[] = "traffic_light";

  //! Directly construct a stop line from its required rule parameters.
  //! Might modify the input data in oder to get correct tags.
  static Ptr make(
    Id id, const AttributeMap & attributes, const LineStringsOrPolygons3d & trafficLights,
    const Optional<LineString3d> & stopLine = {}, const LineStrings3d & lightBulbs = {})
  {
    return Ptr{new AvyanaTrafficLight(id, attributes, trafficLights, stopLine, lightBulbs)};
  }

  /**
   * @brief get the relevant traffic light bulbs
   * @return the traffic light bulbs
   *
   * There might be multiple traffic light bulbs but they are required to show
   * the same signal.
   */
  [[nodiscard]] ConstLineStrings3d lightBulbs() const;

  /**
   * @brief add a new traffic light bulb
   * @param primitive the traffic light bulb to add
   *
   * Traffic light bulbs are represented as linestrings with each point
   * expressing position of each light bulb (lamp).
   */
  void addLightBulbs(const LineStringOrPolygon3d & primitive);

  /**
   * @brief remove a traffic light bulb
   * @param primitive the primitive
   * @return true if the traffic light bulb existed and was removed
   */
  bool removeLightBulbs(const LineStringOrPolygon3d & primitive);

private:
  // the following lines are required so that lanelet2 can create this object
  // when loading a map with this regulatory element
  friend class lanelet::RegisterRegulatoryElement<AvyanaTrafficLight>;
  AvyanaTrafficLight(
    Id id, const AttributeMap & attributes, const LineStringsOrPolygons3d & trafficLights,
    const Optional<LineString3d> & stopLine, const LineStrings3d & lightBulbs);
  explicit AvyanaTrafficLight(const lanelet::RegulatoryElementDataPtr & data);
};
static lanelet::RegisterRegulatoryElement<AvyanaTrafficLight> regAvyanaTraffic;

}  // namespace lanelet::avyana

// NOLINTEND(readability-identifier-naming)

#endif  // LANELET2_EXTENSION__REGULATORY_ELEMENTS__AVYANA_TRAFFIC_LIGHT_HPP_
