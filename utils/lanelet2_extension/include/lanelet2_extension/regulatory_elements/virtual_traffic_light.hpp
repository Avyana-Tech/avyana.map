#ifndef LANELET2_EXTENSION__REGULATORY_ELEMENTS__VIRTUAL_TRAFFIC_LIGHT_HPP_
#define LANELET2_EXTENSION__REGULATORY_ELEMENTS__VIRTUAL_TRAFFIC_LIGHT_HPP_

// NOLINTBEGIN(readability-identifier-naming)

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/LineString.h>

#include <memory>
#include <vector>

namespace lanelet::avyana
{
class VirtualTrafficLight : public lanelet::RegulatoryElement
{
public:
  using Ptr = std::shared_ptr<VirtualTrafficLight>;
  using ConstPtr = std::shared_ptr<const VirtualTrafficLight>;
  static constexpr char RuleName[] = "virtual_traffic_light";

  static Ptr make(
    const Id id, const AttributeMap & attributes, const LineString3d & virtual_traffic_light)
  {
    return Ptr{new VirtualTrafficLight(id, attributes, virtual_traffic_light)};
  }

  [[nodiscard]] ConstLineString3d getVirtualTrafficLight() const
  {
    return getParameters<ConstLineString3d>(RoleName::Refers).front();
  }

  [[nodiscard]] Optional<ConstLineString3d> getStopLine() const
  {
    const auto vec = getParameters<ConstLineString3d>(RoleName::RefLine);
    if (vec.empty()) {
      return {};
    }
    return vec.front();
  }

  [[nodiscard]] ConstLineString3d getStartLine() const
  {
    return getParameters<ConstLineString3d>("start_line").front();
  }

  [[nodiscard]] ConstLineStrings3d getEndLines() const
  {
    return getParameters<ConstLineString3d>("end_line");
  }

private:
  // the following lines are required so that lanelet2 can create this object
  // when loading a map with this regulatory element
  friend class lanelet::RegisterRegulatoryElement<VirtualTrafficLight>;

  VirtualTrafficLight(
    const Id id, const AttributeMap & attributes, const LineString3d & virtual_traffic_light);

  explicit VirtualTrafficLight(const lanelet::RegulatoryElementDataPtr & data);
};

static lanelet::RegisterRegulatoryElement<VirtualTrafficLight> regVirtualTrafficLight;

}  // namespace lanelet::avyana

// NOLINTEND(readability-identifier-naming)

#endif  // LANELET2_EXTENSION__REGULATORY_ELEMENTS__VIRTUAL_TRAFFIC_LIGHT_HPP_
