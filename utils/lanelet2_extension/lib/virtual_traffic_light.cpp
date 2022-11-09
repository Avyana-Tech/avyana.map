#include "lanelet2_extension/regulatory_elements/virtual_traffic_light.hpp"

#include <boost/variant.hpp>

#include <lanelet2_core/primitives/RegulatoryElement.h>

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

namespace lanelet::avyana
{
namespace
{
RegulatoryElementDataPtr constructVirtualTrafficLightData(
  const Id id, const AttributeMap & attributes, const LineString3d & virtual_traffic_light)
{
  RuleParameterMap rpm;
  RuleParameters rule_parameters = {virtual_traffic_light};
  rpm.insert(std::make_pair(RoleNameString::Refers, rule_parameters));

  auto data = std::make_shared<RegulatoryElementData>(id, rpm, attributes);
  data->attributes[AttributeName::Type] = AttributeValueString::RegulatoryElement;
  data->attributes[AttributeName::Subtype] = "virtual_traffic_light";
  return data;
}
}  // namespace

VirtualTrafficLight::VirtualTrafficLight(const RegulatoryElementDataPtr & data)
: RegulatoryElement(data)
{
  if (getParameters<ConstLineString3d>("start_line").size() != 1) {
    throw InvalidInputError("There must be exactly one start_line defined!");
  }
  if (getParameters<ConstLineString3d>("end_line").empty()) {
    throw InvalidInputError("No end_line defined!");
  }
}

VirtualTrafficLight::VirtualTrafficLight(
  const Id id, const AttributeMap & attributes, const LineString3d & virtual_traffic_light)
: VirtualTrafficLight(constructVirtualTrafficLightData(id, attributes, virtual_traffic_light))
{
}

}  // namespace lanelet::avyana

// NOLINTEND(readability-identifier-naming)
