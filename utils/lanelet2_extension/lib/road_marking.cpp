#include "lanelet2_extension/regulatory_elements/road_marking.hpp"

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
RegulatoryElementDataPtr constructRoadMarkingData(
  Id id, const AttributeMap & attributes, const LineString3d & road_marking)
{
  RuleParameterMap rpm;
  RuleParameters rule_parameters = {road_marking};
  rpm.insert(std::make_pair(RoleNameString::Refers, rule_parameters));

  auto data = std::make_shared<RegulatoryElementData>(id, rpm, attributes);
  data->attributes[AttributeName::Type] = AttributeValueString::RegulatoryElement;
  data->attributes[AttributeName::Subtype] = "road_marking";
  return data;
}
}  // namespace

RoadMarking::RoadMarking(const RegulatoryElementDataPtr & data) : RegulatoryElement(data)
{
  if (getParameters<ConstLineString3d>(RoleName::Refers).size() != 1) {
    throw InvalidInputError("There must be exactly one road marking defined!");
  }
}

RoadMarking::RoadMarking(Id id, const AttributeMap & attributes, const LineString3d & road_marking)
: RoadMarking(constructRoadMarkingData(id, attributes, road_marking))
{
}

ConstLineString3d RoadMarking::roadMarking() const
{
  return getParameters<ConstLineString3d>(RoleName::Refers).front();
}

LineString3d RoadMarking::roadMarking()
{
  return getParameters<LineString3d>(RoleName::Refers).front();
}

void RoadMarking::setRoadMarking(const LineString3d & road_marking)
{
  parameters()[RoleName::Refers] = {road_marking};
}

void RoadMarking::removeRoadMarking() { parameters()[RoleName::Refers] = {}; }

}  // namespace lanelet::avyana

// NOLINTEND(readability-identifier-naming)
