#include "lanelet2_extension/regulatory_elements/avyana_traffic_light.hpp"

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
template <typename T>
bool findAndErase(const T & primitive, RuleParameters * member)
{
  if (member == nullptr) {
    std::cerr << __FUNCTION__ << ": member is null pointer";
    return false;
  }
  auto it = std::find(member->begin(), member->end(), RuleParameter(primitive));
  if (it == member->end()) {
    return false;
  }
  member->erase(it);
  return true;
}

template <typename T>
RuleParameters toRuleParameters(const std::vector<T> & primitives)
{
  auto cast_func = [](const auto & elem) { return static_cast<RuleParameter>(elem); };
  return utils::transform(primitives, cast_func);
}

template <>
RuleParameters toRuleParameters(const std::vector<LineStringOrPolygon3d> & primitives)
{
  auto cast_func = [](const auto & elem) { return elem.asRuleParameter(); };
  return utils::transform(primitives, cast_func);
}

LineStringsOrPolygons3d getLsOrPoly(const RuleParameterMap & paramsMap, RoleName role)
{
  auto params = paramsMap.find(role);
  if (params == paramsMap.end()) {
    return {};
  }
  LineStringsOrPolygons3d result;
  for (auto & param : params->second) {
    auto l = boost::get<LineString3d>(&param);
    if (l != nullptr) {
      result.push_back(*l);
    }
    auto p = boost::get<Polygon3d>(&param);
    if (p != nullptr) {
      result.push_back(*p);
    }
  }
  return result;
}

[[maybe_unused]] ConstLineStringsOrPolygons3d getConstLsOrPoly(
  const RuleParameterMap & params, RoleName role)
{
  auto cast_func = [](auto & lsOrPoly) {
    return static_cast<ConstLineStringOrPolygon3d>(lsOrPoly);
  };
  return utils::transform(getLsOrPoly(params, role), cast_func);
}

[[maybe_unused]] RegulatoryElementDataPtr constructAvyanaTrafficLightData(
  Id id, const AttributeMap & attributes, const LineStringsOrPolygons3d & trafficLights,
  const Optional<LineString3d> & stopLine, const LineStrings3d & lightBulbs)
{
  RuleParameterMap rpm = {{RoleNameString::Refers, toRuleParameters(trafficLights)}};

  if (!!stopLine) {
    RuleParameters rule_parameters = {*stopLine};
    rpm.insert(std::make_pair(RoleNameString::RefLine, rule_parameters));
  }
  if (!lightBulbs.empty()) {
    rpm.insert(std::make_pair(AvyanaRoleNameString::LightBulbs, toRuleParameters(lightBulbs)));
  }

  auto data = std::make_shared<RegulatoryElementData>(id, rpm, attributes);
  data->attributes[AttributeName::Type] = AttributeValueString::RegulatoryElement;
  data->attributes[AttributeName::Subtype] = AttributeValueString::TrafficLight;
  return data;
}
}  // namespace

AvyanaTrafficLight::AvyanaTrafficLight(const RegulatoryElementDataPtr & data)
: TrafficLight(data)
{
}

AvyanaTrafficLight::AvyanaTrafficLight(
  Id id, const AttributeMap & attributes, const LineStringsOrPolygons3d & trafficLights,
  const Optional<LineString3d> & stopLine, const LineStrings3d & lightBulbs)
: TrafficLight(id, attributes, trafficLights, stopLine)
{
  for (const auto & lightBulb : lightBulbs) {
    addLightBulbs(lightBulb);
  }
}

ConstLineStrings3d AvyanaTrafficLight::lightBulbs() const
{
  return getParameters<ConstLineString3d>(AvyanaRoleNameString::LightBulbs);
}

void AvyanaTrafficLight::addLightBulbs(const LineStringOrPolygon3d & primitive)
{
  parameters()[AvyanaRoleNameString::LightBulbs].emplace_back(primitive.asRuleParameter());
}

bool AvyanaTrafficLight::removeLightBulbs(const LineStringOrPolygon3d & primitive)
{
  return findAndErase(
    primitive.asRuleParameter(), &parameters().find(AvyanaRoleNameString::LightBulbs)->second);
}

}  // namespace lanelet::avyana

// NOLINTEND(readability-identifier-naming)
