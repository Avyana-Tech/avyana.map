#ifndef LANELET2_EXTENSION__REGULATORY_ELEMENTS__ROAD_MARKING_HPP_
#define LANELET2_EXTENSION__REGULATORY_ELEMENTS__ROAD_MARKING_HPP_

// NOLINTBEGIN(readability-identifier-naming)

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/LineString.h>

#include <memory>
#include <vector>

namespace lanelet::avyana
{
class RoadMarking : public lanelet::RegulatoryElement
{
public:
  using Ptr = std::shared_ptr<RoadMarking>;
  static constexpr char RuleName[] = "road_marking";

  //! Directly construct a stop line from its required rule parameters.
  //! Might modify the input data in oder to get correct tags.
  static Ptr make(Id id, const AttributeMap & attributes, const LineString3d & road_marking)
  {
    return Ptr{new RoadMarking(id, attributes, road_marking)};
  }

  /**
   * @brief get the relevant road marking
   * @return road marking
   */
  [[nodiscard]] ConstLineString3d roadMarking() const;
  [[nodiscard]] LineString3d roadMarking();

  /**
   * @brief add a new road marking
   * @param primitive road marking to add
   */
  void setRoadMarking(const LineString3d & road_marking);

  /**
   * @brief remove a road marking
   */
  void removeRoadMarking();

private:
  // the following lines are required so that lanelet2 can create this object
  // when loading a map with this regulatory element
  friend class lanelet::RegisterRegulatoryElement<RoadMarking>;
  RoadMarking(Id id, const AttributeMap & attributes, const LineString3d & road_marking);
  explicit RoadMarking(const lanelet::RegulatoryElementDataPtr & data);
};
static lanelet::RegisterRegulatoryElement<RoadMarking> regRoadMarking;

}  // namespace lanelet::avyana

// NOLINTEND(readability-identifier-naming)

#endif  // LANELET2_EXTENSION__REGULATORY_ELEMENTS__ROAD_MARKING_HPP_
