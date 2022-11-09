#ifndef LANELET2_EXTENSION__REGULATORY_ELEMENTS__DETECTION_AREA_HPP_
#define LANELET2_EXTENSION__REGULATORY_ELEMENTS__DETECTION_AREA_HPP_

// NOLINTBEGIN(readability-identifier-naming)

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <memory>
#include <vector>

namespace lanelet::avyana
{
class DetectionArea : public lanelet::RegulatoryElement
{
public:
  using Ptr = std::shared_ptr<DetectionArea>;
  static constexpr char RuleName[] = "detection_area";

  //! Directly construct a stop line from its required rule parameters.
  //! Might modify the input data in oder to get correct tags.
  static Ptr make(
    Id id, const AttributeMap & attributes, const Polygons3d & detectionAreas,
    const LineString3d & stopLine)
  {
    return Ptr{new DetectionArea(id, attributes, detectionAreas, stopLine)};
  }

  /**
   * @brief get the relevant detection_areas
   * @return detection_areas
   */
  [[nodiscard]] ConstPolygons3d detectionAreas() const;
  [[nodiscard]] Polygons3d detectionAreas();

  /**
   * @brief add a new detection area
   * @param primitive detection area to add
   */
  void addDetectionArea(const Polygon3d & primitive);

  /**
   * @brief remove a detection area
   * @param primitive the primitive
   * @return true if the detection area existed and was removed
   */
  bool removeDetectionArea(const Polygon3d & primitive);

  /**
   * @brief get the stop line for the detection area
   * @return the stop line as LineString
   */
  [[nodiscard]] ConstLineString3d stopLine() const;
  [[nodiscard]] LineString3d stopLine();

  /**
   * @brief set a new stop line, overwrite the old one
   * @param stopLine new stop line
   */
  void setStopLine(const LineString3d & stopLine);

  //! Deletes the stop line
  void removeStopLine();

private:
  // the following lines are required so that lanelet2 can create this object
  // when loading a map with this regulatory element
  friend class lanelet::RegisterRegulatoryElement<DetectionArea>;
  DetectionArea(
    Id id, const AttributeMap & attributes, const Polygons3d & detectionAreas,
    const LineString3d & stopLine);
  explicit DetectionArea(const lanelet::RegulatoryElementDataPtr & data);
};
static lanelet::RegisterRegulatoryElement<DetectionArea> regDetectionArea;

}  // namespace lanelet::avyana

// NOLINTEND(readability-identifier-naming)

#endif  // LANELET2_EXTENSION__REGULATORY_ELEMENTS__DETECTION_AREA_HPP_
