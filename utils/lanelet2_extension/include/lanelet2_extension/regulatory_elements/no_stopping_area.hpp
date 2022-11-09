#ifndef LANELET2_EXTENSION__REGULATORY_ELEMENTS__NO_STOPPING_AREA_HPP_
#define LANELET2_EXTENSION__REGULATORY_ELEMENTS__NO_STOPPING_AREA_HPP_

// NOLINTBEGIN(readability-identifier-naming)

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <memory>
#include <vector>

namespace lanelet::avyana
{
class NoStoppingArea : public lanelet::RegulatoryElement
{
public:
  using Ptr = std::shared_ptr<NoStoppingArea>;
  static constexpr char RuleName[] = "no_stopping_area";

  //! Directly construct a stop line from its required rule parameters.
  //! Might modify the input data in oder to get correct tags.
  static Ptr make(
    Id id, const AttributeMap & attributes, const Polygons3d & no_stopping_areas,
    const Optional<LineString3d> & stopLine = {})
  {
    return Ptr{new NoStoppingArea(id, attributes, no_stopping_areas, stopLine)};
  }

  /**
   * @brief get the relevant no stopping area
   * @return no stopping area
   */
  [[nodiscard]] ConstPolygons3d noStoppingAreas() const;
  [[nodiscard]] Polygons3d noStoppingAreas();

  /**
   * @brief add a new no stopping area
   * @param primitive no stopping area to add
   */
  void addNoStoppingArea(const Polygon3d & primitive);

  /**
   * @brief remove a no stopping area
   * @param primitive the primitive
   * @return true if the no stopping area existed and was removed
   */
  bool removeNoStoppingArea(const Polygon3d & primitive);

  /**
   * @brief get the stop line for the no stopping area
   * @return the stop line as LineString
   */
  [[nodiscard]] Optional<ConstLineString3d> stopLine() const;
  [[nodiscard]] Optional<LineString3d> stopLine();

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
  friend class lanelet::RegisterRegulatoryElement<NoStoppingArea>;
  NoStoppingArea(
    Id id, const AttributeMap & attributes, const Polygons3d & no_stopping_area,
    const Optional<LineString3d> & stopLine);
  explicit NoStoppingArea(const lanelet::RegulatoryElementDataPtr & data);
};
static lanelet::RegisterRegulatoryElement<NoStoppingArea> regNoStoppingArea;

}  // namespace lanelet::avyana

// NOLINTEND(readability-identifier-naming)

#endif  // LANELET2_EXTENSION__REGULATORY_ELEMENTS__NO_STOPPING_AREA_HPP_
