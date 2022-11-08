/**
 * COPYRIGHT 2022 AVYANA. ALL RIGHTS RESERVED.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * 
 * Based on Autoware Common lanelet2_extension
 * 
*/

#include <lanelet2_extesnion/io/osm_parser.hpp>

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/OsmFile.h>
#include <lanelet2_io/io_handlers/OsmHandler.h>

#include <memory>
#include <string>

namespace lanelet::io_handlers {
    std::unique_ptr<LaneletMap> AvyanaOsmParser::parse(const std::string & filename, ErrorMessages & errors ) const {
        auto map = OsmParser::parse(filename, errors);
        // overwrite x & y values if there are local_x & local_y tags
        for (Point3d point : map->pointLayer) {
            if (point.hasAttribute("local_x")) {
                point.x() = point.attribute("local_x").asDouble().value();
            }
            if (point.hasAttribute("local_y")) {
                point.x() = point.attribute("local_y").asDouble().value();
            }
        }

        // return align function, just in case
        for (Lanelet & lanelet : map->laneletLayer) {
            LineString3d new_left;
            LineString3d new_right;
            std::tie(new_left, new_right) = geometry::align(lanelet.leftBound(), lanelet.rightBound());
            lanelet.setLeftBound(new_left);
            lanelet.setRightBound(new_right);
        }
        return map;
    }

    namespace { RegisterParser <AvyanaOsmParser> regParser; }

    void AvyanaOsmParser::parseVersions( const std::string & filename, std::string * format_version, std::string * map_version ) {
        if (format_version == nullptr || map_version == nullptr) {
            std::cerr << __FUNCTION__ << ": either format_version or map_version is null pointer!";
            return;
        }

        pugi::xml_document doc;
        auto result = doc.load_file(filename.c_str());
        if (!result) {
            throw lanelet::ParseError(
            std::string("Errors occurred while parsing osm file: ") + result.description());
        }
        
        auto osmNode = doc.child("osm");
        auto metainfo = osmNode.child("MetaInfo");
        if (metainfo.attribute("format_version")) {
            *format_version = metainfo.attribute("format_version").value();
        }
        if (metainfo.attribute("map_version")) {
            *map_version = metainfo.attribute("map_version").value();
        }
    }
}