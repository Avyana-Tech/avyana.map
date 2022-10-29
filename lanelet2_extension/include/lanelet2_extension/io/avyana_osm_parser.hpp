#ifndef LANELET2_EXTENSION__IO__AVYANA_OSM_PARSER_HPP_
#define LANELET2_EXTENSION__IO__AVYANA_OSM_PARSER_HPP_

#include <lanelet2_io/io_handlers/OsmHandler.h>

#include <memory>
#include <string>

namespace lanelet::io_handlers {
    class AvyanaOsmParser : public OsmParser {
    public:
        using OsmParser::OsmParser;

        /**
         * [parse parse osm file to laneletMap. It is generally same as default
         * OsmParser, but it will overwrite x and y value with local_x and local_y
         * tags if present]
         * @param  filename [path to osm file]
         * @param  errors   [any errors caught during parsing]
         * @return          [returns LaneletMap]
         */
        std::unique_ptr<LaneletMap> parse(
            const std::string & filename, ErrorMessages & errors) const override;

        /**
         * [parseVersions parses MetaInfo tags from osm file]
         * @param filename       [path to osm file]
         * @param format_version [parsed information about map format version]
         * @param map_version    [parsed information about map version]
         */
        static void parseVersions(
            const std::string & filename, std::string * format_version, std::string * map_version);

        static constexpr const char * extension() { return ".osm"; }

        static constexpr const char * name() { return "avyana_osm_handler"; }
    };
}

#endif  // LANELET2_EXTENSION__IO__AVYANA_OSM_PARSER_HPP_
