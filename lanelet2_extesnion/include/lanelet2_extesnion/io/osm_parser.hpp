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

#ifndef LANELET2_EXTENSION__IO__OSM_PARSER_HPP_
#define LANELET2_EXTENSION__IO__OSM_PARSER_HPP_

#include <lanelet2_io/io_handlers/OsmHandler.h>

#include <memory>
#include <string>

namespace lanelet::io_handlers {
    class AvyanaOsmParser : public OsmParser {
        public : using OsmParser::OsmParser;

        /**
         * 
         * [parse parse osm file to laneletMap. It is generally same as default
         * OsmParser, but it will overwrite x and y value with local_x and local_y
         * tags if present]
         * 
         * @param filename [path to OSM file]
         * @param errors [any error caught during parsing]
         * @return
         * 
        */
        std::unique_ptr<LaneletMap> parse(const std::string & filename, ErrorMessages & errors) const override;

        /**
         * 
         * [parseVersions parses MetaInfo tags from OSM file]
         * @param filename [path to osm file]
         * @param format_version [parsed information about map format version]
         * @param map_version    [parsed information about map version]
         * 
        */
        static void parseVersions(const std::string & filename, std::string * format_version, std::string * map_version);

        static constexpr const char * extension() { return ".osm"; }
        static constexpr const char * name() { return "avyana_osm_handler"; }
    };
}

#endif