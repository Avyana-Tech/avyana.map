cmake_minimum_required(VERSION 3.8)
project(lanelet2_extesnion)

find_package(avyana_cmake REQUIRED)
avyana_package()

find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})

find_library(GeographicLib_LIBRARIES
  NAMES Geographic
)

find_library(PUGIXML_LIBRARIES
  NAMES pugixml
)

find_path(PUGIXML_INCLUDE_DIRS
  NAMES pugixml.hpp
  PATH_SUFFIXES pugixml
)

include_directories(
  ${GeographicLib_INCLUDE_DIRS}
  ${PUGIXML_INCLUDE_DIRS}
)

add_definitions(${GeographicLib_DEFINITIONS})

ament_auto_add_library(lanelet2_extension_lib SHARED
  lib/osm_parser.cpp
)
target_link_libraries(lanelet2_extension_lib
  ${GeographicLib_LIBRARIES}
)

ament_auto_add_executable(lanelet2_extension_sample src/sample.cpp)
add_dependencies(lanelet2_extension_sample lanelet2_extension_lib)
target_link_libraries(lanelet2_extension_sample
  lanelet2_extension_lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
