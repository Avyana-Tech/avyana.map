cmake_minimum_required(VERSION 3.8)
project(map_tf_generator)

find_package(avyana_cmake REQUIRED)
avyana_package()

find_package(PCL REQUIRED)

include_directories(
  include
  SYSTEM
  ${PCL_INCLUDE_DIRS}
)

ament_auto_add_library(pcd_map_tf_generator_node SHARED
  src/pcd_map_tf_generator_node.cpp
)
target_link_libraries(pcd_map_tf_generator_node ${PCL_LIBRARIES})

rclcpp_components_register_node(pcd_map_tf_generator_node
  PLUGIN "PcdMapTFGeneratorNode"
  EXECUTABLE pcd_map_tf_generator
)

ament_auto_add_library(vector_map_tf_generator_node SHARED
  src/vector_map_tf_generator_node.cpp
)

rclcpp_components_register_node(vector_map_tf_generator_node
  PLUGIN "VectorMapTFGeneratorNode"
  EXECUTABLE vector_map_tf_generator
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
