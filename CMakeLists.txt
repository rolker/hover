cmake_minimum_required(VERSION 2.8.3)
project(hover)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geographic_msgs
  roscpp
  genmsg
  actionlib_msgs
  actionlib
  project11
  project11_navigation
  geographic_visualization_msgs
  dynamic_reconfigure
  tf2_ros
)

generate_dynamic_reconfigure_options(
    cfg/hover.cfg
)


add_action_files(DIRECTORY action FILES hover.action)
generate_messages(DEPENDENCIES actionlib_msgs geographic_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hover_plugin
  CATKIN_DEPENDS geographic_msgs geometry_msgs roscpp project11 project11_navigation tf2_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/hover.cpp src/hover_action.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)

add_library(hover_plugin src/hover_plugin.cpp src/hover.cpp)
add_dependencies(hover_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hover_plugin ${catkin_LIBRARIES})


install(TARGETS ${PROJECT_NAME}_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES hover_plugin.xml
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
