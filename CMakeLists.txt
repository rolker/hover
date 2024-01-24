cmake_minimum_required(VERSION 2.8.3)
project(hover)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  behaviortree_cpp
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
  LIBRARIES hover_plugin hover_bt_plugin
  CATKIN_DEPENDS behaviortree_cpp geographic_msgs geometry_msgs roscpp project11 project11_navigation tf2_ros
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

# add_library(hover_plugin src/hover_plugin.cpp src/hover.cpp)
# add_dependencies(hover_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(hover_plugin ${catkin_LIBRARIES})


add_library(hover_bt_plugin SHARED src/hover_bt.cpp )
target_link_libraries(hover_bt_plugin PRIVATE ${BTCPP_LIBRARY})
target_compile_definitions(hover_bt_plugin PRIVATE  BT_PLUGIN_EXPORT )
#set_target_properties(hover_bt_plugin PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${BTCPP_BIN_DESTINATION} )



install(TARGETS ${PROJECT_NAME}_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES hover_plugin.xml
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
