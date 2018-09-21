add_library(footbot_ros SHARED footbot_ros.h footbot_ros.cpp)
target_link_libraries(footbot_ros
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  /opt/ros/$ENV{ROS_DISTRO}/lib/libroscpp.so)
