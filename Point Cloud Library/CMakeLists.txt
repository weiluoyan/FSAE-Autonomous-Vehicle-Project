#Edit the  CMakeLists.txt file of our package
# Find and Include PCL
find_package(PCL REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#Link PCL Libraries
add_executable(point_cloud_processor src/point_cloud_processor.cpp)
target_link_libraries(point_cloud_processor
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#Ensure ros Packages are found
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_conversions
  pcl_ros
)
