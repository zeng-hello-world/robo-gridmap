cmake_minimum_required(VERSION 2.8.3)
project(grid_map)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        pcl_ros
)

catkin_package(

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(grid_map
        src/grid_map.cpp
        src/grid_map.h
        )
target_link_libraries(grid_map ${catkin_LIBRARIES} ${PCL_LIBRARIES})


#add_executable(gp_insac src/grid_map.cpp)
#target_link_libraries(gp_insac ${catkin_LIBRARIES})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES} grid_map)

#add_executable(listener src/listener.cpp)
#target_link_libraries(listener ${catkin_LIBRARIES} grid_map)
