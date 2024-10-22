cmake_minimum_required(VERSION 3.0.2)
project(gridmap)

ADD_COMPILE_OPTIONS(-std=c++11)
ADD_COMPILE_OPTIONS(-std=c++14)

# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS} -O3 -Wall -pthread") # -Wextra -Werror
set(CMAKE_BUILD_TYPE "RELEASE")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  message_filters
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gridmap
)

find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library(gridmap
  include/PlanMapBase.hpp
  include/InESDFMap.hpp
  include/InflationMap.hpp
  include/SOGMMap.hpp
  include/RayCast.hpp
  src/InESDFMap.cpp
  src/InflationMap.cpp
  src/SOGMMap.cpp
  src/RayCast.cpp
)

target_link_libraries(gridmap
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(sogm_node
  include/SOGMMap.hpp
  include/RayCast.hpp
  include/PlanMapBase.hpp
  src/SOGMMap.cpp
  src/RayCast.cpp
  test/sogm_node.cpp
)

target_link_libraries(sogm_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(global_gridmap_node
  include/GlobalGridMap.hpp
  src/GlobalGridMap.cpp
  test/global_gridmap_node.cpp
)

target_link_libraries(global_gridmap_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(inflation_node
  include/InflationMap.hpp
  include/SOGMMap.hpp
  include/RayCast.hpp
  include/PlanMapBase.hpp
  src/InflationMap.cpp
  src/SOGMMap.cpp
  src/RayCast.cpp
  test/inflation_node.cpp
)

target_link_libraries(inflation_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(esdf_node
  include/InESDFMap.hpp
  include/SOGMMap.hpp
  include/RayCast.hpp
  include/PlanMapBase.hpp
  src/InESDFMap.cpp
  src/SOGMMap.cpp
  src/RayCast.cpp
  test/esdf_node.cpp
)

target_link_libraries(esdf_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
