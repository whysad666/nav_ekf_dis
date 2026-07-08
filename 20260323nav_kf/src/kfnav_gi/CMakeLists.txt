cmake_minimum_required(VERSION 3.0.2)
project(kfnav_gi)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf
)

find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kfnav_gi
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs std_msgs tf
)

file(GLOB file_src "./src/*.cpp" "./src/*.cc")
add_executable(${PROJECT_NAME}_node ${file_src})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_node ${YAML_CPP_LIBRARIES})

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIRS})
include_directories(/usr/include/eigen3)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/kfnav_gi/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../public)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
