cmake_minimum_required(VERSION 3.8)
project(spot_bt_ros_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 17)
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
ament_package()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
