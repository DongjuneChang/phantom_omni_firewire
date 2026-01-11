cmake_minimum_required(VERSION 3.8)
project(omni_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(omni_msgs REQUIRED)

# OpenHaptics SDK
set(OPENHAPTICS_INCLUDE_DIR "/opt/OpenHaptics/Developer/3.4-0/include")
set(OPENHAPTICS_LIB_DIR "/opt/OpenHaptics/Developer/3.4-0/lib/lin-x86-64")

# Check if OpenHaptics is available
if(EXISTS ${OPENHAPTICS_INCLUDE_DIR})
  message(STATUS "OpenHaptics SDK found at ${OPENHAPTICS_INCLUDE_DIR}")
else()
  message(WARNING "OpenHaptics SDK not found. Building without hardware support.")
endif()

# omni_state executable
add_executable(omni_state src/omni_state.cpp)

target_include_directories(omni_state PRIVATE
  ${OPENHAPTICS_INCLUDE_DIR}
)

target_link_directories(omni_state PRIVATE
  ${OPENHAPTICS_LIB_DIR}
  /usr/lib
)

ament_target_dependencies(omni_state
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  omni_msgs
)

target_link_libraries(omni_state
  HD
  HDU
  rt
  ncurses
  pthread
)

# Install
install(TARGETS omni_state
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

