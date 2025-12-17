cmake_minimum_required(VERSION 3.8)
project(asr_sdm_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(asr_sdm_control_msgs REQUIRED)
find_package(asr_sdm_hardware_msgs REQUIRED)
# find_package(periphery REQUIRED)  # Optional dependency, commented out for now
# Find libgpiod
# find_package(PkgConfig REQUIRED)
# pkg_check_modules(GPIOD REQUIRED libgpiod)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# find_library(MRAA_LIB NAMES mraa PATHS /usr/local/include)
# find_library(PERIPHERY_LIB NAMES periphery PATHS /usr/local/include)

include_directories(
  include
  # ${GPIOD_INCLUDE_DIRS}
  /usr/local/include
)

# add_executable(mcp_can src/mcp_can.cpp)
# target_link_libraries(mcp_can ${MRAA_LIB})
# ament_target_dependencies(mcp_can rclcpp)

# Build controller library
add_library(front_unit_following_controller
  src/front_unit_following_controller.cpp
)
target_include_directories(front_unit_following_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Build main executable
add_executable(asr_sdm_controller src/asr_sdm_controller_node.cpp)
ament_target_dependencies(asr_sdm_controller
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  asr_sdm_control_msgs
  asr_sdm_hardware_msgs
)
target_link_libraries(asr_sdm_controller
  front_unit_following_controller
)

install(TARGETS
  asr_sdm_controller
  front_unit_following_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  
  # Add unit tests
  ament_add_gtest(test_front_unit_following_controller
    test_front_unit_following_controller.cpp
  )
  if(TARGET test_front_unit_following_controller)
    target_include_directories(test_front_unit_following_controller PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
    )
    target_link_libraries(test_front_unit_following_controller
      front_unit_following_controller
    )
  endif()
endif()

ament_package()
