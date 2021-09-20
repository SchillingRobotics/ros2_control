cmake_minimum_required(VERSION 3.5)
project(limit_enforcement_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(ruckig REQUIRED)

add_library(joint_limiter_interface SHARED src/joint_limiter_interface.cpp)
target_include_directories(
  joint_limiter_interface
  PRIVATE
  include)
ament_target_dependencies(
  joint_limiter_interface
  rclcpp
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(joint_limiter_interface PRIVATE "CONTROLLER_INTERFACE_BUILDING_DLL")

add_library(
  simple_joint_limiter
  SHARED
  src/simple_joint_limiter.cpp
)
target_include_directories(
  simple_joint_limiter
  PRIVATE
  include
)
target_link_libraries(
  simple_joint_limiter
  joint_limiter_interface
)
ament_target_dependencies(
  simple_joint_limiter
  rclcpp
  rcutils
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(simple_joint_limiter PRIVATE "CONTROLLER_INTERFACE_BUILDING_DLL")

add_library(
  ruckig_joint_limiter
  SHARED
  src/ruckig_joint_limiter.cpp
)
target_include_directories(
  ruckig_joint_limiter
  PRIVATE
  include
)
target_link_libraries(
  ruckig_joint_limiter
  joint_limiter_interface
)
ament_target_dependencies(
  ruckig_joint_limiter
  rclcpp
  rcutils
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(ruckig_joint_limiter PRIVATE "CONTROLLER_INTERFACE_BUILDING_DLL")

pluginlib_export_plugin_description_file(limit_enforcement_plugins joint_limiters.xml)

install(DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    joint_limiter_interface
    ruckig_joint_limiter
    simple_joint_limiter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(rclcpp REQUIRED)

  #ament_add_gmock(limit_enforcement_plugins_test test/limit_enforcement_plugins_test.cpp)
  #target_include_directories(limit_enforcement_plugins_test PUBLIC include)
  #target_link_libraries(limit_enforcement_plugins_test limit_enforcement_plugins)

  ament_add_gmock(test_simple_joint_limiter test/test_simple_joint_limiter.cpp)
  target_include_directories(test_simple_joint_limiter PRIVATE include)
  target_link_libraries(test_simple_joint_limiter joint_limiter_interface)
  ament_target_dependencies(
    test_simple_joint_limiter
    pluginlib
    rclcpp
  )

  ament_add_gmock(test_ruckig_joint_limiter test/test_ruckig_joint_limiter.cpp)
  target_include_directories(test_ruckig_joint_limiter PRIVATE include)
  target_link_libraries(test_ruckig_joint_limiter joint_limiter_interface)
  ament_target_dependencies(
    test_ruckig_joint_limiter
    pluginlib
    rclcpp
  )
endif()

ament_export_dependencies(
  rclcpp
  rcutils
  ruckig
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  joint_limiter_interface
  ruckig_joint_limiter
  simple_joint_limiter
)

ament_package()
