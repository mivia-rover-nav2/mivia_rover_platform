cmake_minimum_required(VERSION 3.16)
project(mivia_rover_platform LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  mivia_rover_can_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/mivia_rover_system.cpp
  src/rover.c
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export plugin description to pluginlib/resource_manager
pluginlib_export_plugin_description_file(hardware_interface mivia_rover_platform.xml)

# ----------------------------
# INSTALL
# ----------------------------

# Headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Resources
install(
  DIRECTORY description/
  DESTINATION share/${PROJECT_NAME}/description
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Plugin XML
install(
  FILES mivia_rover_platform.xml
  DESTINATION share/${PROJECT_NAME}
)

# Library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ----------------------------
# TESTING (opzionale ma raccomandato)
# ----------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(rclcpp REQUIRED)

  # TODO: aggiungere test (es. test_load_mivia_rover_system.cpp) con ament_add_gmock
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(include)

ament_package()
