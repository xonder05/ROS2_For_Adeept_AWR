cmake_minimum_required(VERSION 3.16)
project(adeept_awr_diffdrive_control_plugin LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  adeept_awr_diffdrive_control_plugin
  SHARED
  hardware/diffbot_system.cpp
)
target_compile_features(adeept_awr_diffdrive_control_plugin PUBLIC cxx_std_17)
target_include_directories(adeept_awr_diffdrive_control_plugin PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/adeept_awr_diffdrive_control_plugin>
)
ament_target_dependencies(
  adeept_awr_diffdrive_control_plugin PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ADEEPT_AWR_DIFFDRIVE_CONTROL_PLUGIN_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface adeept_awr_diffdrive_control_plugin.xml)

target_link_libraries(adeept_awr_diffdrive_control_plugin
  PRIVATE -lgpiod
)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/adeept_awr_diffdrive_control_plugin
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/adeept_awr_diffdrive_control_plugin
)
install(TARGETS adeept_awr_diffdrive_control_plugin
  EXPORT export_adeept_awr_diffdrive_control_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_adeept_awr_diffdrive_control_plugin HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
