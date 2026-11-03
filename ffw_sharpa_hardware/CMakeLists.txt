cmake_minimum_required(VERSION 3.16)
project(ffw_sharpa_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# The SharpaWave SDK is proprietary and not redistributed: scripts/setup_sharpa.sh downloads the
# pinned release into <repo>/third_party/sharpa-wave-sdk. Set SHARPA_WAVE_SDK_ROOT (CMake or
# environment) to use another copy, e.g. /opt/sharpa-wave-sdk from the vendor's .deb.
set(SHARPA_WAVE_SDK_ROOT "$ENV{SHARPA_WAVE_SDK_ROOT}" CACHE PATH "SharpaWave SDK root")
set(_sharpa_sdk_hints
  ${SHARPA_WAVE_SDK_ROOT}
  ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/sharpa-wave-sdk
  /opt/sharpa-wave-sdk)
find_path(SHARPA_WAVE_SDK_INCLUDE_DIR SharpaWaveSDK.h
  HINTS ${_sharpa_sdk_hints} PATH_SUFFIXES include NO_DEFAULT_PATH)
find_library(SHARPA_WAVE_SDK_LIBRARY sharpa-wave-sdk
  HINTS ${_sharpa_sdk_hints} PATH_SUFFIXES lib NO_DEFAULT_PATH)

if(NOT SHARPA_WAVE_SDK_INCLUDE_DIR OR NOT SHARPA_WAVE_SDK_LIBRARY)
  # Keep the workspace buildable without the SDK (CI, simulation, mock hardware): the plugin is
  # simply not built, so loading it fails with "class ... does not exist".
  message(WARNING
    "SharpaWave SDK not found (searched: ${_sharpa_sdk_hints}); "
    "skipping the ffw_sharpa_hardware plugin. Run ffw_sharpa_hardware/scripts/setup_sharpa.sh.")
  ament_package()
  return()
endif()
message(STATUS "SharpaWave SDK: ${SHARPA_WAVE_SDK_LIBRARY}")

set(THIS_PACKAGE_INCLUDE_DEPENDS
  diagnostic_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED src/sharpa_wave_system.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${SHARPA_WAVE_SDK_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} PRIVATE ${SHARPA_WAVE_SDK_LIBRARY})
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# The SDK library is installed next to the plugin and found through $ORIGIN.
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "$ORIGIN")

pluginlib_export_plugin_description_file(hardware_interface ffw_sharpa_hardware.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(FILES ${SHARPA_WAVE_SDK_LIBRARY} DESTINATION lib)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
