cmake_minimum_required(VERSION 3.10)
project(structure_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(APPLE)
  enable_language(OBJCXX)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig QUIET)

if(PkgConfig_FOUND)
  pkg_check_modules(LIBUSB QUIET libusb-1.0)
endif()

set(STRUCTURE_DRIVER_SOURCES
  src/structure_driver.cpp
)

if(LIBUSB_FOUND)
  list(APPEND STRUCTURE_DRIVER_SOURCES src/vendor_protocol_libusb.cpp)
else()
  list(APPEND STRUCTURE_DRIVER_SOURCES src/vendor_protocol_stub.cpp)
endif()

if(APPLE)
  list(APPEND STRUCTURE_DRIVER_SOURCES src/uvc_capture_apple.mm)
  set_source_files_properties(src/uvc_capture_apple.mm PROPERTIES COMPILE_FLAGS "-fobjc-arc")
else()
  list(APPEND STRUCTURE_DRIVER_SOURCES src/uvc_capture_stub.cpp)
endif()

add_executable(structure_driver ${STRUCTURE_DRIVER_SOURCES})
target_include_directories(
  structure_driver
  PRIVATE
    include
    ${LIBUSB_INCLUDE_DIRS}
)
target_compile_options(structure_driver PRIVATE ${LIBUSB_CFLAGS_OTHER})
ament_target_dependencies(structure_driver rclcpp sensor_msgs)
if(LIBUSB_FOUND)
  target_link_libraries(structure_driver ${LIBUSB_LDFLAGS})
endif()

if(APPLE)
  target_link_libraries(
    structure_driver
    "-framework AVFoundation"
    "-framework CoreMedia"
    "-framework CoreVideo"
    "-framework Foundation"
  )
endif()

install(TARGETS structure_driver DESTINATION lib/${PROJECT_NAME})

if(LIBUSB_FOUND)
  add_executable(vendor_burst_scan tools/vendor_burst_scan.cpp)
  target_include_directories(
    vendor_burst_scan
    PRIVATE
      ${LIBUSB_INCLUDE_DIRS}
  )
  target_compile_options(vendor_burst_scan PRIVATE ${LIBUSB_CFLAGS_OTHER})
  target_link_libraries(vendor_burst_scan ${LIBUSB_LDFLAGS})
  install(TARGETS vendor_burst_scan DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
