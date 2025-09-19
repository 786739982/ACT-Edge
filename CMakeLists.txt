cmake_minimum_required(VERSION 3.16.3)

project(act_infer)
include(GNUInstallDirs)
MESSAGE(STATUS "Using toolchain file: ${CMAKE_TOOLCHAIN_FILE}")

option(BUILD_EXAMPLES "" OFF)
option(SIMULATION "" OFF)
if(SIMULATION AND CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    message(FATAL_ERROR "SIMULATION is not supported on aarch64")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE CACHE STRING "Release")
add_compile_options(-Wno-deprecated-enum-enum-conversion)

# libdnn.so depends on system software dynamic link library, use -Wl,-unresolved-symbols=ignore-in-shared-libs to shield during compilation
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,-unresolved-symbols=ignore-in-shared-libs")

set(DEPS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/deps/${CMAKE_SYSTEM_PROCESSOR})
set(HOST_PACKAGE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/${CMAKE_SYSTEM_PROCESSOR})

message(STATUS "Using SIMULATION: ${SIMULATION}")
message(STATUS "Install directory: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_C_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE " -O3 ")
set(CMAKE_C_FLAGS_RELEASE " -O3 ")

# Build Examples
if(BUILD_EXAMPLES)
    add_subdirectory(examples/00_quick_start)
    add_subdirectory(examples/01_api_tutorial)
    add_subdirectory(examples/02_advanced_samples)
    add_subdirectory(examples/03_misc)
endif()

# Find ROS 2 Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(airbot_controller_msgs REQUIRED)
find_package(fmt REQUIRED)

# Add Library
add_executable(act_infer_app
  src/act_infer_app.cpp
)

if(SIMULATION)
    set(LINK_libs dnn opencv_world dl rt pthread hbdk_sim_x86 fmt::fmt)
else()
    set(LINK_libs dnn opencv_world dl rt pthread fmt::fmt)
endif()

ament_target_dependencies(act_infer_app
    rclcpp 
    rclcpp_lifecycle
    std_msgs
    lifecycle_msgs
    sensor_msgs
    moveit_msgs
    rclcpp_components
    airbot_controller_msgs
)

target_link_libraries(act_infer_app ${LINK_libs})

# Properly install include directories
target_include_directories(act_infer_app PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${HOST_PACKAGE_DIR}/dnn/include
    ${DEPS_ROOT}/opencv/include
)

target_link_directories(act_infer_app PUBLIC
    ${HOST_PACKAGE_DIR}/dnn/lib
    ${DEPS_ROOT}/opencv/lib
)

# Install Executable
install(TARGETS act_infer_app
    DESTINATION lib/${PROJECT_NAME}
)

# rclcpp_components_register_nodes(act_infer_app "act_infer_app")

# Install Shared Libraries
install(FILES ${HOST_PACKAGE_DIR}/dnn/lib/libdnn.so DESTINATION ${CMAKE_INSTALL_LIBDIR})

if(SIMULATION)
    install(FILES ${HOST_PACKAGE_DIR}/dnn/lib/libhbdk_sim_x86.so DESTINATION ${CMAKE_INSTALL_LIBDIR})
else()
    install(FILES ${HOST_PACKAGE_DIR}/dnn/lib/libhbrt_bayes_aarch64.so DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()

install(FILES ${DEPS_ROOT}/opencv/lib/libopencv_world.so.3.4 DESTINATION ${CMAKE_INSTALL_LIBDIR})

if (EXISTS ${HOST_PACKAGE_DIR}/dnn/lib/libperfetto_sdk.so)
    message("File exists: ${HOST_PACKAGE_DIR}/dnn/lib/libperfetto_sdk.so")
    install(FILES ${HOST_PACKAGE_DIR}/dnn/lib/libperfetto_sdk.so DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()

# Ensure ROS 2 package is correctly registered
ament_package()
