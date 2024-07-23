cmake_minimum_required(VERSION 3.8)
project(qorvo_mdek1001_driver)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)

# Find dependencies
find_library(SERIAL_LIBRARY serial REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(dua_interfaces REQUIRED)
find_package(dua_app_management REQUIRED)
find_package(dua_node REQUIRED)
find_package(dua_qos_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(params_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Threads REQUIRED)

# Generate parameters source code
generate_init_parameters(
  YAML_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/qorvo_mdek1001_driver/params.yaml"
  OUT_FILE "init_parameters.cpp")

# Configure Qorvo MDEK1001 Driver node source files
file(GLOB QORVO_MDEK1001_DRIVER_SRCS
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "src/qorvo_mdek1001_driver/*.cpp")
list(APPEND QORVO_MDEK1001_DRIVER_SRCS init_parameters.cpp)

# Qorvo MDEK1001 Driver
add_library(qorvo_mdek1001_driver SHARED ${QORVO_MDEK1001_DRIVER_SRCS})
target_compile_definitions(qorvo_mdek1001_driver PRIVATE COMPOSITION_BUILDING_DLL)
target_include_directories(qorvo_mdek1001_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(qorvo_mdek1001_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(qorvo_mdek1001_driver
  serial
  Threads::Threads)
ament_target_dependencies(qorvo_mdek1001_driver
  dua_interfaces
  dua_node
  dua_qos_cpp
  geometry_msgs
  params_manager
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs)
rclcpp_components_register_nodes(qorvo_mdek1001_driver "qorvo_mdek1001_driver::QorvoMDEK1001Driver")

# Qorvo MDEK1001 Driver standalone application
add_executable(qorvo_mdek1001_driver_app
  src/qorvo_mdek1001_driver_app.cpp)
target_include_directories(qorvo_mdek1001_driver_app PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(qorvo_mdek1001_driver_app PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(qorvo_mdek1001_driver_app qorvo_mdek1001_driver)
ament_target_dependencies(qorvo_mdek1001_driver_app
  dua_app_management
  rclcpp)

# Install Qorvo MDEK1001 Driver component
install(TARGETS qorvo_mdek1001_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install Qorvo MDEK1001 Driver app
install(TARGETS qorvo_mdek1001_driver_app
  DESTINATION lib/${PROJECT_NAME})

# Install Config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

# Install Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
