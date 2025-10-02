cmake_minimum_required(VERSION 3.8)
project(interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

file(GLOB_RECURSE INTERFACE_FILES
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "msg/*.msg"
  "msg/**/*.msg"
  "srv/*.srv"
  "srv/**/*.srv"
  "action/*.action"
  "action/**/*.action")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${INTERFACE_FILES}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
