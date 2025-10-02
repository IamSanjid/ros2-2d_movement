cmake_minimum_required(VERSION 3.8)
project(controller)

set(CMAKE_CXX_STANDARD 20)
set(BIN_NAME "main")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)

find_package(interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

function(install_node)
  set(oneValueArgs NAME)
  set(options)
  set(multiValueArgs DEPENDENCIES)

  cmake_parse_arguments(INSTALL_NODE
    "${options}"
    "${oneValueArgs}"
    "${multiValueArgs}"
    ${ARGN})
  
  if(DEFINED INSTALL_NODE_NAME)
    file(GLOB_RECURSE SRCS
      "src/${INSTALL_NODE_NAME}/*.cpp"
      "src/${INSTALL_NODE_NAME}/**/*.cpp"
      "src/${INSTALL_NODE_NAME}/*.hpp"
      "src/${INSTALL_NODE_NAME}/**/*.hpp"
      "src/${INSTALL_NODE_NAME}/*.c"
      "src/${INSTALL_NODE_NAME}/**/*.c"
      "${INSTALL_NODE_NAME}/*.cpp"
      "${INSTALL_NODE_NAME}/**/*.cpp"
      "${INSTALL_NODE_NAME}/*.hpp"
      "${INSTALL_NODE_NAME}/**/*.hpp"
      "${INSTALL_NODE_NAME}/*.c"
      "${INSTALL_NODE_NAME}/**/*.c")

    add_executable(${INSTALL_NODE_NAME} ${SRCS})
    target_include_directories(${INSTALL_NODE_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/${INSTALL_NODE_NAME}")
    target_include_directories(${INSTALL_NODE_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}")

    ament_target_dependencies(${INSTALL_NODE_NAME} ${INSTALL_NODE_DEPENDENCIES})

    install(TARGETS
      ${INSTALL_NODE_NAME}
      DESTINATION lib/${PROJECT_NAME})
  else()
    file(GLOB_RECURSE SRCS
      "src/*.cpp"
      "src/**/*.cpp"
      "src/*.hpp"
      "src/**/*.hpp"
      "src/*.c"
      "src/**/*.c")

    add_executable(${BIN_NAME} ${SRCS})
    target_include_directories(${BIN_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}")

    ament_target_dependencies(${BIN_NAME} ${INSTALL_NODE_DEPENDENCIES})
    
    install(TARGETS
      ${BIN_NAME}
      DESTINATION lib/${PROJECT_NAME})
  endif()
endfunction()

install_node(NAME "camera" DEPENDENCIES rclcpp rclcpp_action std_msgs interfaces)
install_node(NAME "body" DEPENDENCIES rclcpp rclcpp_action std_msgs interfaces)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch)

# ament_export_include_directories(include)
ament_package()
