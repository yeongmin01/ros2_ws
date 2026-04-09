cmake_minimum_required(VERSION 3.8)
project(j1939_parser)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_msgs REQUIRED)

include_directories(include)

add_library(j1939_parser src/j1939_parser.cpp)
ament_target_dependencies(j1939_parser rclcpp custom_msgs) # 의존성 연결

add_executable(j1939_node src/j1939_node.cpp) # 새로운 실행 파일
ament_target_dependencies(j1939_node rclcpp custom_msgs) # 의존성 연결

#ament_export_include_directories(include)
#ament_export_libraries(j1939_parser)
#ament_export_dependencies(rclcpp custom_msgs)

install(TARGETS j1939_node DESTINATION lib/${PROJECT_NAME})

install(TARGETS j1939_parser DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include
)

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
