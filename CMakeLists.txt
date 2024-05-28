cmake_minimum_required(VERSION 3.5)
project(hbba_lite)


# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Int32Stamped.msg"
  "msg/StrategyState.msg"
  "srv/SetOnOffFilterState.srv"
  "srv/SetThrottlingFilterState.srv"
  DEPENDENCIES std_msgs
)

# C++ Library
include_directories(
  include
)

add_library(hbba_lite_cpp
  src/hbba_lite_cpp/filters/FilterState.cpp
  src/hbba_lite_cpp/utils/HbbaLiteException.cpp
  src/hbba_lite_cpp/core/Desire.cpp
  src/hbba_lite_cpp/core/DesireSet.cpp
  src/hbba_lite_cpp/core/Motivation.cpp
  src/hbba_lite_cpp/core/Strategy.cpp
  src/hbba_lite_cpp/core/RosFilterPool.cpp
  src/hbba_lite_cpp/core/Solver.cpp
  src/hbba_lite_cpp/core/GecodeSolver.cpp
  src/hbba_lite_cpp/core/StrategyStateLogger.cpp
  src/hbba_lite_cpp/core/RosStrategyStateLogger.cpp
  src/hbba_lite_cpp/core/HbbaLite.cpp
)
ament_target_dependencies(hbba_lite_cpp rclcpp)
target_link_libraries(hbba_lite_cpp
  gecodeflatzinc
  gecodedriver
  gecodegist
  gecodesearch
  gecodeminimodel
  gecodeset
  gecodefloat
  gecodeint
  gecodekernel
  gecodesupport
)
rosidl_target_interfaces(hbba_lite_cpp ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(TARGETS hbba_lite_cpp DESTINATION lib)

ament_export_dependencies(rclcpp)
ament_export_include_directories(include)
ament_export_libraries(hbba_lite_cpp)

add_library(rosbag2_generic_topic
  src/rosbag2_generic_topic/generic_publisher.cpp
  src/rosbag2_generic_topic/generic_subscription.cpp
  src/rosbag2_generic_topic/rosbag2_node.cpp
)
ament_target_dependencies(rosbag2_generic_topic rclcpp rosbag2_cpp)
target_link_libraries(rosbag2_generic_topic)
install(TARGETS rosbag2_generic_topic DESTINATION lib)

# C++ Nodes
add_executable(arbitration_node src/arbitration_node.cpp)
ament_target_dependencies(arbitration_node rclcpp std_msgs)
target_link_libraries(arbitration_node hbba_lite_cpp rosbag2_generic_topic)
install(TARGETS arbitration_node DESTINATION lib/${PROJECT_NAME})

add_executable(on_off_hbba_filter_node src/on_off_hbba_filter_node.cpp)
ament_target_dependencies(on_off_hbba_filter_node rclcpp std_msgs)
target_link_libraries(on_off_hbba_filter_node hbba_lite_cpp rosbag2_generic_topic)
rosidl_target_interfaces(on_off_hbba_filter_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(TARGETS on_off_hbba_filter_node DESTINATION lib/${PROJECT_NAME})

add_executable(throttling_hbba_filter_node src/throttling_hbba_filter_node.cpp)
ament_target_dependencies(throttling_hbba_filter_node rclcpp std_msgs)
target_link_libraries(throttling_hbba_filter_node hbba_lite_cpp rosbag2_generic_topic)
rosidl_target_interfaces(throttling_hbba_filter_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(TARGETS throttling_hbba_filter_node DESTINATION lib/${PROJECT_NAME})

# C++ Test Nodes
add_executable(test_on_off_hbba_publisher src/hbba_lite_cpp_tests/test_on_off_hbba_publisher.cpp)
ament_target_dependencies(test_on_off_hbba_publisher rclcpp std_msgs)
target_link_libraries(test_on_off_hbba_publisher hbba_lite_cpp)
rosidl_target_interfaces(test_on_off_hbba_publisher ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(TARGETS test_on_off_hbba_publisher DESTINATION lib/${PROJECT_NAME})

add_executable(test_on_off_hbba_subscriber src/hbba_lite_cpp_tests/test_on_off_hbba_subscriber.cpp)
ament_target_dependencies(test_on_off_hbba_subscriber rclcpp std_msgs)
target_link_libraries(test_on_off_hbba_subscriber hbba_lite_cpp)
rosidl_target_interfaces(test_on_off_hbba_subscriber ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(TARGETS test_on_off_hbba_subscriber DESTINATION lib/${PROJECT_NAME})

add_executable(test_throttling_hbba_publisher src/hbba_lite_cpp_tests/test_throttling_hbba_publisher.cpp)
ament_target_dependencies(test_throttling_hbba_publisher rclcpp std_msgs)
target_link_libraries(test_throttling_hbba_publisher hbba_lite_cpp)
rosidl_target_interfaces(test_throttling_hbba_publisher ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(TARGETS test_throttling_hbba_publisher DESTINATION lib/${PROJECT_NAME})

add_executable(test_throttling_hbba_subscriber src/hbba_lite_cpp_tests/test_throttling_hbba_subscriber.cpp)
ament_target_dependencies(test_throttling_hbba_subscriber rclcpp std_msgs)
target_link_libraries(test_throttling_hbba_subscriber hbba_lite_cpp)
rosidl_target_interfaces(test_throttling_hbba_subscriber ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(TARGETS test_throttling_hbba_subscriber DESTINATION lib/${PROJECT_NAME})


# Python Librairies
ament_python_install_package(${PROJECT_NAME})

# Python Nodes
install(PROGRAMS
  scripts/dummy_synchronized_publishers.py
  scripts/test_arbitration_publisher.py
  scripts/test_on_off_hbba_approximate_time_synchronizer.py
  scripts/test_on_off_hbba_publisher.py
  scripts/test_on_off_hbba_subscriber.py
  scripts/test_on_off_hbba_time_synchronizer.py
  scripts/test_throttling_hbba_approximate_time_synchronizer.py
  scripts/test_throttling_hbba_publisher.py
  scripts/test_throttling_hbba_subscriber.py
  scripts/test_throttling_hbba_time_synchronizer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test
    test/hbba_lite_cpp/core/DesireTests.cpp
    test/hbba_lite_cpp/core/DesireSetTests.cpp
    test/hbba_lite_cpp/core/StrategyTests.cpp
    test/hbba_lite_cpp/core/SolverTests.cpp
    test/hbba_lite_cpp/core/GecodeSolverTests.cpp
    test/hbba_lite_cpp/core/HbbaLiteTests.cpp
    test/hbba_lite_cpp/main.cpp
  )
  ament_target_dependencies(${PROJECT_NAME}_test
    rclcpp
    std_msgs
  )
  target_link_libraries(${PROJECT_NAME}_test hbba_lite_cpp)
endif()


ament_package()
