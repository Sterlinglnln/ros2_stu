cmake_minimum_required(VERSION 3.8)
project(demo_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
include_directories(include)

add_executable(cpp_node src/cppNode.cpp)
add_executable(person_node src/personNode.cpp)
add_executable(learn_auto src/learnAuto.cpp)
add_executable(learn_shared_ptr src/learnSharedPtr.cpp)
add_executable(learn_function src/learnFunction.cpp)
add_executable(learn_thread src/learnThread.cpp)

ament_target_dependencies(cpp_node rclcpp)
ament_target_dependencies(person_node rclcpp)

install(TARGETS
        cpp_node
        person_node
        learn_auto
        learn_shared_ptr
        learn_function
        learn_thread
        DESTINATION lib/${PROJECT_NAME}
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
