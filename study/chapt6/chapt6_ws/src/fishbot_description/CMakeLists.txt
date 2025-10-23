cmake_minimum_required(VERSION 3.8)
project(fishbot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY launch config urdf worlds scripts
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/laser_frame_fix.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
