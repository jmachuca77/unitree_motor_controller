cmake_minimum_required(VERSION 3.8)
project(unitree_motor_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64") 
    set(EXTRA_LIBS UnitreeMotorSDK_Arm64)
else()
    set(EXTRA_LIBS UnitreeMotorSDK_Linux64)
endif()

# Check if the library file exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib/lib${EXTRA_LIBS}.so")
    message(STATUS "Found library: ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib${EXTRA_LIBS}.so")
else()
    message(FATAL_ERROR "Library not found: ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib${EXTRA_LIBS}.so")
endif()

# Add executable if not already defined
if(NOT TARGET motor_controller)
    add_executable(motor_controller src/motor_controller.cpp)
endif()

# Link the Unitree Motor library
target_link_libraries(motor_controller ${EXTRA_LIBS})

ament_target_dependencies(motor_controller rclcpp sensor_msgs std_msgs)

# Install the executable
install(TARGETS
  motor_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install the shared library
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib${EXTRA_LIBS}.so
  DESTINATION lib
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()