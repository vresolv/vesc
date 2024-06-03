cmake_minimum_required(VERSION 3.8)
project(vesc_motor_controller)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(Qt5 COMPONENTS Network REQUIRED)

file(GLOB vesc_driver_sources "vesc_driver/src/*.cpp")

add_executable(${PROJECT_NAME}_node
    ${vesc_driver_sources}
)

include_directories(vesc_driver/include)

target_link_libraries(
    ${PROJECT_NAME}_node
    DDSCommon
    fastrtps
    fastcdr
    Qt5::Network
)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)