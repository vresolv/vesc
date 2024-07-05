cmake_minimum_required(VERSION 3.8)
project(vesc_motor_controller)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(asio CONFIG REQUIRED)

add_library(${PROJECT_NAME} STATIC
    vesc_driver/src/vesc_device_namer.cpp
    vesc_driver/src/vesc_device_uuid_lookup.cpp
    vesc_driver/src/vesc_driver.cpp
    vesc_driver/src/vesc_interface.cpp
    vesc_driver/src/vesc_packet_factory.cpp
    vesc_driver/src/vesc_packet.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC vesc_driver/include)

target_link_libraries(${PROJECT_NAME}
    DDSCommon
    asio::asio
    io_context
    serial_driver
    fastrtps
    fastcdr
)

add_executable(${PROJECT_NAME}_node
    vesc_driver/src/vesc_driver_node.cpp
)


target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
