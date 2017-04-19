cmake_minimum_required(VERSION 2.8.3)
project(cyborg_led_sim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  genmsg
)

add_message_files(FILES rgba.msg)
generate_messages(DEPENDENCIES)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(led_sim src/main.cpp src/LED.cpp src/Display.cpp src/LEDGrid.cpp)
add_executable(publisher src/dummy_publisher.cpp)

target_link_libraries(led_sim ${catkin_LIBRARIES} GL GLEW SDL2 opencv_core opencv_imgproc)
target_link_libraries(publisher ${catkin_LIBRARIES})

