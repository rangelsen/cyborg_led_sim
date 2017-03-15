cmake_minimum_required(VERSION 2.8.3)
project(cyborg_led_sim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

#Generate added messages and services with any dependencies listed here
# generate_messages(DEPENDENCIES)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

MESSAGE(${catkin_INCLUDE_DIRS})

add_executable(led_sim src/main.cpp src/LED.cpp src/Display.cpp src/LEDGrid.cpp)

target_link_libraries(led_sim ${catkin_LIBRARIES} GL GLEW SDL2)

