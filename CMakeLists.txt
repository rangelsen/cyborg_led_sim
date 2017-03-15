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

add_executable(main src/main.cpp src/LED.cpp src/Display.cpp src/LEDGrid.cpp)
# add_executable(Display src/Display.cpp)
# add_executable(LED src/LED.cpp)
# add_executable(LEDGrid src/LEDGrid.cpp src/LED.cpp)

target_link_libraries(main ${catkin_LIBRARIES} GL GLEW SDL2)
# target_link_libraries(Display ${catkin_LIBRARIES} GL GLEW SDL2)
# target_link_libraries(LED ${catkin_LIBRARIES} GL GLEW SDL2)
# target_link_libraries(LEDGrid ${catkin_LIBRARIES} GL GLEW SDL2)

