cmake_minimum_required(VERSION 2.8.3)
project(framework_test)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    genmsg
    framework
)

catkin_package(
    LIBRARIES
    framework
#  INCLUDE_DIRS include
#  LIBRARIES rdt-mqtt
    CATKIN_DEPENDS
    framework
#  DEPENDS system_lib
)

include_directories(
    include ${catkin_INCLUDE_DIRS}
)

add_executable(tester_send src/Emoji_Sender.cpp)
target_link_libraries(tester_send ${catkin_LIBRARIES})
add_dependencies(tester_send ${catkin_EXPORTED_TARGETS})

add_executable(tester_recv src/Emoji_Reciever.cpp)
target_link_libraries(tester_recv ${catkin_LIBRARIES})
add_dependencies(tester_recv ${catkin_EXPORTED_TARGETS})
