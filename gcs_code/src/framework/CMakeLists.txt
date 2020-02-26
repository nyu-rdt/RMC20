cmake_minimum_required(VERSION 2.8.3)
project(framework)

find_package(catkin REQUIRED
    COMPONENTS
    message_generation
    roscpp
    std_msgs
    genmsg
)

find_package(SDL2 REQUIRED)
string(STRIP ${SDL2_LIBRARIES} SDL2_LIBRARIES)
find_package(PkgConfig)

add_service_files(FILES
    IntStr.srv
    StrInt.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${SDL2_INCLUDE_DIRS}
)

add_executable(framework_node
    src/main.cpp
    src/JumpTable.cpp
    src/Manager.cpp
    src/ROSTable.cpp
    src/MoveFunction.cpp
)

target_link_libraries(framework_node
    ${catkin_LIBRARIES}
    ${SDL2_LIBRARIES}
)

