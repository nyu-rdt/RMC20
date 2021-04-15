# Install script for directory: /home/rdt/Desktop/RMC20-nucnodes/nuc_code/src/rdt_localization

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/rdt/Desktop/RMC20-nucnodes/nuc_code/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rdt_localization/msg" TYPE FILE FILES
    "/home/rdt/Desktop/RMC20-nucnodes/nuc_code/src/rdt_localization/msg/Pose.msg"
    "/home/rdt/Desktop/RMC20-nucnodes/nuc_code/src/rdt_localization/msg/Location.msg"
    "/home/rdt/Desktop/RMC20-nucnodes/nuc_code/src/rdt_localization/msg/Orientation_Vector.msg"
    "/home/rdt/Desktop/RMC20-nucnodes/nuc_code/src/rdt_localization/msg/Drive_Vector.msg"
    "/home/rdt/Desktop/RMC20-nucnodes/nuc_code/src/rdt_localization/msg/Obstacle.msg"
    "/home/rdt/Desktop/RMC20-nucnodes/nuc_code/src/rdt_localization/msg/Limb_Vector.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rdt_localization/cmake" TYPE FILE FILES "/home/rdt/Desktop/RMC20-nucnodes/nuc_code/build/rdt_localization/catkin_generated/installspace/rdt_localization-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/rdt/Desktop/RMC20-nucnodes/nuc_code/devel/include/rdt_localization")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/rdt/Desktop/RMC20-nucnodes/nuc_code/devel/share/roseus/ros/rdt_localization")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/rdt/Desktop/RMC20-nucnodes/nuc_code/devel/share/common-lisp/ros/rdt_localization")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/rdt/Desktop/RMC20-nucnodes/nuc_code/devel/share/gennodejs/ros/rdt_localization")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/rdt/Desktop/RMC20-nucnodes/nuc_code/devel/lib/python2.7/dist-packages/rdt_localization")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/rdt/Desktop/RMC20-nucnodes/nuc_code/devel/lib/python2.7/dist-packages/rdt_localization")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/rdt/Desktop/RMC20-nucnodes/nuc_code/build/rdt_localization/catkin_generated/installspace/rdt_localization.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rdt_localization/cmake" TYPE FILE FILES "/home/rdt/Desktop/RMC20-nucnodes/nuc_code/build/rdt_localization/catkin_generated/installspace/rdt_localization-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rdt_localization/cmake" TYPE FILE FILES
    "/home/rdt/Desktop/RMC20-nucnodes/nuc_code/build/rdt_localization/catkin_generated/installspace/rdt_localizationConfig.cmake"
    "/home/rdt/Desktop/RMC20-nucnodes/nuc_code/build/rdt_localization/catkin_generated/installspace/rdt_localizationConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rdt_localization" TYPE FILE FILES "/home/rdt/Desktop/RMC20-nucnodes/nuc_code/src/rdt_localization/package.xml")
endif()

