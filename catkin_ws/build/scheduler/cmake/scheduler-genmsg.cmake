# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "scheduler: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators

add_custom_target(scheduler_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/rdt/catkin_ws/src/scheduler/srv/GetPhase.srv" NAME_WE)
add_custom_target(_scheduler_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "scheduler" "/home/rdt/catkin_ws/src/scheduler/srv/GetPhase.srv" ""
)

#
#  langs = 
#


