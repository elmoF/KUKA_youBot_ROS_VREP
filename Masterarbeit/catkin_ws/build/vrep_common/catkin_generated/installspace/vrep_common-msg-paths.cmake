# generated from genmsg/cmake/pkg-msg-paths.cmake.em

# message include dirs in installspace
_prepend_path("${vrep_common_DIR}/.." "msg" vrep_common_MSG_INCLUDE_DIRS UNIQUE)
set(vrep_common_MSG_DEPENDENCIES std_msgs;sensor_msgs)
