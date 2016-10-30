# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "vrep_skeleton_msg_and_srv: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(vrep_skeleton_msg_and_srv_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(vrep_skeleton_msg_and_srv
  "/home/youbot/Simulation_Examples/towerOfHanoi_ROS/catkin_ws/src/vrep_skeleton_msg_and_srv/srv/displayText.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vrep_skeleton_msg_and_srv
)

### Generating Module File
_generate_module_cpp(vrep_skeleton_msg_and_srv
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vrep_skeleton_msg_and_srv
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(vrep_skeleton_msg_and_srv_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(vrep_skeleton_msg_and_srv_generate_messages vrep_skeleton_msg_and_srv_generate_messages_cpp)

# target for backward compatibility
add_custom_target(vrep_skeleton_msg_and_srv_gencpp)
add_dependencies(vrep_skeleton_msg_and_srv_gencpp vrep_skeleton_msg_and_srv_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vrep_skeleton_msg_and_srv_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(vrep_skeleton_msg_and_srv
  "/home/youbot/Simulation_Examples/towerOfHanoi_ROS/catkin_ws/src/vrep_skeleton_msg_and_srv/srv/displayText.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vrep_skeleton_msg_and_srv
)

### Generating Module File
_generate_module_lisp(vrep_skeleton_msg_and_srv
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vrep_skeleton_msg_and_srv
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(vrep_skeleton_msg_and_srv_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(vrep_skeleton_msg_and_srv_generate_messages vrep_skeleton_msg_and_srv_generate_messages_lisp)

# target for backward compatibility
add_custom_target(vrep_skeleton_msg_and_srv_genlisp)
add_dependencies(vrep_skeleton_msg_and_srv_genlisp vrep_skeleton_msg_and_srv_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vrep_skeleton_msg_and_srv_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(vrep_skeleton_msg_and_srv
  "/home/youbot/Simulation_Examples/towerOfHanoi_ROS/catkin_ws/src/vrep_skeleton_msg_and_srv/srv/displayText.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vrep_skeleton_msg_and_srv
)

### Generating Module File
_generate_module_py(vrep_skeleton_msg_and_srv
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vrep_skeleton_msg_and_srv
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(vrep_skeleton_msg_and_srv_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(vrep_skeleton_msg_and_srv_generate_messages vrep_skeleton_msg_and_srv_generate_messages_py)

# target for backward compatibility
add_custom_target(vrep_skeleton_msg_and_srv_genpy)
add_dependencies(vrep_skeleton_msg_and_srv_genpy vrep_skeleton_msg_and_srv_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vrep_skeleton_msg_and_srv_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vrep_skeleton_msg_and_srv)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vrep_skeleton_msg_and_srv
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(vrep_skeleton_msg_and_srv_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vrep_skeleton_msg_and_srv)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vrep_skeleton_msg_and_srv
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(vrep_skeleton_msg_and_srv_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vrep_skeleton_msg_and_srv)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vrep_skeleton_msg_and_srv\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vrep_skeleton_msg_and_srv
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(vrep_skeleton_msg_and_srv_generate_messages_py std_msgs_generate_messages_py)
