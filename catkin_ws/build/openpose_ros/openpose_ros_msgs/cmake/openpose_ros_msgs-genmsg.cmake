# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "openpose_ros_msgs: 4 messages, 0 services")

set(MSG_I_FLAGS "-Iopenpose_ros_msgs:/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(openpose_ros_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/BoundingBox.msg" NAME_WE)
add_custom_target(_openpose_ros_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "openpose_ros_msgs" "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/BoundingBox.msg" ""
)

get_filename_component(_filename "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/OpenPoseHuman.msg" NAME_WE)
add_custom_target(_openpose_ros_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "openpose_ros_msgs" "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/OpenPoseHuman.msg" "openpose_ros_msgs/BoundingBox:openpose_ros_msgs/PointWithProb"
)

get_filename_component(_filename "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/OpenPoseHumanList.msg" NAME_WE)
add_custom_target(_openpose_ros_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "openpose_ros_msgs" "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/OpenPoseHumanList.msg" "std_msgs/Header:openpose_ros_msgs/BoundingBox:openpose_ros_msgs/PointWithProb:openpose_ros_msgs/OpenPoseHuman"
)

get_filename_component(_filename "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/PointWithProb.msg" NAME_WE)
add_custom_target(_openpose_ros_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "openpose_ros_msgs" "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/PointWithProb.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(openpose_ros_msgs
  "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/openpose_ros_msgs
)
_generate_msg_cpp(openpose_ros_msgs
  "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/OpenPoseHuman.msg"
  "${MSG_I_FLAGS}"
  "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/BoundingBox.msg;/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/PointWithProb.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/openpose_ros_msgs
)
_generate_msg_cpp(openpose_ros_msgs
  "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/OpenPoseHumanList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/BoundingBox.msg;/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/PointWithProb.msg;/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/OpenPoseHuman.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/openpose_ros_msgs
)
_generate_msg_cpp(openpose_ros_msgs
  "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/PointWithProb.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/openpose_ros_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(openpose_ros_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/openpose_ros_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(openpose_ros_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(openpose_ros_msgs_generate_messages openpose_ros_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/BoundingBox.msg" NAME_WE)
add_dependencies(openpose_ros_msgs_generate_messages_cpp _openpose_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/OpenPoseHuman.msg" NAME_WE)
add_dependencies(openpose_ros_msgs_generate_messages_cpp _openpose_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/OpenPoseHumanList.msg" NAME_WE)
add_dependencies(openpose_ros_msgs_generate_messages_cpp _openpose_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/PointWithProb.msg" NAME_WE)
add_dependencies(openpose_ros_msgs_generate_messages_cpp _openpose_ros_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(openpose_ros_msgs_gencpp)
add_dependencies(openpose_ros_msgs_gencpp openpose_ros_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS openpose_ros_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(openpose_ros_msgs
  "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/openpose_ros_msgs
)
_generate_msg_eus(openpose_ros_msgs
  "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/OpenPoseHuman.msg"
  "${MSG_I_FLAGS}"
  "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/BoundingBox.msg;/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/PointWithProb.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/openpose_ros_msgs
)
_generate_msg_eus(openpose_ros_msgs
  "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/OpenPoseHumanList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/BoundingBox.msg;/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/PointWithProb.msg;/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/OpenPoseHuman.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/openpose_ros_msgs
)
_generate_msg_eus(openpose_ros_msgs
  "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/PointWithProb.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/openpose_ros_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(openpose_ros_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/openpose_ros_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(openpose_ros_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(openpose_ros_msgs_generate_messages openpose_ros_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/BoundingBox.msg" NAME_WE)
add_dependencies(openpose_ros_msgs_generate_messages_eus _openpose_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/OpenPoseHuman.msg" NAME_WE)
add_dependencies(openpose_ros_msgs_generate_messages_eus _openpose_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/OpenPoseHumanList.msg" NAME_WE)
add_dependencies(openpose_ros_msgs_generate_messages_eus _openpose_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/PointWithProb.msg" NAME_WE)
add_dependencies(openpose_ros_msgs_generate_messages_eus _openpose_ros_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(openpose_ros_msgs_geneus)
add_dependencies(openpose_ros_msgs_geneus openpose_ros_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS openpose_ros_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(openpose_ros_msgs
  "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/openpose_ros_msgs
)
_generate_msg_lisp(openpose_ros_msgs
  "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/OpenPoseHuman.msg"
  "${MSG_I_FLAGS}"
  "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/BoundingBox.msg;/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/PointWithProb.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/openpose_ros_msgs
)
_generate_msg_lisp(openpose_ros_msgs
  "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/OpenPoseHumanList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/BoundingBox.msg;/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/PointWithProb.msg;/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/OpenPoseHuman.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/openpose_ros_msgs
)
_generate_msg_lisp(openpose_ros_msgs
  "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/PointWithProb.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/openpose_ros_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(openpose_ros_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/openpose_ros_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(openpose_ros_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(openpose_ros_msgs_generate_messages openpose_ros_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/BoundingBox.msg" NAME_WE)
add_dependencies(openpose_ros_msgs_generate_messages_lisp _openpose_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/OpenPoseHuman.msg" NAME_WE)
add_dependencies(openpose_ros_msgs_generate_messages_lisp _openpose_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/OpenPoseHumanList.msg" NAME_WE)
add_dependencies(openpose_ros_msgs_generate_messages_lisp _openpose_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/PointWithProb.msg" NAME_WE)
add_dependencies(openpose_ros_msgs_generate_messages_lisp _openpose_ros_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(openpose_ros_msgs_genlisp)
add_dependencies(openpose_ros_msgs_genlisp openpose_ros_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS openpose_ros_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(openpose_ros_msgs
  "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/openpose_ros_msgs
)
_generate_msg_nodejs(openpose_ros_msgs
  "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/OpenPoseHuman.msg"
  "${MSG_I_FLAGS}"
  "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/BoundingBox.msg;/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/PointWithProb.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/openpose_ros_msgs
)
_generate_msg_nodejs(openpose_ros_msgs
  "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/OpenPoseHumanList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/BoundingBox.msg;/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/PointWithProb.msg;/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/OpenPoseHuman.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/openpose_ros_msgs
)
_generate_msg_nodejs(openpose_ros_msgs
  "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/PointWithProb.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/openpose_ros_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(openpose_ros_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/openpose_ros_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(openpose_ros_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(openpose_ros_msgs_generate_messages openpose_ros_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/BoundingBox.msg" NAME_WE)
add_dependencies(openpose_ros_msgs_generate_messages_nodejs _openpose_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/OpenPoseHuman.msg" NAME_WE)
add_dependencies(openpose_ros_msgs_generate_messages_nodejs _openpose_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/OpenPoseHumanList.msg" NAME_WE)
add_dependencies(openpose_ros_msgs_generate_messages_nodejs _openpose_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/PointWithProb.msg" NAME_WE)
add_dependencies(openpose_ros_msgs_generate_messages_nodejs _openpose_ros_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(openpose_ros_msgs_gennodejs)
add_dependencies(openpose_ros_msgs_gennodejs openpose_ros_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS openpose_ros_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(openpose_ros_msgs
  "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/openpose_ros_msgs
)
_generate_msg_py(openpose_ros_msgs
  "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/OpenPoseHuman.msg"
  "${MSG_I_FLAGS}"
  "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/BoundingBox.msg;/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/PointWithProb.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/openpose_ros_msgs
)
_generate_msg_py(openpose_ros_msgs
  "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/OpenPoseHumanList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/BoundingBox.msg;/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/PointWithProb.msg;/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/OpenPoseHuman.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/openpose_ros_msgs
)
_generate_msg_py(openpose_ros_msgs
  "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/PointWithProb.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/openpose_ros_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(openpose_ros_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/openpose_ros_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(openpose_ros_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(openpose_ros_msgs_generate_messages openpose_ros_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/BoundingBox.msg" NAME_WE)
add_dependencies(openpose_ros_msgs_generate_messages_py _openpose_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/OpenPoseHuman.msg" NAME_WE)
add_dependencies(openpose_ros_msgs_generate_messages_py _openpose_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/OpenPoseHumanList.msg" NAME_WE)
add_dependencies(openpose_ros_msgs_generate_messages_py _openpose_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/moriokalab-pc16/catkin_ws/src/openpose_ros/openpose_ros_msgs/msg/PointWithProb.msg" NAME_WE)
add_dependencies(openpose_ros_msgs_generate_messages_py _openpose_ros_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(openpose_ros_msgs_genpy)
add_dependencies(openpose_ros_msgs_genpy openpose_ros_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS openpose_ros_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/openpose_ros_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/openpose_ros_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(openpose_ros_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/openpose_ros_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/openpose_ros_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(openpose_ros_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/openpose_ros_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/openpose_ros_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(openpose_ros_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/openpose_ros_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/openpose_ros_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(openpose_ros_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/openpose_ros_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/openpose_ros_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/openpose_ros_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(openpose_ros_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
