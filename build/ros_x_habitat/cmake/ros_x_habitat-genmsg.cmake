# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ros_x_habitat: 2 messages, 5 services")

set(MSG_I_FLAGS "-Iros_x_habitat:/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ros_x_habitat_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/msg/PointGoalWithGPSCompass.msg" NAME_WE)
add_custom_target(_ros_x_habitat_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ros_x_habitat" "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/msg/PointGoalWithGPSCompass.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/msg/DepthImage.msg" NAME_WE)
add_custom_target(_ros_x_habitat_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ros_x_habitat" "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/msg/DepthImage.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/EvalEpisode.srv" NAME_WE)
add_custom_target(_ros_x_habitat_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ros_x_habitat" "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/EvalEpisode.srv" ""
)

get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/ResetAgent.srv" NAME_WE)
add_custom_target(_ros_x_habitat_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ros_x_habitat" "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/ResetAgent.srv" ""
)

get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/GetAgentTime.srv" NAME_WE)
add_custom_target(_ros_x_habitat_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ros_x_habitat" "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/GetAgentTime.srv" ""
)

get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/Roam.srv" NAME_WE)
add_custom_target(_ros_x_habitat_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ros_x_habitat" "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/Roam.srv" ""
)

get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/GetAgentPose.srv" NAME_WE)
add_custom_target(_ros_x_habitat_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ros_x_habitat" "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/GetAgentPose.srv" "geometry_msgs/Point:geometry_msgs/Quaternion:geometry_msgs/Pose"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/msg/PointGoalWithGPSCompass.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ros_x_habitat
)
_generate_msg_cpp(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/msg/DepthImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ros_x_habitat
)

### Generating Services
_generate_srv_cpp(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/EvalEpisode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ros_x_habitat
)
_generate_srv_cpp(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/ResetAgent.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ros_x_habitat
)
_generate_srv_cpp(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/GetAgentTime.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ros_x_habitat
)
_generate_srv_cpp(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/Roam.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ros_x_habitat
)
_generate_srv_cpp(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/GetAgentPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ros_x_habitat
)

### Generating Module File
_generate_module_cpp(ros_x_habitat
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ros_x_habitat
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ros_x_habitat_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ros_x_habitat_generate_messages ros_x_habitat_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/msg/PointGoalWithGPSCompass.msg" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_cpp _ros_x_habitat_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/msg/DepthImage.msg" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_cpp _ros_x_habitat_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/EvalEpisode.srv" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_cpp _ros_x_habitat_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/ResetAgent.srv" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_cpp _ros_x_habitat_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/GetAgentTime.srv" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_cpp _ros_x_habitat_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/Roam.srv" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_cpp _ros_x_habitat_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/GetAgentPose.srv" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_cpp _ros_x_habitat_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ros_x_habitat_gencpp)
add_dependencies(ros_x_habitat_gencpp ros_x_habitat_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ros_x_habitat_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/msg/PointGoalWithGPSCompass.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ros_x_habitat
)
_generate_msg_eus(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/msg/DepthImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ros_x_habitat
)

### Generating Services
_generate_srv_eus(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/EvalEpisode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ros_x_habitat
)
_generate_srv_eus(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/ResetAgent.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ros_x_habitat
)
_generate_srv_eus(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/GetAgentTime.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ros_x_habitat
)
_generate_srv_eus(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/Roam.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ros_x_habitat
)
_generate_srv_eus(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/GetAgentPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ros_x_habitat
)

### Generating Module File
_generate_module_eus(ros_x_habitat
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ros_x_habitat
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(ros_x_habitat_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(ros_x_habitat_generate_messages ros_x_habitat_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/msg/PointGoalWithGPSCompass.msg" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_eus _ros_x_habitat_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/msg/DepthImage.msg" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_eus _ros_x_habitat_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/EvalEpisode.srv" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_eus _ros_x_habitat_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/ResetAgent.srv" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_eus _ros_x_habitat_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/GetAgentTime.srv" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_eus _ros_x_habitat_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/Roam.srv" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_eus _ros_x_habitat_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/GetAgentPose.srv" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_eus _ros_x_habitat_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ros_x_habitat_geneus)
add_dependencies(ros_x_habitat_geneus ros_x_habitat_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ros_x_habitat_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/msg/PointGoalWithGPSCompass.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ros_x_habitat
)
_generate_msg_lisp(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/msg/DepthImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ros_x_habitat
)

### Generating Services
_generate_srv_lisp(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/EvalEpisode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ros_x_habitat
)
_generate_srv_lisp(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/ResetAgent.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ros_x_habitat
)
_generate_srv_lisp(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/GetAgentTime.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ros_x_habitat
)
_generate_srv_lisp(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/Roam.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ros_x_habitat
)
_generate_srv_lisp(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/GetAgentPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ros_x_habitat
)

### Generating Module File
_generate_module_lisp(ros_x_habitat
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ros_x_habitat
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ros_x_habitat_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ros_x_habitat_generate_messages ros_x_habitat_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/msg/PointGoalWithGPSCompass.msg" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_lisp _ros_x_habitat_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/msg/DepthImage.msg" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_lisp _ros_x_habitat_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/EvalEpisode.srv" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_lisp _ros_x_habitat_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/ResetAgent.srv" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_lisp _ros_x_habitat_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/GetAgentTime.srv" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_lisp _ros_x_habitat_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/Roam.srv" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_lisp _ros_x_habitat_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/GetAgentPose.srv" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_lisp _ros_x_habitat_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ros_x_habitat_genlisp)
add_dependencies(ros_x_habitat_genlisp ros_x_habitat_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ros_x_habitat_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/msg/PointGoalWithGPSCompass.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ros_x_habitat
)
_generate_msg_nodejs(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/msg/DepthImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ros_x_habitat
)

### Generating Services
_generate_srv_nodejs(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/EvalEpisode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ros_x_habitat
)
_generate_srv_nodejs(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/ResetAgent.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ros_x_habitat
)
_generate_srv_nodejs(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/GetAgentTime.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ros_x_habitat
)
_generate_srv_nodejs(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/Roam.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ros_x_habitat
)
_generate_srv_nodejs(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/GetAgentPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ros_x_habitat
)

### Generating Module File
_generate_module_nodejs(ros_x_habitat
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ros_x_habitat
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(ros_x_habitat_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(ros_x_habitat_generate_messages ros_x_habitat_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/msg/PointGoalWithGPSCompass.msg" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_nodejs _ros_x_habitat_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/msg/DepthImage.msg" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_nodejs _ros_x_habitat_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/EvalEpisode.srv" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_nodejs _ros_x_habitat_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/ResetAgent.srv" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_nodejs _ros_x_habitat_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/GetAgentTime.srv" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_nodejs _ros_x_habitat_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/Roam.srv" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_nodejs _ros_x_habitat_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/GetAgentPose.srv" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_nodejs _ros_x_habitat_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ros_x_habitat_gennodejs)
add_dependencies(ros_x_habitat_gennodejs ros_x_habitat_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ros_x_habitat_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/msg/PointGoalWithGPSCompass.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ros_x_habitat
)
_generate_msg_py(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/msg/DepthImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ros_x_habitat
)

### Generating Services
_generate_srv_py(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/EvalEpisode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ros_x_habitat
)
_generate_srv_py(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/ResetAgent.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ros_x_habitat
)
_generate_srv_py(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/GetAgentTime.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ros_x_habitat
)
_generate_srv_py(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/Roam.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ros_x_habitat
)
_generate_srv_py(ros_x_habitat
  "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/GetAgentPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ros_x_habitat
)

### Generating Module File
_generate_module_py(ros_x_habitat
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ros_x_habitat
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ros_x_habitat_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ros_x_habitat_generate_messages ros_x_habitat_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/msg/PointGoalWithGPSCompass.msg" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_py _ros_x_habitat_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/msg/DepthImage.msg" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_py _ros_x_habitat_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/EvalEpisode.srv" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_py _ros_x_habitat_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/ResetAgent.srv" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_py _ros_x_habitat_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/GetAgentTime.srv" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_py _ros_x_habitat_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/Roam.srv" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_py _ros_x_habitat_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/GetAgentPose.srv" NAME_WE)
add_dependencies(ros_x_habitat_generate_messages_py _ros_x_habitat_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ros_x_habitat_genpy)
add_dependencies(ros_x_habitat_genpy ros_x_habitat_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ros_x_habitat_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ros_x_habitat)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ros_x_habitat
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(ros_x_habitat_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(ros_x_habitat_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ros_x_habitat)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ros_x_habitat
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(ros_x_habitat_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(ros_x_habitat_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ros_x_habitat)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ros_x_habitat
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(ros_x_habitat_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(ros_x_habitat_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ros_x_habitat)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ros_x_habitat
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(ros_x_habitat_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(ros_x_habitat_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ros_x_habitat)
  install(CODE "execute_process(COMMAND \"/home/kjsbrian/miniconda3/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ros_x_habitat\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ros_x_habitat
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(ros_x_habitat_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(ros_x_habitat_generate_messages_py geometry_msgs_generate_messages_py)
endif()
