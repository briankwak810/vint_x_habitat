# Install script for directory: /home/kjsbrian/projects/vint_ws/src/ros_x_habitat

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/kjsbrian/projects/vint_ws/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_x_habitat/msg" TYPE FILE FILES
    "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/msg/PointGoalWithGPSCompass.msg"
    "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/msg/DepthImage.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_x_habitat/srv" TYPE FILE FILES
    "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/EvalEpisode.srv"
    "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/ResetAgent.srv"
    "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/GetAgentTime.srv"
    "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/Roam.srv"
    "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/srv/GetAgentPose.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_x_habitat/cmake" TYPE FILE FILES "/home/kjsbrian/projects/vint_ws/build/ros_x_habitat/catkin_generated/installspace/ros_x_habitat-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/kjsbrian/projects/vint_ws/devel/include/ros_x_habitat")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/kjsbrian/projects/vint_ws/devel/share/roseus/ros/ros_x_habitat")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/kjsbrian/projects/vint_ws/devel/share/common-lisp/ros/ros_x_habitat")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/kjsbrian/projects/vint_ws/devel/share/gennodejs/ros/ros_x_habitat")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/home/kjsbrian/miniconda3/bin/python3" -m compileall "/home/kjsbrian/projects/vint_ws/devel/lib/python3/dist-packages/ros_x_habitat")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/kjsbrian/projects/vint_ws/devel/lib/python3/dist-packages/ros_x_habitat")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/kjsbrian/projects/vint_ws/build/ros_x_habitat/catkin_generated/installspace/ros_x_habitat.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_x_habitat/cmake" TYPE FILE FILES "/home/kjsbrian/projects/vint_ws/build/ros_x_habitat/catkin_generated/installspace/ros_x_habitat-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_x_habitat/cmake" TYPE FILE FILES
    "/home/kjsbrian/projects/vint_ws/build/ros_x_habitat/catkin_generated/installspace/ros_x_habitatConfig.cmake"
    "/home/kjsbrian/projects/vint_ws/build/ros_x_habitat/catkin_generated/installspace/ros_x_habitatConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_x_habitat" TYPE FILE FILES "/home/kjsbrian/projects/vint_ws/src/ros_x_habitat/package.xml")
endif()

