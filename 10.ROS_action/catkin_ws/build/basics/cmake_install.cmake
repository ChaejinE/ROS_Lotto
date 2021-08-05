# Install script for directory: /home/cjlotto/git_clone/ROS_Lotto/10.ROS_action/catkin_ws/src/basics

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/cjlotto/git_clone/ROS_Lotto/10.ROS_action/catkin_ws/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basics/msg" TYPE FILE FILES "/home/cjlotto/git_clone/ROS_Lotto/10.ROS_action/catkin_ws/src/basics/msg/Timer.action")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basics/msg" TYPE FILE FILES
    "/home/cjlotto/git_clone/ROS_Lotto/10.ROS_action/catkin_ws/devel/share/basics/msg/TimerAction.msg"
    "/home/cjlotto/git_clone/ROS_Lotto/10.ROS_action/catkin_ws/devel/share/basics/msg/TimerActionGoal.msg"
    "/home/cjlotto/git_clone/ROS_Lotto/10.ROS_action/catkin_ws/devel/share/basics/msg/TimerActionResult.msg"
    "/home/cjlotto/git_clone/ROS_Lotto/10.ROS_action/catkin_ws/devel/share/basics/msg/TimerActionFeedback.msg"
    "/home/cjlotto/git_clone/ROS_Lotto/10.ROS_action/catkin_ws/devel/share/basics/msg/TimerGoal.msg"
    "/home/cjlotto/git_clone/ROS_Lotto/10.ROS_action/catkin_ws/devel/share/basics/msg/TimerResult.msg"
    "/home/cjlotto/git_clone/ROS_Lotto/10.ROS_action/catkin_ws/devel/share/basics/msg/TimerFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basics/cmake" TYPE FILE FILES "/home/cjlotto/git_clone/ROS_Lotto/10.ROS_action/catkin_ws/build/basics/catkin_generated/installspace/basics-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/cjlotto/git_clone/ROS_Lotto/10.ROS_action/catkin_ws/devel/include/basics")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/cjlotto/git_clone/ROS_Lotto/10.ROS_action/catkin_ws/devel/share/roseus/ros/basics")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/cjlotto/git_clone/ROS_Lotto/10.ROS_action/catkin_ws/devel/share/common-lisp/ros/basics")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/cjlotto/git_clone/ROS_Lotto/10.ROS_action/catkin_ws/devel/share/gennodejs/ros/basics")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/cjlotto/git_clone/ROS_Lotto/10.ROS_action/catkin_ws/devel/lib/python2.7/dist-packages/basics")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/cjlotto/git_clone/ROS_Lotto/10.ROS_action/catkin_ws/devel/lib/python2.7/dist-packages/basics")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/cjlotto/git_clone/ROS_Lotto/10.ROS_action/catkin_ws/build/basics/catkin_generated/installspace/basics.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basics/cmake" TYPE FILE FILES "/home/cjlotto/git_clone/ROS_Lotto/10.ROS_action/catkin_ws/build/basics/catkin_generated/installspace/basics-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basics/cmake" TYPE FILE FILES
    "/home/cjlotto/git_clone/ROS_Lotto/10.ROS_action/catkin_ws/build/basics/catkin_generated/installspace/basicsConfig.cmake"
    "/home/cjlotto/git_clone/ROS_Lotto/10.ROS_action/catkin_ws/build/basics/catkin_generated/installspace/basicsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basics" TYPE FILE FILES "/home/cjlotto/git_clone/ROS_Lotto/10.ROS_action/catkin_ws/src/basics/package.xml")
endif()

