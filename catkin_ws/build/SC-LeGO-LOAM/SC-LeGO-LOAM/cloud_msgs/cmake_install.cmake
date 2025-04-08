# Install script for directory: /home/sunhr/FOR-SC-LeGO-LOAM/catkin_ws/src/SC-LeGO-LOAM/SC-LeGO-LOAM/cloud_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/sunhr/FOR-SC-LeGO-LOAM/catkin_ws/install")
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

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cloud_msgs/msg" TYPE FILE FILES "/home/sunhr/FOR-SC-LeGO-LOAM/catkin_ws/src/SC-LeGO-LOAM/SC-LeGO-LOAM/cloud_msgs/msg/cloud_info.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cloud_msgs/cmake" TYPE FILE FILES "/home/sunhr/FOR-SC-LeGO-LOAM/catkin_ws/build/SC-LeGO-LOAM/SC-LeGO-LOAM/cloud_msgs/catkin_generated/installspace/cloud_msgs-msg-paths.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/sunhr/FOR-SC-LeGO-LOAM/catkin_ws/devel/include/cloud_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/sunhr/FOR-SC-LeGO-LOAM/catkin_ws/devel/share/roseus/ros/cloud_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/sunhr/FOR-SC-LeGO-LOAM/catkin_ws/devel/share/common-lisp/ros/cloud_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/sunhr/FOR-SC-LeGO-LOAM/catkin_ws/devel/share/gennodejs/ros/cloud_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/sunhr/FOR-SC-LeGO-LOAM/catkin_ws/devel/lib/python2.7/dist-packages/cloud_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/sunhr/FOR-SC-LeGO-LOAM/catkin_ws/devel/lib/python2.7/dist-packages/cloud_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/sunhr/FOR-SC-LeGO-LOAM/catkin_ws/build/SC-LeGO-LOAM/SC-LeGO-LOAM/cloud_msgs/catkin_generated/installspace/cloud_msgs.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cloud_msgs/cmake" TYPE FILE FILES "/home/sunhr/FOR-SC-LeGO-LOAM/catkin_ws/build/SC-LeGO-LOAM/SC-LeGO-LOAM/cloud_msgs/catkin_generated/installspace/cloud_msgs-msg-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cloud_msgs/cmake" TYPE FILE FILES
    "/home/sunhr/FOR-SC-LeGO-LOAM/catkin_ws/build/SC-LeGO-LOAM/SC-LeGO-LOAM/cloud_msgs/catkin_generated/installspace/cloud_msgsConfig.cmake"
    "/home/sunhr/FOR-SC-LeGO-LOAM/catkin_ws/build/SC-LeGO-LOAM/SC-LeGO-LOAM/cloud_msgs/catkin_generated/installspace/cloud_msgsConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cloud_msgs" TYPE FILE FILES "/home/sunhr/FOR-SC-LeGO-LOAM/catkin_ws/src/SC-LeGO-LOAM/SC-LeGO-LOAM/cloud_msgs/package.xml")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/sunhr/FOR-SC-LeGO-LOAM/catkin_ws/build/SC-LeGO-LOAM/SC-LeGO-LOAM/cloud_msgs/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
