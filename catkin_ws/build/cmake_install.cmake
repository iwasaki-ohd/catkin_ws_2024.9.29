# Install script for directory: /home/moriokalab-pc16/catkin_ws/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/moriokalab-pc16/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/moriokalab-pc16/catkin_ws/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/moriokalab-pc16/catkin_ws/install" TYPE PROGRAM FILES "/home/moriokalab-pc16/catkin_ws/build/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/moriokalab-pc16/catkin_ws/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/moriokalab-pc16/catkin_ws/install" TYPE PROGRAM FILES "/home/moriokalab-pc16/catkin_ws/build/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/moriokalab-pc16/catkin_ws/install/setup.bash;/home/moriokalab-pc16/catkin_ws/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/moriokalab-pc16/catkin_ws/install" TYPE FILE FILES
    "/home/moriokalab-pc16/catkin_ws/build/catkin_generated/installspace/setup.bash"
    "/home/moriokalab-pc16/catkin_ws/build/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/moriokalab-pc16/catkin_ws/install/setup.sh;/home/moriokalab-pc16/catkin_ws/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/moriokalab-pc16/catkin_ws/install" TYPE FILE FILES
    "/home/moriokalab-pc16/catkin_ws/build/catkin_generated/installspace/setup.sh"
    "/home/moriokalab-pc16/catkin_ws/build/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/moriokalab-pc16/catkin_ws/install/setup.zsh;/home/moriokalab-pc16/catkin_ws/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/moriokalab-pc16/catkin_ws/install" TYPE FILE FILES
    "/home/moriokalab-pc16/catkin_ws/build/catkin_generated/installspace/setup.zsh"
    "/home/moriokalab-pc16/catkin_ws/build/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/moriokalab-pc16/catkin_ws/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/moriokalab-pc16/catkin_ws/install" TYPE FILE FILES "/home/moriokalab-pc16/catkin_ws/build/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/moriokalab-pc16/catkin_ws/build/gtest/cmake_install.cmake")
  include("/home/moriokalab-pc16/catkin_ws/build/fast_gicp/cmake_install.cmake")
  include("/home/moriokalab-pc16/catkin_ws/build/realsense-ros/realsense2_description/cmake_install.cmake")
  include("/home/moriokalab-pc16/catkin_ws/build/detection_msgs/cmake_install.cmake")
  include("/home/moriokalab-pc16/catkin_ws/build/ros_posenet/cmake_install.cmake")
  include("/home/moriokalab-pc16/catkin_ws/build/camera_opencv/cmake_install.cmake")
  include("/home/moriokalab-pc16/catkin_ws/build/hello/cmake_install.cmake")
  include("/home/moriokalab-pc16/catkin_ws/build/image_gain1/cmake_install.cmake")
  include("/home/moriokalab-pc16/catkin_ws/build/internship_css/cmake_install.cmake")
  include("/home/moriokalab-pc16/catkin_ws/build/keypoints_posenet/cmake_install.cmake")
  include("/home/moriokalab-pc16/catkin_ws/build/motpy/cmake_install.cmake")
  include("/home/moriokalab-pc16/catkin_ws/build/my_topic/cmake_install.cmake")
  include("/home/moriokalab-pc16/catkin_ws/build/person_tracking_ros/cmake_install.cmake")
  include("/home/moriokalab-pc16/catkin_ws/build/ros_iou_tracking/cmake_install.cmake")
  include("/home/moriokalab-pc16/catkin_ws/build/ROS-TCP-Endpoint/cmake_install.cmake")
  include("/home/moriokalab-pc16/catkin_ws/build/rowma_ros/cmake_install.cmake")
  include("/home/moriokalab-pc16/catkin_ws/build/sort_detection_and_tracking/cmake_install.cmake")
  include("/home/moriokalab-pc16/catkin_ws/build/darknet_ros/darknet_ros_msgs/cmake_install.cmake")
  include("/home/moriokalab-pc16/catkin_ws/build/Yolov5_DeepSort_Pytorch_ros/src/detection_only/cmake_install.cmake")
  include("/home/moriokalab-pc16/catkin_ws/build/darknet_ros/darknet_ros/cmake_install.cmake")
  include("/home/moriokalab-pc16/catkin_ws/build/motpy_ros/motpy_ros/cmake_install.cmake")
  include("/home/moriokalab-pc16/catkin_ws/build/opencv_example/cmake_install.cmake")
  include("/home/moriokalab-pc16/catkin_ws/build/hdl_global_localization/cmake_install.cmake")
  include("/home/moriokalab-pc16/catkin_ws/build/ndt_omp/cmake_install.cmake")
  include("/home/moriokalab-pc16/catkin_ws/build/realsense-ros/realsense2_camera/cmake_install.cmake")
  include("/home/moriokalab-pc16/catkin_ws/build/hdl_localization/cmake_install.cmake")
  include("/home/moriokalab-pc16/catkin_ws/build/tracking_person_yolo/cmake_install.cmake")
  include("/home/moriokalab-pc16/catkin_ws/build/yolo_person/cmake_install.cmake")
  include("/home/moriokalab-pc16/catkin_ws/build/yolov5_ros/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/moriokalab-pc16/catkin_ws/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
