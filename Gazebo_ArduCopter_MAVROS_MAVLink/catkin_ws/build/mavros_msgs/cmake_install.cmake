# Install script for directory: /home/mini/catkin_ws/src/mavros/mavros_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/mini/catkin_ws/install")
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
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/mini/catkin_ws/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/mini/catkin_ws/install" TYPE PROGRAM FILES "/home/mini/catkin_ws/build/mavros_msgs/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/mini/catkin_ws/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/mini/catkin_ws/install" TYPE PROGRAM FILES "/home/mini/catkin_ws/build/mavros_msgs/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/mini/catkin_ws/install/setup.bash;/home/mini/catkin_ws/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/mini/catkin_ws/install" TYPE FILE FILES
    "/home/mini/catkin_ws/build/mavros_msgs/catkin_generated/installspace/setup.bash"
    "/home/mini/catkin_ws/build/mavros_msgs/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/mini/catkin_ws/install/setup.sh;/home/mini/catkin_ws/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/mini/catkin_ws/install" TYPE FILE FILES
    "/home/mini/catkin_ws/build/mavros_msgs/catkin_generated/installspace/setup.sh"
    "/home/mini/catkin_ws/build/mavros_msgs/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/mini/catkin_ws/install/setup.zsh;/home/mini/catkin_ws/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/mini/catkin_ws/install" TYPE FILE FILES
    "/home/mini/catkin_ws/build/mavros_msgs/catkin_generated/installspace/setup.zsh"
    "/home/mini/catkin_ws/build/mavros_msgs/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/mini/catkin_ws/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/mini/catkin_ws/install" TYPE FILE FILES "/home/mini/catkin_ws/build/mavros_msgs/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mavros_msgs/msg" TYPE FILE FILES
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/ADSBVehicle.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/ActuatorControl.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/Altitude.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/AttitudeTarget.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/BatteryStatus.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/CamIMUStamp.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/CellularStatus.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/CameraImageCaptured.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/CommandCode.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/CompanionProcessStatus.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/OnboardComputerStatus.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/DebugValue.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/ESCInfo.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/ESCInfoItem.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/ESCStatus.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/ESCStatusItem.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/ESCTelemetry.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/ESCTelemetryItem.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/EstimatorStatus.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/ExtendedState.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/FileEntry.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/GlobalPositionTarget.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/GPSINPUT.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/GPSRAW.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/GPSRTK.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/HilActuatorControls.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/HilControls.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/HilGPS.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/HilSensor.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/HilStateQuaternion.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/HomePosition.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/LandingTarget.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/LogData.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/LogEntry.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/MagnetometerReporter.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/ManualControl.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/Mavlink.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/MountControl.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/NavControllerOutput.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/OpticalFlowRad.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/OverrideRCIn.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/Param.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/ParamValue.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/PlayTuneV2.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/PositionTarget.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/RCIn.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/RCOut.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/RTCM.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/RadioStatus.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/RTKBaseline.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/State.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/StatusText.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/TerrainReport.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/Thrust.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/TimesyncStatus.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/Trajectory.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/Tunnel.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/VFR_HUD.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/VehicleInfo.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/Vibration.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/Waypoint.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/WaypointList.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/WaypointReached.msg"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/msg/WheelOdomStamped.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mavros_msgs/srv" TYPE FILE FILES
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/CommandAck.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/CommandBool.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/CommandHome.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/CommandInt.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/CommandLong.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/CommandTOL.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/CommandTriggerControl.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/CommandTriggerInterval.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/CommandVtolTransition.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/FileChecksum.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/FileClose.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/FileList.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/FileMakeDir.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/FileOpen.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/FileRead.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/FileRemove.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/FileRemoveDir.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/FileRename.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/FileTruncate.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/FileWrite.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/LogRequestData.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/LogRequestEnd.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/LogRequestList.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/MountConfigure.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/MessageInterval.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/ParamGet.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/ParamPull.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/ParamPush.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/ParamSet.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/SetMavFrame.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/SetMode.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/StreamRate.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/VehicleInfoGet.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/WaypointClear.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/WaypointPull.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/WaypointPush.srv"
    "/home/mini/catkin_ws/src/mavros/mavros_msgs/srv/WaypointSetCurrent.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mavros_msgs/cmake" TYPE FILE FILES "/home/mini/catkin_ws/build/mavros_msgs/catkin_generated/installspace/mavros_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/mini/catkin_ws/devel/.private/mavros_msgs/include/mavros_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/mini/catkin_ws/devel/.private/mavros_msgs/share/roseus/ros/mavros_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/mini/catkin_ws/devel/.private/mavros_msgs/share/common-lisp/ros/mavros_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/mini/catkin_ws/devel/.private/mavros_msgs/share/gennodejs/ros/mavros_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/mini/catkin_ws/devel/.private/mavros_msgs/lib/python3/dist-packages/mavros_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/mini/catkin_ws/devel/.private/mavros_msgs/lib/python3/dist-packages/mavros_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/mini/catkin_ws/build/mavros_msgs/catkin_generated/installspace/mavros_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mavros_msgs/cmake" TYPE FILE FILES "/home/mini/catkin_ws/build/mavros_msgs/catkin_generated/installspace/mavros_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mavros_msgs/cmake" TYPE FILE FILES
    "/home/mini/catkin_ws/build/mavros_msgs/catkin_generated/installspace/mavros_msgsConfig.cmake"
    "/home/mini/catkin_ws/build/mavros_msgs/catkin_generated/installspace/mavros_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mavros_msgs" TYPE FILE FILES "/home/mini/catkin_ws/src/mavros/mavros_msgs/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mavros_msgs" TYPE DIRECTORY FILES "/home/mini/catkin_ws/src/mavros/mavros_msgs/include/mavros_msgs/" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/mini/catkin_ws/build/mavros_msgs/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/mini/catkin_ws/build/mavros_msgs/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
