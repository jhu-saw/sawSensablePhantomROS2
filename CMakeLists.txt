#
# (C) Copyright 2015-2022 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (sensable_phantom VERSION 1.1)

find_package (ament_cmake REQUIRED)
find_package (rclcpp REQUIRED)
find_package (std_msgs REQUIRED)
find_package (std_srvs REQUIRED)
find_package (geometry_msgs REQUIRED)
find_package (sensor_msgs REQUIRED)
find_package (diagnostic_msgs REQUIRED)
find_package (cisst_msgs REQUIRED)
find_package (tf2_ros REQUIRED)
find_package (tf2_msgs REQUIRED)
find_package (crtk_msgs REQUIRED)
find_package (cisst_ros2_bridge REQUIRED)
find_package (cisst_ros2_crtk REQUIRED)

# find cisst and make sure the required libraries have been compiled
set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstCommonQt
     cisstVector
     cisstVectorQt
     cisstOSAbstraction
     cisstMultiTask
     cisstMultiTaskQt
     cisstParameterTypes
     cisstParameterTypesQt
     cisstQt
     )

find_package (cisst 1.1.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  include (${CISST_USE_FILE})

  find_package (sawSensablePhantom 1.0.0 REQUIRED)

  include_directories (
    ${sensable_phantom_SOURCE_DIR}
    ${CATKIN_DEVEL_PREFIX}/include
    ${sawSensablePhantom_INCLUDE_DIR}
    )

  link_directories (
    ${sawSensablePhantom_LIBRARY_DIR}
    )

  # executable
  add_executable (sensable_phantom mainQt.cpp)
  target_link_libraries (
    sensable_phantom
    ${sawSensablePhantom_LIBRARIES}
    ${catkin_LIBRARIES}
    )
  cisst_target_link_libraries (sensable_phantom ${REQUIRED_CISST_LIBRARIES})
  ament_target_dependencies (
    sensable_phantom
    rclcpp
    std_msgs
    std_srvs
    geometry_msgs
    sensor_msgs
    diagnostic_msgs
    cisst_msgs
    tf2_ros
    crtk_msgs
    cisst_ros2_bridge
    cisst_ros2_crtk
    )

  install (TARGETS sensable_phantom
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib
    INCLUDES DESTINATION include
    )

  # launch files
  install (
    PROGRAMS launch/sensable_phantom_rviz.launch.py
    DESTINATION share/${PROJECT_NAME}
    )

  ament_package ()

endif (cisst_FOUND_AS_REQUIRED)
