#Copyright(c) 2017, United States Government, as represented by the
#Administrator of the National Aeronautics and Space Administration.
#
#All rights reserved.
#
#The Astrobee platform is licensed under the Apache License, Version 2.0
#(the "License"); you may not use this file except in compliance with the
#License.You may obtain a copy of the License at
#
#http:  // www.apache.org/licenses/LICENSE-2.0
#
#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.See the
#License for the specific language governing permissions and limitations
#under the License.

cmake_minimum_required(VERSION 3.0)
project(bag_processing)

if (USE_ROS)
  ## Compile as C++14, supported in ROS Kinetic and newer
  add_compile_options(-std=c++14)
  add_definitions(-DROS1)

  ## Find catkin macros and libraries
  find_package(catkin2 REQUIRED COMPONENTS
    rosbag
  )

  catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS rosbag
  )

  if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/test_fix_all.test)
  endif()
endif (USE_ROS)
