# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

string serial                           # Tracker serial number
ff_hw_msgs/ViveSensor[] sensors         # Sensor position/normals
geometry_msgs/Vector3 acc_bias          # Acceleromater bias
geometry_msgs/Vector3 acc_scale         # Accelerometer scale
geometry_msgs/Vector3 gyr_bias          # Gyro bias
geometry_msgs/Vector3 gyr_scale         # Gyro scale
geometry_msgs/Transform p_ti            # Transform: IMU -> PHOTODIODE
geometry_msgs/Transform p_th            # Transform: HEAD -> PHOTODIODE
uint32 lastcount                        # Last count
uint32 overflows                        # Number of overflows
