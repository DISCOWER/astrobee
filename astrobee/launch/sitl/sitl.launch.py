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


from utilities.utilities import *

import subprocess


def kill_px4_processes():
    try:
        # Use the "pgrep" command to find all "px4" processes
        process = subprocess.Popen(["pgrep", "px4"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, _ = process.communicate()
        if process.returncode == 0:
            # Parse the process IDs from the output
            pids = [int(pid) for pid in stdout.split()]

            # Kill each "px4" process
            for pid in pids:
                subprocess.run(["kill", str(pid)])

            print(f"Successfully killed {len(pids)} px4 processes.")
        else:
            print("No px4 processes found.")
    except Exception as e:
        print(f"An error occurred while killing px4 processes: {e}")


def launch_px4_instance():
    path_to_bin = os.getenv("PX4_BINARY_PATH")
    path_to_build = path_to_bin + "/../"
    args = (path_to_bin + "/px4", path_to_build + "/etc", "-s", "etc/init.d-posix/rcS", "-i", "0", "-d")
    process = subprocess.Popen(args, shell=True)
    output, error = process.communicate()


def generate_launch_description():
    kill_px4_processes()
    launch_px4_instance()
    return
