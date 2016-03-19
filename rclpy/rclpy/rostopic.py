# Copyright 2016 Erle Robotics, LLC
#
# A relevant part of the code has been written taking inpiration
# from ROS 1 ros_comm package attributed to Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import time
import os
import errno
import sys

from optparse import OptionParser

NAME='rostopic'

def _rostopic_cmd_hz(argv):
    pass
def _rostopic_cmd_type(argv):
    pass
def _rostopic_cmd_list(argv):
    pass
def _rostopic_cmd_info(argv):
    pass
def _rostopic_cmd_pub(argv):
    pass
def _rostopic_cmd_bw(argv):
    pass
def _rostopic_cmd_find(argv):
    pass
def _rostopic_cmd_delay(argv):
    pass
def _rostopic_cmd_echo(argv):
    pass

def _fullusage():
    print("""rostopic is a command-line tool for printing information about ROS Topics.
Commands:
\trostopic bw (NOT IMPLEMENTED)\tdisplay bandwidth used by topic
\trostopic delay (NOT IMPLEMENTED)\tdisplay delay of topic from timestamp in header
\trostopic echo\tprint messages to screen
\trostopic find (NOT IMPLEMENTED)\tfind topics by type
\trostopic hz (NOT IMPLEMENTED)\tdisplay publishing rate of topic    
\trostopic info (NOT IMPLEMENTED)\tprint information about active topic
\trostopic list (NOT IMPLEMENTED)\tlist active topics
\trostopic pub (NOT IMPLEMENTED)\tpublish data to topic
\trostopic type (NOT IMPLEMENTED)\tprint topic type
Type rostopic <command> -h for more detailed usage, e.g. 'rostopic echo -h'
""")
    sys.exit(getattr(os, 'EX_USAGE', 1))

def rostopicmain(argv=None):
    if argv is None:
        argv=sys.argv
    
    # TODO FIXME, review
    # filter out remapping arguments in case we are being invoked via roslaunch
    #argv = rospy.myargv(argv)
    
    # process argv
    if len(argv) == 1:
        _fullusage()
    try:
        command = argv[1]
        if command == 'echo':
            _rostopic_cmd_echo(argv)
        elif command == 'hz':
            _rostopic_cmd_hz(argv)
        elif command == 'type':
            _rostopic_cmd_type(argv)
        elif command == 'list':
            _rostopic_cmd_list(argv)
        elif command == 'info':
            _rostopic_cmd_info(argv)
        elif command == 'pub':
            _rostopic_cmd_pub(argv)
        elif command == 'bw':
            _rostopic_cmd_bw(argv)
        elif command == 'find':
            _rostopic_cmd_find(argv)
        elif command == 'delay':
            _rostopic_cmd_delay(argv)
        else:
            _fullusage()
    except socket.error:
        sys.stderr.write("Network communication failed.\n")
        sys.exit(1)
    except Exception as e:
        sys.stderr.write("ERROR: %s\n"%str(e))
        sys.exit(1)