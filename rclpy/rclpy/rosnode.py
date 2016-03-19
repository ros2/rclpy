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

NAME='rosnode'

# TODO implement
def _rosnode_cmd_ping(argv):
    print("NOT IMPLEMENTED\n")
    sys.exit(0)

# TODO implement
def _rosnode_cmd_info(argv):
    print("NOT IMPLEMENTED\n")
    sys.exit(0)

# TODO implement
def _rosnode_cmd_machine(argv):
    print("NOT IMPLEMENTED\n")
    sys.exit(0)

# TODO implement
def _rosnode_cmd_cleanup(argv):
    print("NOT IMPLEMENTED\n")
    sys.exit(0)

# TODO implement
def _rosnode_cmd_kill(argv):
    print("NOT IMPLEMENTED\n")
    sys.exit(0)

def _rosnode_cmd_list(argv):
    """
    Implements rosnode 'list' command.
    """
    args = argv[2:]
    parser = OptionParser(usage="usage: %prog list", prog=NAME)
    parser.add_option("-u",
                      dest="list_uri", default=False,
                      action="store_true",
                      help="list XML-RPC URIs (NOT IMPLEMENTED)")
    parser.add_option("-a","--all",
                      dest="list_all", default=False,
                      action="store_true",
                      help="list all information (NOT IMPLEMENTED)")
    (options, args) = parser.parse_args(args)
    namespace = None
    if len(args) > 1:
        parser.error("invalid args: you may only specify one namespace")
    elif len(args) == 1:
        #namespace = rosgraph.names.script_resolve_name('rostopic', args[0])
        pass

    # In ROS 1, the rosnode list invocation was performed using:
    #    rosnode_listnodes(namespace=namespace, list_uri=options.list_uri, list_all=options.list_all)

    result = rclpy.get_node_names()
    for node in result:
        print(node)


def _fullusage(return_error=True):
    """
    Prints rosnode usage information.
    @param return_error whether to exit with error code os.EX_USAGE
    """
    print("""rosnode is a command-line tool for printing information about ROS Nodes.
Commands:
\trosnode ping\ttest connectivity to node (NOT IMPLEMENTED)
\trosnode list\tlist active nodes
\trosnode info\tprint information about node (NOT IMPLEMENTED)
\trosnode machine\tlist nodes running on a particular machine or list machines
\trosnode kill\tkill a running node (NOT IMPLEMENTED)
\trosnode cleanup\tpurge registration information of unreachable nodes (NOT IMPLEMENTED)
Type rosnode <command> -h for more detailed usage, e.g. 'rosnode ping -h'
""")
    if return_error:
        sys.exit(getattr(os, 'EX_USAGE', 1))
    else:
        sys.exit(0)

def rosnodemain(argv=None):
    """
    Prints rosnode main entrypoint.
    @param argv: override sys.argv
    @param argv: [str]
    """
    if argv == None:
        argv = sys.argv
    if len(argv) == 1:
        _fullusage()
    try:
        command = argv[1]
        if command == 'ping':
            sys.exit(_rosnode_cmd_ping(argv) or 0)
        elif command == 'list':
            sys.exit(_rosnode_cmd_list(argv) or 0)
        elif command == 'info':
            sys.exit(_rosnode_cmd_info(argv) or 0)
        elif command == 'machine':
            sys.exit(_rosnode_cmd_machine(argv) or 0)
        elif command == 'cleanup':
            sys.exit(_rosnode_cmd_cleanup(argv) or 0)
        elif command == 'kill':
            sys.exit(_rosnode_cmd_kill(argv) or 0)
        elif command == '--help':
            _fullusage(False)
        else:
            _fullusage()
    except KeyboardInterrupt:
        pass    
