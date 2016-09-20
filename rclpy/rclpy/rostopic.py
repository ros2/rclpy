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
import socket
from optparse import OptionParser
from rclpy.qos import qos_profile_default
import itertools
import importlib
from rclpy.impl.rmw_implementation_tools import select_rmw_implementation, reload_rmw_implementations

NAME='rostopic'

# TODO implement
def _rostopic_cmd_hz(argv):
    print("NOT IMPLEMENTED\n")
    sys.exit(0)

# TODO implement
def _rostopic_cmd_type(argv):
    args = argv[2:]
    from optparse import OptionParser
    parser = OptionParser(usage="usage: %prog type /topic", prog=NAME)
    (options, args) = parser.parse_args(args)
    if len(args) < 1:
        parser.error("provide at least one topic")

    target_topic = args[0]
    target_type = ''
    result = rclpy.get_remote_topic_names_and_types()
    for i in range(len(result[0])):
        topic_name = result[0][i]
        topic_type = result[1][i]
        if target_topic == topic_name:
            target_type = topic_type
    if target_type != '':
        print(target_type)
    sys.exit(0)

def _rostopic_cmd_list(argv):
    result = rclpy.get_remote_topic_names_and_types()
    for i in range(len(result[0])):
        topic_name = result[0][i]
        topic_type = result[1][i]
        print(topic_name)
        #print(topic_name+" (type: "+topic_type+")")

# TODO implement
def _rostopic_cmd_info(argv):
    print("NOT IMPLEMENTED\n")
    sys.exit(0)

# TODO implement
def _rostopic_cmd_pub(argv):
    print("NOT IMPLEMENTED\n")
    sys.exit(0)

# TODO implement
def _rostopic_cmd_bw(argv):
    print("NOT IMPLEMENTED\n")
    sys.exit(0)

# TODO implement
def _rostopic_cmd_find(argv):
    print("NOT IMPLEMENTED\n")
    sys.exit(0)

# TODO implement
def _rostopic_cmd_delay(argv):
    print("NOT IMPLEMENTED\n")
    sys.exit(0)

def _rostopic_cmd_echo(argv):
    def expr_eval(expr):
        def eval_fn(m):
            return eval(expr)
        return eval_fn

    args = argv[2:]
    from optparse import OptionParser
    parser = OptionParser(usage="usage: %prog echo [options] /topic", prog=NAME)
    parser.add_option("-b", "--bag",
                      dest="bag", default=None,
                      help="echo messages from .bag file", metavar="BAGFILE")
    parser.add_option("-p", 
                      dest="plot", default=False,
                      action="store_true",
                      help="echo in a plotting friendly format")
    parser.add_option("-w",
                      dest="fixed_numeric_width", default=None, metavar="NUM_WIDTH",
                      help="fixed width for numeric values")
    parser.add_option("--filter", 
                      dest="filter_expr", default=None,
                      metavar="FILTER-EXPRESSION",
                      help="Python expression to filter messages that are printed. Expression can use Python builtins as well as m (the message) and topic (the topic name).")
    parser.add_option("--nostr", 
                      dest="nostr", default=False,
                      action="store_true",
                      help="exclude string fields")
    parser.add_option("--noarr",
                      dest="noarr", default=False,
                      action="store_true",
                      help="exclude arrays")
    parser.add_option("-c", "--clear",
                      dest="clear", default=False,
                      action="store_true",
                      help="clear screen before printing next message")
    parser.add_option("-a", "--all",
                      dest="all_topics", default=False,
                      action="store_true",
                      help="display all message in bag, only valid with -b option")
    parser.add_option("-n", 
                      dest="msg_count", default=None, metavar="COUNT",
                      help="number of messages to echo")
    parser.add_option("--offset",
                      dest="offset_time", default=False,
                      action="store_true",
                      help="display time as offsets from current time (in seconds)")

    (options, args) = parser.parse_args(args)
    if len(args) > 1:
        parser.error("you may only specify one input topic")
    if options.all_topics and not options.bag:
        parser.error("Display all option is only valid when echoing from bag files")
    if options.offset_time and options.bag:
        parser.error("offset time option is not valid with bag files")
    if options.all_topics:
        topic = ''
    else:
        if len(args) == 0:
            parser.error("topic must be specified")        

        topic = args[0]
        print("rostopic: topic is [%s]"%topic)

    filter_fn = None
    if options.filter_expr:
        filter_fn = expr_eval(options.filter_expr)

    try:
        msg_count = int(options.msg_count) if options.msg_count else None
    except ValueError:
        parser.error("COUNT must be an integer")

    try:
        fixed_numeric_width = int(options.fixed_numeric_width) if options.fixed_numeric_width else None
        if fixed_numeric_width is not None and fixed_numeric_width < 2:
            parser.error("Fixed width for numeric values must be at least 2")
    except ValueError:
        parser.error("NUM_WIDTH must be an integer")

    field_filter_fn = create_field_filter(options.nostr, options.noarr)
    callback_echo = CallbackEcho(topic, None, plot=options.plot,
                                 filter_fn=filter_fn,
                                 echo_clear=options.clear, echo_all_topics=options.all_topics,
                                 offset_time=options.offset_time, count=msg_count,
                                 field_filter_fn=field_filter_fn, fixed_numeric_width=fixed_numeric_width)
    try:
        # TODO FIXME, hardcoded to a function that solely supports simple string messages    
        #_rostopic_echo(topic, callback_echo, bag_file=options.bag)        
        _rostopic_echo_test(topic)
    except socket.error:
        sys.stderr.write("Network communication failed.\n")

def _rostopic_echo_test(topic):
    #rclpy.init(None)
    node = rclpy.create_node('rostopic')
    # TODO FIXME no path resolving for topics yet implemented thereby the initial "/" is
    # omited.
    from std_msgs.msg import String
    sub = node.create_subscription(String, topic[1:], chatter_callback, qos_profile_default)
    while rclpy.ok():
       rclpy.spin_once(node)

module= None
class_ = None

def _convert_getattr(val, f, t): 
    attr = getattr(val, f) 
    if type(attr) is (str) and 'uint8[' in t: 
        return [ord(x) for x in attr] 
    else: 
        return attr 

def strify_message(val, indent=''): 
    global module
    global class_
    type_ = type(val) 
    if type_ in (int, float, bool): 
      return str(val) 
    elif type_ is (str): 
      #TODO: need to escape strings correctly 
      if not val: 
          return "''" 
      return val 

    elif type_ in (list, tuple): 
      if len(val) == 0: 
          return "[]" 
      val0 = val[0] 
      if type(val0) in (int, float, str, bool): 
          # TODO: escape strings properly 
          return str(list(val)) 
      else: 
          pref = indent + '- ' 
          indent = indent + '  ' 
          return '\n'+'\n'.join([pref+strify_message(v, indent) for v in val]) 
    elif isinstance(val, class_): 
        fields = val.__slots__ 

        type_list = [];
        for a in val.__slots__:
            try:
                type_list.append(val.__getattribute__(a))
            except:
                type_list.append(str)

        p = '%s%%s: %%s'%(indent) 
        ni = '  '+indent 
        vals = '\n'.join([p%(f, 
                           strify_message(_convert_getattr(val, f, t), ni)) for f,t in zip(val.__slots__, type_list) if f in fields]) 
        if indent: 
          return '\n'+vals 
        else: 
          return vals 

    else: 
      return str(val) #pun

def chatter_callback(msg):
    print('------------------')
    print(strify_message(msg))
    print('------------------')

def _rostopic_cmd_echo(argv):
    global module
    global class_

    topic_name_sub = ""

    if len(argv)==3:
        topic_name_sub = argv[2]
        select_rmw_implementation("rmw_fastrtps_cpp")
        rclpy.init(argv)
        node = rclpy.create_node('rostopic_echo')
        target_topic = argv[2]
        target_type = ''
        result = rclpy.get_remote_topic_names_and_types()
        for i in range(len(result[0])):
            topic_name = result[0][i]
            topic_type = result[1][i]
            if target_topic == topic_name:
                target_type = topic_type
                
        message_split = target_type.split('/')
        module = importlib.import_module(message_split[0]+'.msg')
        class_ = getattr(module, message_split[1])
        rclpy.impl.rmw_implementation_tools.__rmw_implementation_module = None
        rclpy.impl.rmw_implementation_tools.__rmw_implementations = None
        rclpy.impl.rmw_implementation_tools.__selected_rmw_implementation = None
        rclpy.shutdown();

        select_rmw_implementation("rmw_opensplice_cpp")
        rclpy.init(argv)
        node = rclpy.create_node('rostopic_echo')
    else:
        topic_name_sub = argv[4]
        # from args[1] import args[2]
        module = importlib.import_module(argv[2])
        class_ = getattr(module, argv[3])
        select_rmw_implementation("rmw_opensplice_cpp")
        rclpy.init(argv)
        node = rclpy.create_node('rostopic_echo')

    sub = node.create_subscription(class_,
                                topic_name_sub,
                                chatter_callback,
                                qos_profile_default)
    assert sub  # prevent unused warning

    while rclpy.ok():
        rclpy.spin_once(node)

def _fullusage():
    print("""rostopic is a command-line tool for printing information about ROS Topics.
Commands:
\trostopic bw\tdisplay bandwidth used by topic
\trostopic delay\tdisplay delay of topic from timestamp in header
\trostopic echo module message topic\tprint messages to screen
\trostopic find\tfind topics by type
\trostopic hz\tdisplay publishing rate of topic    
\trostopic info\tprint information about active topic
\trostopic list\tlist active topics
\trostopic pub\tpublish data to topic (NOT IMPLEMENTED)
\trostopic type\tprint topic type
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
            select_rmw_implementation("rmw_fastrtps_cpp")
            rclpy.init(argv)
            _rostopic_cmd_type(argv)
        elif command == 'list':
            select_rmw_implementation("rmw_fastrtps_cpp")
            rclpy.init(argv)
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
