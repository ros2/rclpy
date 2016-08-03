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

def chatter_callback(msg):
    print('%s' % msg.data)
    print("-------")

class CallbackEcho(object):
    """
    Callback instance that can print callback data in a variety of
    formats. Used for all variants of rostopic echo
    """

    def __init__(self, topic, msg_eval, plot=False, filter_fn=None,
                 echo_clear=False, echo_all_topics=False,
                 offset_time=False, count=None,
                 field_filter_fn=None, fixed_numeric_width=None):
        """
        :param plot: if ``True``, echo in plotting-friendly format, ``bool``
        :param filter_fn: function that evaluates to ``True`` if message is to be echo'd, ``fn(topic, msg)``
        :param echo_all_topics: (optional) if ``True``, echo all messages in bag, ``bool``
        :param offset_time: (optional) if ``True``, display time as offset from current time, ``bool``
        :param count: number of messages to echo, ``None`` for infinite, ``int``
        :param field_filter_fn: filter the fields that are strified for Messages, ``fn(Message)->iter(str)``
        :param fixed_numeric_width: fixed width for numeric values, ``None`` for automatic, ``int``
        """
        if topic and topic[-1] == '/':
            topic = topic[:-1]
        self.topic = topic
        self.msg_eval = msg_eval
        self.plot = plot
        self.filter_fn = filter_fn
        self.fixed_numeric_width = fixed_numeric_width

        self.prefix = ''
        self.suffix = '\n---' if not plot else ''# same as YAML document separator, bug #3291
        
        self.echo_all_topics = echo_all_topics
        self.offset_time = offset_time

        # done tracks when we've exceeded the count
        self.done = False
        self.max_count = count
        self.count = 0

        # determine which strifying function to use
        if plot:
            #TODOXXX: need to pass in filter function
            self.str_fn = _str_plot
            self.sep = ''
        else:
            #TODOXXX: need to pass in filter function
            self.str_fn = self.custom_strify_message
            if echo_clear:
                self.prefix = '\033[2J\033[;H'

        self.field_filter=field_filter_fn
        
        # first tracks whether or not we've printed anything yet. Need this for printing plot fields.
        self.first = True

        # cache
        self.last_topic = None
        self.last_msg_eval = None

    def custom_strify_message(self, val, indent='', time_offset=None, current_time=None, field_filter=None, type_information=None, fixed_numeric_width=None):
        # ensure to print uint8[] as array of numbers instead of string
        if type_information and type_information.startswith('uint8['):
            val = [ord(x) for x in val]
        return genpy.message.strify_message(val, indent=indent, time_offset=time_offset, current_time=current_time, field_filter=field_filter, fixed_numeric_width=fixed_numeric_width)

    def callback(self, data, callback_args, current_time=None):
        """
        Callback to pass to rospy.Subscriber or to call
        manually. rospy.Subscriber constructor must also pass in the
        topic name as an additional arg
        :param data: Message
        :param topic: topic name, ``str``
        :param current_time: override calculation of current time, :class:`genpy.Time`
        """
        topic = callback_args['topic']
        type_information = callback_args.get('type_information', None)
        if self.filter_fn is not None and not self.filter_fn(data):
            return

        if self.max_count is not None and self.count >= self.max_count:
            self.done = True
            return
        
        try:
            msg_eval = self.msg_eval
            if topic == self.topic:
                pass
            elif self.topic.startswith(topic + '/'):
                # self.topic is actually a reference to topic field, generate msgeval
                if topic == self.last_topic:
                    # use cached eval
                    msg_eval = self.last_msg_eval
                else:
                    # generate msg_eval and cache
                    self.last_msg_eval = msg_eval = msgevalgen(self.topic[len(topic):])
                    self.last_topic = topic
            elif not self.echo_all_topics:
                return

            if msg_eval is not None:
                data = msg_eval(data)
                
            # data can be None if msg_eval returns None
            if data is not None:
                # NOTE: we do all prints using direct writes to sys.stdout, which works better with piping
                
                self.count += 1
                
                # print fields header for plot
                if self.plot and self.first:
                    sys.stdout.write("%"+_str_plot_fields(data, 'field', self.field_filter)+'\n')
                    self.first = False

                if self.offset_time:
                    sys.stdout.write(self.prefix+\
                                     self.str_fn(data, time_offset=rospy.get_rostime(),
                                                 current_time=current_time, field_filter=self.field_filter, type_information=type_information, fixed_numeric_width=self.fixed_numeric_width) + \
                                     self.suffix + '\n')
                else:
                    sys.stdout.write(self.prefix+\
                                     self.str_fn(data,
                                                 current_time=current_time, field_filter=self.field_filter, type_information=type_information, fixed_numeric_width=self.fixed_numeric_width) + \
                                     self.suffix + '\n')

                # we have to flush in order before piping to work
                sys.stdout.flush()
            # #2778 : have to check count after incr to set done flag
            if self.max_count is not None and self.count >= self.max_count:
                self.done = True

        except IOError:
            self.done = True
        except:
            # set done flag so we exit
            self.done = True
            traceback.print_exc()

def create_field_filter(echo_nostr, echo_noarr):
    def field_filter(val):
        fields = val.__slots__
        field_types = val._slot_types
        for f, t in zip(val.__slots__, val._slot_types):
            if echo_noarr and '[' in t:
                continue
            elif echo_nostr and 'string' in t:
                continue
            yield f
    return field_filter


def _rostopic_echo(topic, callback_echo, bag_file=None, echo_all_topics=False):
    """
    Print new messages on topic to screen.
    
    :param topic: topic name, ``str``
    :param bag_file: name of bag file to echo messages from or ``None``, ``str``
    """
    # we have to init a node regardless and bag echoing can print timestamps

    if bag_file:
        # TODO review
        # initialize rospy time due to potential timestamp printing
        #rospy.rostime.set_rostime_initialized(True)        
        #_rostopic_echo_bag(callback_echo, bag_file)
        pass
    else:
        rclpy.init(None)
        # Get topic information
        msg_class, real_topic, msg_eval = get_topic_class(topic, blocking=True)
        if msg_class is None:
            # occurs on ctrl-C
            return
        callback_echo.msg_eval = msg_eval

        # extract type information for submessages
        type_information = None
        if len(topic) > len(real_topic):
            subtopic = topic[len(real_topic):]
            subtopic = subtopic.strip('/')
            if subtopic:
                fields = subtopic.split('/')
                submsg_class = msg_class
                while fields:
                    field = fields[0].split('[')[0]
                    del fields[0]
                    index = submsg_class.__slots__.index(field)
                    type_information = submsg_class._slot_types[index]
                    if fields:
                        submsg_class = roslib.message.get_message_class(type_information.split('[', 1)[0])
                        if not submsg_class:
                            raise ROSTopicException("Cannot load message class for [%s]. Are your messages built?" % type_information)

        use_sim_time = rospy.get_param('/use_sim_time', False)
        sub = rospy.Subscriber(real_topic, msg_class, callback_echo.callback, {'topic': topic, 'type_information': type_information})

        if use_sim_time:
            # #2950: print warning if nothing received for two seconds

            timeout_t = time.time() + 2.
            while time.time() < timeout_t and \
                    callback_echo.count == 0 and \
                    not rospy.is_shutdown() and \
                    not callback_echo.done:
                _sleep(0.1)

            if callback_echo.count == 0 and \
                    not rospy.is_shutdown() and \
                    not callback_echo.done:
                sys.stderr.write("WARNING: no messages received and simulated time is active.\nIs /clock being published?\n")

        while not rospy.is_shutdown() and not callback_echo.done:
            _sleep(0.1)    

def _fullusage():
    print("""rostopic is a command-line tool for printing information about ROS Topics.
Commands:
\trostopic bw\tdisplay bandwidth used by topic (NOT IMPLEMENTED)
\trostopic delay\tdisplay delay of topic from timestamp in header (NOT IMPLEMENTED)
\trostopic echo\tprint messages to screen
\trostopic find\tfind topics by type (NOT IMPLEMENTED)
\trostopic hz\tdisplay publishing rate of topic (NOT IMPLEMENTED)
\trostopic info\tprint information about active topic (NOT IMPLEMENTED)
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
