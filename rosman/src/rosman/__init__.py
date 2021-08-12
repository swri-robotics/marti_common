#!/usr/bin/env python

import os
import sys
import socket
import re
import time
import yaml
# Fast zip compatibility with Python 2 and 3
try:
    from itertools import izip as zip
except ImportError:
    pass

from optparse import OptionParser
import rosgraph
import rostopic
import rospy
import genpy
from marti_common_msgs.msg import NodeInfo, TopicInfo, ParamInfo, ServiceInfo

def _check_master(rosmaster):
    try:
        rosmaster.getPid()
    except socket.error:
        # Stealing rostopics exception type for now
        raise rostopic.ROSTopicIOException("Unable to communicate with master!")

class DocTopicReader:
    def __init__(self, rosmaster):
        self.rosmaster = rosmaster
        self.callback_called = False
        self.last_doc_msg = None
        # Setup a node to read topics with
        _check_master(self.rosmaster)
        rospy.init_node('rosman', anonymous=True)

    def doc_topic_callback(self, doc_msg):
        self.last_doc_msg = doc_msg
        self.callback_called = True

    def read_doc_topic(self, topic, timeout_duration=2.0):
        self.callback_called = False
        doc_sub = rospy.Subscriber(topic, NodeInfo, self.doc_topic_callback) 
        timeout = time.time()+timeout_duration
        while time.time() < timeout and self.callback_called == False and \
                not rospy.is_shutdown():
            rospy.rostime.wallsleep(0.1)
        if not self.callback_called:
            self.last_doc_msg = None
        # Let caller know if reading was successful
        return self.callback_called

## TODO make this a document topic reader class that can cache 
## the document messages it reads and do the reading / file output seperatly 
def read_documentation_topic(rosmaster, topic, output_file=sys.stdout):
    topic_reader = DocTopicReader(rosmaster)
    if topic_reader.read_doc_topic(topic):
        output_file.write(genpy.message.strify_message(topic_reader.last_doc_msg) + '\n')

def get_documentation_publications(rosmaster):
    """
    Get all the current documentation topics in the system
    """
    # get the master system state
    try:
        ros_sys_state = rosmaster.getSystemState()
    except socket.error:
        print('Could not communicate with ROS master!')
        sys.exit(2)
    pubs, _, _ = ros_sys_state
    # doc_matcher = re.compile('/documentation$')
    doc_match_string = '/documentation$'
    doc_topics = []
    doc_node_namespaces = []
    doc_publisher_nodes = []
    for t, n in pubs:
        doc_match = re.search(doc_match_string, t)
        if doc_match:
            # Try stripping the node namespace off the topic
            node_namespace = t[0:doc_match.span()[0]]
            doc_topics.append(t)
            doc_node_namespaces.append(node_namespace)
            doc_publisher_nodes.append(n)
            #print('node {0} with node namespace {1} publishes topic {2}'.format(n, node_namespace ,t))
    return doc_topics, doc_node_namespaces, doc_publisher_nodes

def rosman_node(rosmaster, node_name):
    # The doc_node_namespaces are probably the more accurate "node" information for the documentation topic
    # since the doc_publisher nodes for a doc topic can be a nodelet manager
    documentation_info = get_documentation_publications(rosmaster)
    # TODO I don't think I'm iterating through this correctly
    for topic, node_namespace, publishers in zip(documentation_info[0], documentation_info[1], documentation_info[2]):
        if node_name in node_namespace or node_name in publishers:
            # TODO handle or buble up the handling of file opening/closing if the 
            # output is desired from something other than stdout
            read_documentation_topic(rosmaster, topic)

def _rosman_node_main(argv):
    """
    Entry point for rosman node command
    """
    args = argv[2:]
    parser = OptionParser(usage='usage: %prog node node1 [node2...]')
    # parser.add_option(help="Print overview/developer information for a desired node")
    (options, args) = parser.parse_args(args)
    
    if not args:
        parser.error('You must specify at least one node name')

    ros_master = rosgraph.Master('/rosman')
    for node in args:
        rosman_node(ros_master, node)

def _rosman_topics_main(argv):
    """
    Entry point for rosman topics command
    """
    args = argv[2:]
    parser = OptionParser(usage='usage: %prog topics topic1 [topic2...]')
    (options, args) = parser.parse_args(args)

    if not args:
        parser.error('You must specify at least one topic name')
    for node in args:
        print('querying node {n}'.format(n=node))

def _rosman_params_main(argv):
    """
    Entry point for rosman params command
    """
    args = argv[2:]
    parser = OptionParser(usage='usage: %prog params param1 [param2...]')
    (options, args) = parser.parse_args(args)

    if not args:
        parser.error('You must specify at least one param name')
    for node in args:
        print('querying node {n}'.format(n=node))

def _rosman_services_main(argv):
    """
    Entry point for rosman services command
    """
    args = argv[2:]
    parser = OptionParser(usage='usage: %prog services service1 [service2...]')
    (options, args) = parser.parse_args(args)

    if not args:
        parser.error('You must specify at least one service name')
    for node in args:
        print('querying node {n}'.format(n=node))

def _tool_usage(return_error=True):
    """
    Print the full usage information for the rosman tool.
    @param return_error set to true to return from this printout with error code os.EX_USAGE, otherwise exit returning 0.
    """
    print("""rosman is a command-line tool for printing documentation about nodes, topics, and parameters from a live or playback system.

Commands:
\trosman node\tGet overview documentation for a running node
\trosman topics\tGet documentation for a desired topic
\trosman params\tGet documentation for a desired parameter
\trosman services\tGet documentation about a desired service

Type rosman <command> -h for more detailed usage, e.g. 'rosman params -h'
""")
    if return_error:
        sys.exit(getattr(os, 'EX_USAGE', 1))
    else:
        sys.exit(0)

def rosmanmain(argv=None):
    """
    Prints rosman main entrypoint.
    @param argv: override sys.argv
    @param argv: [str]
    """
    if argv == None:
        argv = sys.argv
    if len(argv) == 1:
        _tool_usage()
    try:
        command = argv[1]
        if command == 'node':
            _rosman_node_main(argv)
        elif command == 'topics':
            _rosman_topics_main(argv)
        elif command == 'params':
            _rosman_params_main(argv)
        elif command == 'services':
            _rosman_services_main(argv)
        elif command in ('-h', '--help'):
            _tool_usage(return_error=False)
        else:
            _tool_usage()
    except KeyboardInterrupt:
        pass

