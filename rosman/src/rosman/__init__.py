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
from marti_introspection_msgs.msg import NodeInfo, TopicInfo, ParamInfo, ServiceInfo

DOCUMENTATION_TOPIC_MATCHER = re.compile("/documentation$")

def _check_master(rosmaster):
    try:
        rosmaster.getPid()
    except socket.error:
        # Stealing rostopics exception type for now
        raise rostopic.ROSTopicIOException("Unable to communicate with master!")

def _sort_fn(e):
    return e.name

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

    def write_node_documentation(self, output_file=sys.stdout):
        # TODO error out correctly if the last topic wasn't read correctly
        if (self.last_doc_msg is None):
            print('DocTopicReader failed to read documentation topic and cannot write out the node documentation')
            return

        # sort things
        self.last_doc_msg.topics.sort(key=_sort_fn)
        self.last_doc_msg.parameters.sort(key=_sort_fn)
        self.last_doc_msg.services.sort(key=_sort_fn)

        # output the overall header
        self.write_node_header_documentation(output_file)

        # divide by grouping then print for each group, starting with empty
        groups = {}
        for item in self.last_doc_msg.topics:
            if item.group not in groups:
                groups[item.group] = NodeInfo()
            groups[item.group].topics.append(item)

        for item in self.last_doc_msg.parameters:
            if item.group not in groups:
                groups[item.group] = NodeInfo()
            groups[item.group].parameters.append(item)

        for item in self.last_doc_msg.services:
            if item.group not in groups:
                groups[item.group] = NodeInfo()
            groups[item.group].services.append(item)

        # output the empty group first
        self.write_node_subscriptions_documentation(output_file, groups[""])
        self.write_node_publishers_documentation(output_file, groups[""])
        self.write_node_parameters_documentation(output_file, groups[""])
        self.write_node_services_documentation(output_file, groups[""])
        
        for group in groups:
            if group == "":
                continue

            output_file.write("\n\nGroup: {name}\n\n".format(name=group))

            self.write_node_subscriptions_documentation(output_file, groups[group])
            self.write_node_publishers_documentation(output_file, groups[group])
            self.write_node_parameters_documentation(output_file, groups[group])
            self.write_node_services_documentation(output_file, groups[group])


    def write_node_header_documentation(self, output_file=sys.stdout):
        output_file.write("{name} - ({nodelet_manager})\n{description}\n\n".format(name=self.last_doc_msg.name,
            nodelet_manager=self.last_doc_msg.nodelet_manager,
            description=self.last_doc_msg.description if self.last_doc_msg.description else "TODO: node description"))

    def write_node_subscriptions_documentation(self, output_file, data):
        output_file.write("Subscriptions:\n")
        subs = [topic for topic in data.topics if topic.advertised == False] 
        for sub in subs:
            output_file.write('  * ')
            self.write_topic_info_docstring(sub, output_file)
        output_file.write('\n')

    def write_topic_info_docstring(self, topic_info_msg, output_file=sys.stdout):
        output_file.write("{name} - ({type}) - {description}\n".format(name=topic_info_msg.name,
            type=topic_info_msg.message_type,
            description=topic_info_msg.description))

    def write_node_publishers_documentation(self, output_file, data):
        output_file.write("Publishers:\n")
        pubs = [topic for topic in data.topics if topic.advertised == True]
        for pub in pubs:
            output_file.write('  * ')
            self.write_topic_info_docstring(pub, output_file)
        output_file.write('\n')

    def write_node_parameters_documentation(self, output_file, data):
        if len(self.last_doc_msg.parameters) > 0:
            output_file.write("Parameters:\n")
            for param in data.parameters:
                output_file.write('  * ')
                self.write_param_info_docstring(param, output_file)
            output_file.write('\n')

    def write_node_services_documentation(self, output_file, data):
        servs = [service for service in data.services if service.server == True]
        if len(servs) > 0:
            output_file.write("Service Servers:\n")
            for serv in servs:
                output_file.write('  * ')
                self.write_service_info_docstring(serv, output_file=sys.stdout)
            output_file.write('\n')

        servs = [service for service in data.services if service.server == False]
        if len(servs) > 0:
            output_file.write("Service Clients:\n")
            for serv in servs:
                output_file.write('  * ')
                self.write_service_info_docstring(serv, output_file=sys.stdout)
            output_file.write('\n')

    def write_param_info_docstring(self, param_info_msg, output_file=sys.stdout):
        default_val = ""
        type_str = "unknown_type"
        if (param_info_msg.type == ParamInfo.TYPE_DOUBLE):
            default_val = param_info_msg.default_double
            type_str = "double"
            if param_info_msg.max_value != param_info_msg.min_value:
                output_file.write("{name} - ({type}, {min:.6g} <= {default:.6g} <= {max:.6g}) - {description}\n".format(name=param_info_msg.name,
                type=type_str, default=default_val, description=param_info_msg.description, max=param_info_msg.max_value, min=param_info_msg.min_value))
            else:
                output_file.write("{name} - ({type}, {default:.6g}) - {description}\n".format(name=param_info_msg.name,
                type=type_str, default=default_val, description=param_info_msg.description))
            return
        elif (param_info_msg.type == ParamInfo.TYPE_STRING):
            default_val = param_info_msg.default_string
            type_str = "string"
            output_file.write("{name} - ({type}, {default}) - {description}\n".format(name=param_info_msg.name,
                type=type_str, default=default_val, description=param_info_msg.description))
            return
        elif (param_info_msg.type == ParamInfo.TYPE_INT):
            default_val = param_info_msg.default_int
            type_str = "int"
            output_file.write("{name} - ({type}, {default:d}) - {description}\n".format(name=param_info_msg.name,
                type=type_str, default=default_val, description=param_info_msg.description))
            return
        elif (param_info_msg.type == ParamInfo.TYPE_FLOAT):
            default_val = param_info_msg.default_float
            type_str = "float"
            if param_info_msg.max_value != param_info_msg.min_value:
                output_file.write("{name} - ({type}, {min:.6g} <= {default:.6g} <= {max:.6g}) - {description}\n".format(name=param_info_msg.name,
                type=type_str, default=default_val, description=param_info_msg.description, max=param_info_msg.max_value, min=param_info_msg.min_value))
            else:
                output_file.write("{name} - ({type}, {default:.6g}) - {description}\n".format(name=param_info_msg.name,
                type=type_str, default=default_val, description=param_info_msg.description))
            return
        elif (param_info_msg.type == ParamInfo.TYPE_BOOL):
            default_val = "true" if param_info_msg.default_bool else "false"
            type_str = "bool"
            output_file.write("{name} - ({type}, {default}) - {description}\n".format(name=param_info_msg.name,
                type=type_str, default=default_val, description=param_info_msg.description))
            return
        elif (param_info_msg.type == ParamInfo.TYPE_XMLRPC):
            type_str = "XMLRPC"
            output_file.write("{name} - ({type}) - {description}\n".format(name=param_info_msg.name,
                type=type_str, description=param_info_msg.description))
            return
        # Unknown type write
        output_file.write("{name} - ({type}, {default}) - {description}\n".format(name=param_info_msg.name,
            type=type_str, default=default_val, description=param_info_msg.description))


    def write_service_info_docstring(self, service_info_msg, output_file=sys.stdout):
        output_file.write("{name} - ({type}) - {description}\n".format(name=service_info_msg.name,
            type=service_info_msg.message_type, description=service_info_msg.description))

    def get_doc_msg_topic(self, input_topic):
        if self.last_doc_msg is None:
            return False
        for doc_topic in self.last_doc_msg.topics:
            if doc_topic.resolved_name == input_topic:
                return doc_topic
        return False

    def get_doc_msg_param(self, input_param):
        if self.last_doc_msg is None:
            return False
        for doc_param in self.last_doc_msg.parameters:
            if doc_param.resolved_name == input_param:
                return doc_param
        return False

    def get_doc_msg_service(self, input_service):
        if self.last_doc_msg is None:
            return False
        for doc_service in self.last_doc_msg.services:
            if doc_service.resolved_name == input_service:
                return doc_service
        return False

def read_documentation_topic(rosmaster, topic, yaml=False, output_file=sys.stdout):
    topic_reader = DocTopicReader(rosmaster)
    if topic_reader.read_doc_topic(topic):
        if yaml:
            output_file.write(genpy.message.strify_message(topic_reader.last_doc_msg) + '\n')
        else:
            topic_reader.write_node_documentation(output_file)

def get_documentation_publications(rosmaster, ros_sys_state=None):
    """
    Get all the current documentation topics in the system
    """
    # get the master system state if not passed in by caller
    if ros_sys_state is None:
        try:
            ros_sys_state = rosmaster.getSystemState()
        except socket.error:
            print('Could not communicate with ROS master!')
            sys.exit(2)
    pubs, _, _ = ros_sys_state
    doc_topics = []
    doc_node_namespaces = []
    doc_publisher_nodes = []
    for t, n in pubs:
        doc_match = DOCUMENTATION_TOPIC_MATCHER.search(t)
        if doc_match:
            # Try stripping the node namespace off the topic
            node_namespace = t[0:doc_match.span()[0]]
            doc_topics.append(t)
            doc_node_namespaces.append(node_namespace)
            doc_publisher_nodes.append(n)
            #print('node {0} with node namespace {1} publishes topic {2}'.format(n, node_namespace ,t))
    return doc_topics, doc_node_namespaces, doc_publisher_nodes

def rosman_node(rosmaster, node_name, yaml=False, output_file=sys.stdout):
    # The doc_node_namespaces are probably the more accurate "node" information for the documentation topic
    # since the doc_publisher nodes for a doc topic can be a nodelet manager
    documentation_info = get_documentation_publications(rosmaster)
    node_documentation_found = False
    for topic, node_namespace, publishers in zip(documentation_info[0], documentation_info[1], documentation_info[2]):
        if node_namespace in node_name or node_name in publishers:
            read_documentation_topic(rosmaster, topic, yaml=yaml, output_file=output_file)
            node_documentation_found = True
    return node_documentation_found

def rosman_node_fallback(rosmaster, node_name, yaml=False, output_file=sys.stdout):
    # TODO figure out if we need to support other options in fallback?
    # if yaml:
    #     print('YAML output not supported for rosman node fallback documentation!')
    #     yaml = False
    if output_file is not sys.stdout:
        print('Output other than stdout not support for rosman node fallback documentation!')
        output_file = sys.stdout

    def topic_type(t, pub_topics):
        matches = [t_type for t_name, t_type in pub_topics if t_name == t]
        if matches:
            return matches[0]
        return 'unknown type'

    def param_type(param_value):
        if isinstance(param_value, float):
            # Can't really distinguish between float and double
            return ParamInfo.TYPE_DOUBLE
        elif isinstance(param_value, str):
            return ParamInfo.TYPE_STRING
        elif isinstance(param_value, int):
            return ParamInfo.TYPE_INT
        elif isinstance(param_value, bool):
            return ParamInfo.TYPE_BOOL
        else:
            # Reasonable failure fallback?
            return ParamInfo.TYPE_STRING 
    try:
        ros_sys_state = rosmaster.getSystemState()
        pub_topics = rosmaster.getPublishedTopics('/')
        # param_list_success, _, param_list = rosmaster.getParamNames()
        param_list = rosmaster.getParamNames()
    except socket.error:
        print('Could not communicate with ROS master!')
        sys.exit(2)
    pubs = sorted([t for t, l in ros_sys_state[0] if node_name in l])
    subs = sorted([t for t, l in ros_sys_state[1] if node_name in l])
    srvs = sorted([t for t, l in ros_sys_state[2] if node_name in l])
    # This will only retrieve private parameters for this node, but better than nothing
    params = sorted([p for p in param_list if p.startswith(node_name)])

    # Fill out a shim node documentation and hack the DocTopicReader to write output
    doc = NodeInfo()
    doc.name = node_name
    doc.description = 'Warning: no documentation topic is published for {n}. Falling back to standard rosnode info.'.format(n=node_name)
    for pub in pubs:
        topic_doc = TopicInfo()
        topic_doc.name = pub
        topic_doc.resolved_name = pub
        topic_doc.message_type = topic_type(pub, pub_topics)
        topic_doc.advertised = True
        doc.topics.append(topic_doc)
    for sub in subs:
        topic_doc = TopicInfo()
        topic_doc.name = sub
        topic_doc.resolved_name = sub
        topic_doc.message_type = topic_type(sub, pub_topics)
        topic_doc.advertised = False
        doc.topics.append(topic_doc)
    for param in params:
        param_doc = ParamInfo()
        param_doc.name = param.replace(node_name, '')
        if param_doc.name[0] == '/':
            param_doc.name = param_doc.name[1:]
        param_doc.resolved_name = param
        param_val = rosmaster.getParam(param)
        param_doc.type = param_type(param_val)
        if param_doc.type == ParamInfo.TYPE_DOUBLE:
            param_doc.default_double = float(param_val)
        elif param_doc.type == ParamInfo.TYPE_STRING:
            param_doc.default_string = str(param_val)
        elif param_doc.type == ParamInfo.TYPE_INT:
            param_doc.default_int = int(param_val)
        elif param_doc.type == ParamInfo.TYPE_BOOL:
            param_doc.default_bool = bool(param_val)
        doc.parameters.append(param_doc)
    for srv in srvs:
        if not (srv.endswith('get_loggers') or srv.endswith('set_logger_level')):
            srv_doc = ServiceInfo()
            srv_doc.name = srv
            srv_doc.resolved_name = srv
            # TODO get service type info
            srv_doc.message_type = 'unknown type'
            srv_doc.server = True
            doc.services.append(srv_doc)
    topic_reader = DocTopicReader(rosmaster)
    topic_reader.last_doc_msg = doc
    if yaml:
        output_file.write(genpy.message.strify_message(doc) + '\n')
    else:
        topic_reader.write_node_documentation(output_file)


def rosman_topic(rosmaster, topic):
    # Find the desired topic in the list of publishers from the system state
    try:
        ros_sys_state = rosmaster.getSystemState()
    except socket.error:
        print('Could not communicate with ROS master!')
        sys.exit(2)
    pubs, subs, _ = ros_sys_state
    topic_documentation_found = False
    for t, publisher_nodes in pubs:
        if t == topic:
            for node in publisher_nodes:
                # print('Found topic: {t} published by node: {n}'.format(t=t, n=node))
                doc_info = get_documentation_publications(rosmaster, ros_sys_state)
                for doc_topic, doc_pub_namespace, doc_pubs in zip(doc_info[0], doc_info[1], doc_info[2]):
                    if node in doc_pub_namespace or node in doc_pubs:
                        # print('Found {node} in namespace {ns} or doc publishers {dp}'.format(node=node, ns=doc_pub_namespace, dp=doc_pubs))
                        topic_reader = DocTopicReader(rosmaster)
                        if topic_reader.read_doc_topic(doc_topic):
                            topic_doc = topic_reader.get_doc_msg_topic(topic)
                            if topic_doc:
                                topic_reader.write_node_header_documentation()
                                topic_reader.write_topic_info_docstring(topic_doc)
                                topic_documentation_found = True

    # Find subscribers if there are no publishers
    for t, subscriber_nodes in subs:
        if t == topic:
            for node in subscriber_nodes:
                # print('Found topic: {t} published by node: {n}'.format(t=t, n=node))
                doc_info = get_documentation_publications(rosmaster, ros_sys_state)
                for doc_topic, doc_pub_namespace, doc_pubs in zip(doc_info[0], doc_info[1], doc_info[2]):
                    if node in doc_pub_namespace or node in doc_pubs:
                        # print('Found {node} in namespace {ns} or doc publishers {dp}'.format(node=node, ns=doc_pub_namespace, dp=doc_pubs))
                        topic_reader = DocTopicReader(rosmaster)
                        if topic_reader.read_doc_topic(doc_topic):
                            topic_doc = topic_reader.get_doc_msg_topic(topic)
                            if topic_doc and not topic_documentation_found:
                                topic_reader.write_node_header_documentation()
                                topic_reader.write_topic_info_docstring(topic_doc)
                                topic_documentation_found = True

    if topic_documentation_found == False:
        print('Could not find published documentation for topic: {t}'.format(t=topic))

def rosman_param(rosmaster, param):
   param_documentation_found = False
   doc_topics, doc_node_namespaces, doc_publishers = get_documentation_publications(rosmaster)
   topic_reader = DocTopicReader(rosmaster)
   for doc_topic in doc_topics:
        if topic_reader.read_doc_topic(doc_topic):
            param_doc = topic_reader.get_doc_msg_param(param)
            if param_doc:
                topic_reader.write_node_header_documentation()
                topic_reader.write_param_info_docstring(param_doc)
                param_documentation_found = True
   if param_documentation_found == False:
       print('Could not find published documentation for parameter: {p}'.format(p=param))

def rosman_service(rosmaster, service):
    service_documentation_found = False
    doc_topics, _, _ = get_documentation_publications(rosmaster)
    topic_reader = DocTopicReader(rosmaster)
    for doc_topic in doc_topics:
        if topic_reader.read_doc_topic(doc_topic):
            service_doc = topic_reader.get_doc_msg_service(service)
            if service_doc and service_doc.server:
                topic_reader.write_node_header_documentation()
                topic_reader.write_service_info_docstring(service_doc)
                service_documentation_found = True

    # If still not found, check clients
    for doc_topic in doc_topics:
        if topic_reader.read_doc_topic(doc_topic):
            service_doc = topic_reader.get_doc_msg_service(service)
            if service_doc and not service_doc.server and not service_documentation_found:
                topic_reader.write_node_header_documentation()
                topic_reader.write_service_info_docstring(service_doc)
                service_documentation_found = True
    if service_documentation_found == False:
        print('Could not find published documentation for service: {s}'.format(s=service))

def _rosman_node_main(argv):
    """
    Entry point for rosman node command
    """
    args = argv[2:]
    parser = OptionParser(usage='usage: %prog node node1 [node2...]')
    parser.add_option('-y','--yaml', dest="yaml", action="store_true", 
            default=False, help='print node documentation output as a yaml compliant string')
    parser.add_option('-f', '--filename', dest="filename", action="store",
            metavar="FILE", help="write output to FILE. If the file exists, the output will be appended, otherwise a new file with the name FILE will be created.")
    (options, args) = parser.parse_args(args)
    
    if not args:
        parser.error('You must specify at least one node name')

    ros_master = rosgraph.Master('/rosman')
    if options.filename:
        with open(options.filename, 'a') as output_file:
            for node in args:
                if not rosman_node(ros_master, node, yaml=options.yaml, output_file=output_file):
                    rosman_node_fallback(ros_master, node, yaml=options.yaml, output_file=output_file)
    else:
        # Write all to stdout
        for node in args:
            if not rosman_node(ros_master, node, yaml=options.yaml):
                rosman_node_fallback(ros_master, node, yaml=options.yaml)

def _rosman_check_main(argv):
    """
    Entry point for rosman check command
    """
    args = argv[2:]
    parser = OptionParser(usage='usage: %prog node node1 [node2...]')
    parser.add_option('-y','--yaml', dest="yaml", action="store_true", 
            default=False, help='print node documentation output as a yaml compliant string')
    parser.add_option('-f', '--filename', dest="filename", action="store",
            metavar="FILE", help="write output to FILE. If the file exists, the output will be appended, otherwise a new file with the name FILE will be created.")
    parser.add_option("-v", action="store_true", dest="reverse")
    (options, args) = parser.parse_args(args)

    if not args:
        parser.error('You must specify at least one node name')

    ros_master = rosgraph.Master('/rosman')

    # Write all to stdout
    for node in args:
        compare_param(ros_master, node, yaml=options.yaml, reverse=options.reverse)

def compare_param(rosmaster, node_name, yaml=False, output_file=sys.stdout, reverse=False):
    doc_server_array = []
    param_server_array = []
    unused_param = []

    documentation_info = get_documentation_publications(rosmaster)
    print("\n--------------------------------------------------------------------------------\nNode [" + node_name + "]")
    for topic, node_namespace, publishers in zip(documentation_info[0], documentation_info[1], documentation_info[2]):
        if node_namespace in node_name or node_name in publishers:
            topic_reader = DocTopicReader(rosmaster)
            if topic_reader.read_doc_topic(topic):
                # TODO error out correctly if the last topic wasn't read correctly
                if (topic_reader.last_doc_msg is None):
                    print('DocTopicReader failed to read documentation topic and cannot write out the node documentation')
                    return
                # sort things
                topic_reader.last_doc_msg.parameters.sort(key=_sort_fn)
                # divide by grouping then print for each group, starting with empty
                groups = {}

                for item in topic_reader.last_doc_msg.parameters:
                    if item.group not in groups:
                        groups[item.group] = NodeInfo()
                    groups[item.group].parameters.append(item)

                data = groups[""]
                if len(topic_reader.last_doc_msg.parameters) > 0:
                    #print("DOCUMENTED:\n")
                    for param in data.parameters:
                        #print(param.resolved_name)
                        doc_server_array.append(param.resolved_name)

                for group in groups:
                    if group == "":
                        continue
                    data = groups[group]
                    if len(topic_reader.last_doc_msg.parameters) > 0:
                        for param in data.parameters:
                            #print(param.resolved_name)
                            doc_server_array.append(param.resolved_name)
    if not doc_server_array:
        print("Warning: no documentation topic is published for " + node_name + " Falling back to standard rosnode info.\n")
        return
    
    try:
        ros_sys_state = rosmaster.getSystemState()
        param_list = rosmaster.getParamNames() #array
    except socket.error:
        print('Could not communicate with ROS master!')
        sys.exit(2)

    # This will only retrieve private parameters for this node, but better than nothing
    params = sorted([p for p in param_list if p.startswith(node_name)])

    # Fill out a shim node documentation and hack the DocTopicReader to write output
    doc = NodeInfo()
    for param in params:
        param_doc = ParamInfo()
        param_doc.name = param
        param_server_array.append(param_doc.name)

    for param in param_server_array:
        found = 0
        for doc in doc_server_array:
            for doc_param in data.parameters:
                if doc_param.type == 0:
                    if doc in param:
                        found = 1
            if doc.find(param) > -1:
                found = 1
        if found == 0:
            unused_param.append(param)

    print("Set but undocumented params\n")
    for param in unused_param:
        print("  " + param)

    if reverse == True:
        undocumented_param = []
        for doc in doc_server_array:
            found = 0
            for param in param_server_array:
                    if param.find(doc) > -1:
                        found = 1
            if found == 0:
                undocumented_param.append(doc)

        print("\nDocumented but not set params\n")
        for param in undocumented_param:
            print("  " + param)

def _rosman_topic_main(argv):
    """
    Entry point for rosman topics command
    """
    args = argv[2:]
    parser = OptionParser(usage='usage: %prog topics topic1 [topic2...]')
    (options, args) = parser.parse_args(args)
    
    rosmaster = rosgraph.Master('/rosman')
    if not args:
        parser.error('You must specify at least one topic name')
    for topic in args:
        rosman_topic(rosmaster, topic)

def _rosman_param_main(argv):
    """
    Entry point for rosman params command
    """
    args = argv[2:]
    parser = OptionParser(usage='usage: %prog params param1 [param2...]')
    (options, args) = parser.parse_args(args)

    rosmaster = rosgraph.Master('/rosman')
    if not args:
        parser.error('You must specify at least one param name')
    for param in args:
        rosman_param(rosmaster, param)

def _rosman_service_main(argv):
    """
    Entry point for rosman services command
    """
    args = argv[2:]
    parser = OptionParser(usage='usage: %prog services service1 [service2...]')
    (options, args) = parser.parse_args(args)

    rosmaster = rosgraph.Master('/rosman')
    if not args:
        parser.error('You must specify at least one service name')
    for serv in args:
        rosman_service(rosmaster, serv)

def _tool_usage(return_error=True):
    """
    Print the full usage information for the rosman tool.
    @param return_error set to true to return from this printout with error code os.EX_USAGE, otherwise exit returning 0.
    """
    print("""rosman is a command-line tool for printing documentation about nodes, topics, and parameters from a live or playback system.

Commands:
\trosman node\tGet overview documentation for a running node
\trosman topic\tGet documentation for a desired topic
\trosman param\tGet documentation for a desired parameter
\trosman service\tGet documentation about a desired service

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
        elif command == 'check':
            _rosman_check_main(argv)
        elif command == 'topic':
            _rosman_topic_main(argv)
        elif command == 'param':
            _rosman_param_main(argv)
        elif command == 'service':
            _rosman_service_main(argv)
        elif command in ('-h', '--help'):
            _tool_usage(return_error=False)
        else:
            _tool_usage()
    except KeyboardInterrupt:
        pass

