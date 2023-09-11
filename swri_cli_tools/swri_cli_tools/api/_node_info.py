# Copyright (c) 2023, Southwest Research Institute® (SwRI®)
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Southwest Research Institute® (SwRI®) nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from typing import List

from rcl_interfaces.msg import ParameterType
from ros2node.api import TopicInfo

from swri_cli_tools.api._node_info import NodeInfo
from swri_cli_tools.api._node_info import ParameterInfo


def print_node_infos(nodes: List(NodeInfo)):
    """Print information about nodes."""
    for node in nodes:
        print_node_info(node)


def print_node_info(node: NodeInfo):
    """Print information about single node."""
    print('- {}:'.format(node.name))
    print('    - Publishers:')
    for p in node.publishers:
        print('        - {}:'.format(p.name))
        for t in p.types:
            print('            {}'.format(t))
    print('    - Subscribers:')
    for s in node.subscribers:
        print('        - {}:'.format(s.name))
        for t in s.types:
            print('            {}'.format(t))
    print('    - Service Servers:')
    for s in node.service_servers:
        print('        - {}:'.format(s.name))
        for t in s.types:
            print('            {}'.format(t))
    print('    - Service Clients:')
    for c in node.service_clients:
        print('        - {}:'.format(c.name))
        for t in c.types:
            print('            {}'.format(t))
    print('    - Action Servers:')
    for s in node.action_servers:
        print('        - {}:'.format(s.name))
        for t in s.types:
            print('            {}'.format(t))
    print('    - Action Clients:')
    for c in node.action_clients:
        print('        - {}:'.format(c.name))
        for t in c.types:
            print('            {}'.format(t))
    print('    - Parameters:')
    for p in node.parameters:
        print('        - {}:'.format(p.param_name))
        print('            - {}'.format(p.param_type))
        print('            - {}'.format(p.param_value))


class NodeInfo:
    """Information about node for documentation."""

    def __init__(self,
                 name: str = None,
                 publishers: List(TopicInfo) = [],
                 subscribers: List(TopicInfo) = [],
                 service_servers: List(TopicInfo) = [],
                 service_clients: List(TopicInfo) = [],
                 action_servers: List(TopicInfo) = [],
                 action_clients: List(TopicInfo) = [],
                 parameters: List(ParameterInfo) = []) -> None:
        """Construct node information class."""
        self._name = name
        self._publishers = publishers
        self._subscribers = subscribers
        self._service_servers = service_servers
        self._service_clients = service_clients
        self._action_servers = action_servers
        self._action_clients = action_clients
        self._parameters = parameters

    @property
    def name(self):
        """Get name of node."""
        return self._name

    @name.setter
    def name(self, value) -> None:
        """Set node name."""
        self._name = value

    def add_publisher(self, connection: TopicInfo):
        """Add publisher connection information to node."""
        self._publishers.append(connection)

    def add_publishers(self, connections: List(TopicInfo)):
        """Add publisher connection information to node."""
        self._publishers += connections

    @property
    def publishers(self):
        """Get all publishers associated with node."""
        return self._publishers

    def add_subscriber(self, connection: TopicInfo):
        """Add subscriber connection information to node."""
        self._subscribers.append(connection)

    def add_subscribers(self, connection: TopicInfo):
        """Add subscriber connection information to node."""
        self._subscribers.append(connection)

    @property
    def subscribers(self):
        """Get all subscribers associated with node."""
        return self._subscribers

    def add_service_server(self, connection: TopicInfo):
        """Add service server information to node."""
        self._service_servers.append(connection)

    def add_service_servers(self, connections: List(TopicInfo)):
        """Add service server information to node."""
        self._service_servers += connections

    @property
    def service_servers(self):
        """Get all service servers associated with node."""
        return self._service_servers

    def add_service_client(self, connection: TopicInfo):
        """Add service client information to node."""
        self._service_clients.append(connection)

    def add_service_clients(self, connections: List(TopicInfo)):
        """Add service client information to node."""
        self._service_clients += connections

    @property
    def service_clients(self):
        """Get all service clients associated with node."""
        return self._service_clients

    def add_action_server(self, connection: TopicInfo):
        """Add action server information to node."""
        self._action_servers.append(connection)

    @property
    def action_servers(self):
        """Get all action servers associated with node."""
        return self._action_servers

    def add_action_client(self, connection: TopicInfo):
        """Add action client information to node."""
        self._action_clients.append(connection)

    def add_action_clients(self, connections: List(TopicInfo)):
        """Add action client information to node."""
        self._action_clients += connections

    @property
    def action_clients(self):
        """Get all action clients associated with node."""
        return self._action_clients

    def add_parameter(self, parameter):
        """Add parameter to node."""
        self._parameters.append(parameter)

    def add_parameters(self, parameters):
        """Add parameters to node."""
        self._parameters += parameters

    @property
    def parameters(self):
        """Get parameters associated with node."""
        return self._parameters


class ParameterInfo:
    """Hold information about node parameter."""

    def __init__(self,
                 param_name: str = None,
                 param_type: ParameterType = None,
                 param_value: str = None) -> None:

        self._param_name = param_name
        self._param_type = "Unknown"
        if param_type == ParameterType.PARAMETER_BOOL:
            self._param_type = "boolean"
        elif param_type == ParameterType.PARAMETER_INTEGER:
            self._param_type = "integer"
        elif param_type == ParameterType.PARAMETER_DOUBLE:
            self._param_type = "double"
        elif param_type == ParameterType.PARAMETER_STRING:
            self._param_type = "string"
        elif param_type == ParameterType.PARAMETER_BYTE_ARRAY:
            self._param_type = "byte array"
        elif param_type == ParameterType.PARAMETER_BOOL_ARRAY:
            self._param_type = "bool array"
        elif param_type == ParameterType.PARAMETER_INTEGER_ARRAY:
            self._param_type = "integer array"
        elif param_type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            self._param_type = "double array"
        elif param_type == ParameterType.PARAMETER_STRING_ARRAY:
            self._param_type = "string array"
        elif param_type == ParameterType.PARAMETER_NOT_SET:
            self._param_type = "Not set"

        self._param_value = param_value

    @property
    def param_name(self):
        """Get parameter name."""
        return self._param_name

    @property
    def param_type(self):
        """Get parameter type."""
        return self._param_type

    @property
    def param_value(self):
        """Get parameter value."""
        return self._param_value
