"""Holds information about node in ROS 2 system."""

from __future__ import annotations
from typing import List

from ros2node.api import TopicInfo


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
        print('            {}:'.format(p.types))
    print('    - Subscribers:')
    for s in node.subscribers:
        print('        - {}:'.format(s.name))
        print('            {}:'.format(s.types))
    print('    - Service Servers:')
    for s in node.service_servers:
        print('        - {}:'.format(s.name))
        print('            {}:'.format(s.types))
    for c in node.service_clients:
        print('        - {}:'.format(c.name))
        print('            {}:'.format(c.types))
    for s in node.action_servers:
        print('        - {}:'.format(s.name))
        print('            {}:'.format(s.types))
    for c in node.action_clients:
        print('        - {}:'.format(c.name))
        print('            {}:'.format(c.types))

class NodeInfo:
    """Information about node for documentation."""

    def __init__(self,
        name: str = None,
        publishers: List(TopicInfo) = [],
        subscribers: List(TopicInfo) = [],
        service_servers: List(TopicInfo) = [],
        service_clients: List(TopicInfo) = [],
        action_servers: List(TopicInfo) = [],
        action_clients: List(TopicInfo) = []) -> None:
        """Construct node information class."""
        self._name = name
        self._publishers = publishers
        self._subscribers = subscribers
        self._service_servers = service_servers
        self._service_clients = service_clients
        self._action_servers = action_servers
        self._action_clients = action_clients

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
