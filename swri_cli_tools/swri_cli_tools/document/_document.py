from natsort import natsorted


from swri_cli_tools.api._node_info import NodeInfo
from swri_cli_tools.api._node_info import print_node_infos

from ros2cli.node.strategy import NodeStrategy

from ros2node.api import get_absolute_node_name
from ros2node.api import parse_node_name
from ros2node.api import get_action_client_info
from ros2node.api import get_action_server_info
from ros2node.api import get_node_names
from ros2node.api import get_publisher_info
from ros2node.api import get_service_client_info
from ros2node.api import get_service_server_info
from ros2node.api import get_subscriber_info


def document_system(args):
    """Document a running system."""
    nodes = {}
    with NodeStrategy(args) as node:
        names = get_node_names(node=node, include_hidden_nodes=args.hidden)
        names = natsorted(get_absolute_node_name(name.full_name) for name in names)

        for name in names:
            subscribers = get_subscriber_info(
                node=node,
                remote_node_name=name,
                include_hidden=args.hidden)
            publishers = get_publisher_info(
                node=node,
                remote_node_name=name,
                include_hidden=args.hidden)
            service_servers = get_service_server_info(
                node=node,
                remote_node_name=name,
                include_hidden=args.hidden)
            service_clients = get_service_client_info(
                node=node,
                remote_node_name=name,
                include_hidden=args.hidden)
            action_servers = get_action_server_info(
                node=node,
                remote_node_name=name,
                include_hidden=args.hidden)
            action_clients = get_action_client_info(
                node=node,
                remote_node_name=name,
                include_hidden=args.hidden)

            nodes[name] = NodeInfo(name=name,
                publishers=publishers,
                subscribers=subscribers,
                service_servers=service_servers,
                service_clients=service_clients,
                action_servers=action_servers,
                action_clients=action_clients)
        
    print_node_infos(nodes)
