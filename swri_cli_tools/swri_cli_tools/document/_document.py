from natsort import natsorted


from swri_cli_tools.api._node_info import NodeInfo

from ros2cli.node.strategy import NodeStrategy

from ros2node.api import get_action_client_info
from ros2node.api import get_action_server_info
from ros2node.api import get_node_names
from ros2node.api import get_publisher_info
from ros2node.api import get_service_client_info
from ros2node.api import get_service_server_info
from ros2node.api import get_subscriber_info


def document_system(args):
    """Document a running system."""
    with NodeStrategy(args) as node:
        names = get_node_names(node=node, include_hidden_nodes=args.hidden)
        names = natsorted(name.full_name for name in names)

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
            actions_servers = get_action_server_info(
                node=node,
                remote_node_name=name,
                include_hidden=args.hidden)
            actions_clients = get_action_client_info(
                node=node,
                remote_node_name=name,
                include_hidden=args.hidden)

            new_node = NodeInfo()
            for s in subscribers:
                c = ConnectionInfo()
                new_node.add_subscriber(c)

