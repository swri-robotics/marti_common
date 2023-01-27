from natsort import natsorted

from swri_cli_tools.api._node_info import NodeInfo
from swri_cli_tools.api._node_info import ParameterInfo
from swri_cli_tools.api._node_info import print_node_infos

import rclpy

from rcl_interfaces.srv import ListParameters

from ros2cli.node.strategy import NodeStrategy
from ros2cli.node.direct import DirectNode

from ros2node.api import get_absolute_node_name
from ros2node.api import get_action_client_info
from ros2node.api import get_action_server_info
from ros2node.api import get_node_names
from ros2node.api import get_publisher_info
from ros2node.api import get_service_client_info
from ros2node.api import get_service_server_info
from ros2node.api import get_subscriber_info

from ros2param.api import call_get_parameters
from ros2param.api import get_value


def document_system(args):
    """Document a running system."""
    nodes = {}
    names = ()
    with NodeStrategy(args) as node:
        if args.nodes is None:
            names = get_node_names(node=node, include_hidden_nodes=args.hidden)
            names = natsorted(get_absolute_node_name(name.full_name)
                              for name in names)
        else:
            names = natsorted(args.nodes)

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

    with DirectNode(args) as node:
        for target_node in names:
            service_name = f'{target_node}/list_parameters'
            client = node.create_client(ListParameters, service_name)

            client.wait_for_service()

            if not client.service_is_ready():
                nodes[target_node].parameters = None
                continue

            request = ListParameters.Request()
            future = client.call_async(request)

            rclpy.spin_until_future_complete(node, future)

            response = future.result()
            if response is None:
                nodes[target_node].parameters = None
            else:
                for param_name in natsorted(response.result.names):
                    parameter = call_get_parameters(
                        node=node,
                        node_name=target_node,
                        parameter_names=[param_name])
                    value = None
                    if parameter.values:
                        value = get_value(parameter_value=parameter.values[0])

                    nodes[target_node].parameters.append(ParameterInfo(
                        param_name,
                        parameter.values[0].type,
                        str(value)))

            node.destroy_client(client)
    print_node_infos(list(nodes.values()))
