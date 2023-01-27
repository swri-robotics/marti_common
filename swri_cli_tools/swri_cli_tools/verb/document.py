import sys

from ros2cli.node.strategy import add_arguments as add_strategy_node_arguments
from swri_cli_tools.document import document_system
from swri_cli_tools.verb import VerbExtension


class DocumentVerb(VerbExtension):
    """Document running system"""

    def add_arguments(self, parser, cli_name):
        add_strategy_node_arguments(parser)
        parser.add_argument(
            '--hidden',
            action='store_true',
            dest='hidden',
            required=False,
            help='Include hidden node and topic information')
        parser.add_argument(
            '--node', '-n',
            dest='nodes',
            required=False,
            default=None,
            nargs='+',
            help='Set of nodes to document. If unused, all nodes will be documented')

    def main(self, *, args):
        document_system(args)
