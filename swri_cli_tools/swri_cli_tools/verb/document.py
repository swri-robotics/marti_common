import sys

from ros2cli.node.strategy import add_arguments
from swri_cli_tools.document import document_system
from swri_cli_tools.verb import VerbExtension
from ros2node.api import NodeNameCompleter


class DocumentVerb(VerbExtension):
    """Document running system"""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        argument = parser.add_argument(
            'node_name',
            help='Node name to request information')
        argument.completer = NodeNameCompleter()
        parser.add_argument(
            '--include-hidden', action='store_true',
            help='Display hidden topics, services, and actions as well')

    def main(self, *, args):
        document_system(args)
