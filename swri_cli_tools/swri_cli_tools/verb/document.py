import sys

import swri_cli_tools.document
from swri_cli_tools.verb import VerbExtension
from ros2node.api import NodeNameCompleter


class DocumentVerb(VerbExtension):
    """Document running system"""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            '-a', '--all', action='store_true',
            help='Display all nodes even hidden ones')

    def main(self, *, args):
        print('DJA: Verb success!')
