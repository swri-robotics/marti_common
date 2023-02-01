from ros2cli.command import add_subparsers_on_demand
from ros2cli.command import CommandExtension


class SwriCommand(CommandExtension):
    """Execute SwRI CLI command"""

    def add_arguments(self, parser, cli_name):
        """Add arguments"""
        self._subparser = parser
        add_subparsers_on_demand(
            parser,
            cli_name,
            '_verb',
            'swri_cli_tools.verb',
            required=False)

    def main(self, *, parser, args):
        """ Create command """
        if not hasattr(args, '_verb'):
            self._subparser.print_help()
            return 0

        extension = getattr(args, '_verb')

        return extension.main(args=args)
