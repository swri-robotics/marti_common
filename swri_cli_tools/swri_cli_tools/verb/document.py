import sys

try:
    from argcomplete.completers import DirectoriesCompleter
except ImportError:
    def DirectoriesCompleter():
        return None

import swri_cli_tools.document
from swri_cli_tools.verb import VerbExtension

class DocumentVerb(VerbExtension):
    """Document running system"""

    def add_arguments(self, parser, cli_name):
        pass

    def main(self, *, args) -> int:
        try:
            swri_cli_tools.document.document_system()
        except swri_cli_tools.errors.SWRIError as e:
            print(f'Unable to document system: {str(e)}', file=sys.stderr)
            return 1
        return 0
