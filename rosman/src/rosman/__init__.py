#!/usr/bin/env python

import os
import sys

from optparse import OptionParser

def _rosman_node_main(argv):
    """
    Entry point for rosman node command
    """
    args = argv[2:]
    parser = OptionParser(usage='usage: %prog node node1 [node2...]')
    # parser.add_option(help="Print overview/developer information for a desired node")
    (options, args) = parser.parse_args(args)

    if not args:
        parser.error('You must specify at least one node name')
    for node in args:
        print('querying node {n}'.format(n=node))

def _rosman_topics_main(argv):
    """
    Entry point for rosman topics command
    """
    args = argv[2:]
    parser = OptionParser(usage='usage: %prog topics node1 [node2...]')
    (options, args) = parser.parse_args(args)

    if not args:
        parser.error('You must specify at least one node name')
    for node in args:
        print('querying node {n}'.format(n=node))

def _rosman_params_main(argv):
    """
    Entry point for rosman params command
    """
    args = argv[2:]
    parser = OptionParser(usage='usage: %prog params node1 [node2...]')
    (options, args) = parser.parse_args(args)

    if not args:
        parser.error('You must specify at least one node name')
    for node in args:
        print('querying node {n}'.format(n=node))

def _rosman_services_main(argv):
    """
    Entry point for rosman services command
    """
    args = argv[2:]
    parser = OptionParser(usage='usage: %prog services node1 [node2...]')
    (options, args) = parser.parse_args(args)

    if not args:
        parser.error('You must specify at least one node name')
    for node in args:
        print('querying node {n}'.format(n=node))

def _tool_usage(return_error=True):
    """
    Print the full usage information for the rosman tool.
    @param return_error set to true to return from this printout with error code os.EX_USAGE, otherwise exit returning 0.
    """
    print("""rosman is a command-line tool for printing documentation about nodes, topics, and parameters from a live or playback system.

Commands:
\trosman node\tGet overview documentation about a running node
\trosman topics\tGet documentation about the topics published by a node
\trosman params\tGet documentation about the parameters used by a node
\trosman services\tGet documentation about the services provided by this node

Type rosman <command> -h for more detailed usage, e.g. 'rosman params -h'
""")
    if return_error:
        sys.exit(getattr(os, 'EX_USAGE', 1))
    else:
        sys.exit(0)

def rosmanmain(argv=None):
    """
    Prints rosman main entrypoint.
    @param argv: override sys.argv
    @param argv: [str]
    """
    if argv == None:
        argv = sys.argv
    if len(argv) == 1:
        _tool_usage()
    try:
        command = argv[1]
        if command == 'node':
            _rosman_node_main(argv)
        elif command == 'topics':
            _rosman_topics_main(argv)
        elif command == 'params':
            _rosman_params_main(argv)
        elif command == 'services':
            _rosman_services_main(argv)
        elif command in ('-h', '--help'):
            _tool_usage(return_error=False)
        else:
            _tool_usage()
    except KeyboardInterrupt:
        pass

