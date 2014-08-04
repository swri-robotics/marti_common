#!/usr/bin/python

'''
Author: Ed Venator (evenator@swri.org)
Script to check when a package (or all packages) were last built. Also parses 
the repository information.

'''

import argparse
import subprocess
import sys
import os
import os.path
import fnmatch

rosmake_log_dir = os.environ['HOME'] + "/.ros/rosmake"
ws_path = os.environ['ROS_WORKSPACE']

def PackageBuildSuccess(build, package):
  """ Returns whether the build of the package was successful.
  
  Returns false if the package was not in the given build or the build failed
  for that package.

  build -- The directory name of the build log. The format is 
            "rosmake_output-yyyymmdd-hhddss"
  package -- The name of the package
  """
  if not os.path.isfile(os.path.join(rosmake_log_dir, build, package, "build_output.log")):
    return False
  build_failure_path = os.path.join(rosmake_log_dir, build, "buildfailures.txt")
  if not os.path.isfile(build_failure_path):
    return True
  with open(build_failure_path) as build_failures:
    for line in build_failures:
      if package in line:
        return False
    return True

def FindBuilds(package, success_only = True):
  """ Returns a list of build log directory names for the package
  
  package -- The package to find builds for
  success_only -- Return successful builds only (default True)
  """
  # os.walk is slow, so doing this manually
  all_builds = os.listdir(rosmake_log_dir)
  all_builds.sort(reverse=True)
  builds = []
  # TODO: filter these to only get correctly formatted dirs
  package_dir = subprocess.call(
            ["rospack", "find", package],
            shell=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE)
  for build in all_builds:
    build_dir = os.path.join(rosmake_log_dir, build)
    if os.path.isfile(os.path.join(rosmake_log_dir, build, package, "build_output.log")):
      # Skip this build if it was a build of another workspace
      with open(os.path.join(build_dir, package, "build_output.log")) as build_output:
        for line in build_output:
          if package_dir in line:
            if not success_only or PackageBuildSuccess(build_dir, package):
              builds.append(build_dir)
  return builds

def FindLastBuild(package, success_only = True):
  """ Returns the last build of the package, or None if it was never built.
  
  package -- The package to find builds for
  success_only -- Return last successful build (default True)
  """
  # os.walk is slow, so doing this manually
  all_builds = os.listdir(rosmake_log_dir)
  all_builds.sort(reverse=True)
  # TODO: filter these to only get correctly formatted dirs
  package_dir = subprocess.check_output(
            ["rospack", "find", package]).strip()
  for build in all_builds:
    build_dir = os.path.join(rosmake_log_dir, build)
    if os.path.isfile(os.path.join(rosmake_log_dir, build, package, "build_output.log")):
      # Skip this build if it was a build of another workspace
      with open(os.path.join(build_dir, package, "build_output.log")) as build_output:
        for line in build_output:
          if package_dir in line:
            if not success_only or PackageBuildSuccess(build_dir, package):
              return build_dir
  return None

def ProcessPackage(package, no_version):
  """ Processes the given package to get information on its last build
  Returns a tuple of (date, revision hash, repository status)
  
  package -- The package to process
  no_version -- If true, skip checking for version information (these will
    be set to None)
  """
  last_build = FindLastBuild(package)
  if not last_build:
    return (None, None, None)
  build_time = last_build.split('-',1)[1]
  if no_version:
    return (build_time, None, None)
  (revision, status) = GetVersionInfo(last_build, package)
  return (build_time, revision, status)

def PrintPackage(package, no_version):
  """ Prints information on the last build of the given package as a table row
  
  package -- The package to process
  no_version -- If true, skip checking for version information
  """
  (build_time, revision, status) = ProcessPackage(package, no_version)
  if not build_time:
    build_time = "never"
  if no_version:
    print package.ljust(32) + build_time
    return
  if not revision:
    revision = "none"
    status = ""
  elif not status:
    status = "clean"
  # Add indents to status
  first_line = True
  indented_status = ""
  for line in status.split("\n"):
    if not first_line:
      indented_status += " " * (32 + 16 + 45) + line + "\n"
    else:
      indented_status += line + "\n"
      first_line = False
  status = indented_status.rstrip()
  print package.ljust(32) + build_time.ljust(16) + revision.ljust(45) + status

def GetVersionInfo(build, package):
  """ Gets repository information from the build log for the given build and 
  package.

  build -- The directory name of the build log. The format is 
            "rosmake_output-yyyymmdd-hhddss"
  package -- The name of the package
  """
  build
  build_path = os.path.join(rosmake_log_dir, build, package)
  build_log_path = os.path.join(build_path, "build_output.log")
  revision = ""
  status = ""
  with  open(build_log_path) as build_log:
    line = build_log.readline()
    while len(line):
      if line[0:13] == "-- Revision: ":
        revision = line[13:].rstrip()
        line = build_log.readline()
      elif "-- Repository Status:" in line:
        line = build_log.readline()
        while line[0] != "-":
          status += line
          line = build_log.readline()
        status = status.rstrip()
      else:
        line = build_log.readline()
  return (revision, status)

def main():
  parser = argparse.ArgumentParser("Find out when packages were last built");
  parser.add_argument("-a", "--all", dest="all", action="store_true",
                      help="List all packages (even outside of the current workspace)")
  parser.add_argument("--no-version", dest="no_version", action="store_true",
                      help="Don't check for repository version information")
  parser.add_argument("packages", metavar="PACKAGE", type=str, nargs='*',
                      help="Package(s) to check. (Overridden by ALL). If none are specified, all packages in the workspace are checked")
  options = parser.parse_args()
  if options.no_version:
    print "Package".ljust(32) + "Last Build".ljust(16)
    print "-" * (32 + 16)
  else:
    print "Package".ljust(32) + "Last Build".ljust(16) + "Version".ljust(45) + "Status"
    print "-" * (32 + 16 + 45 + 32)
  if options.all or not options.packages:
    # Get a list of all packages
    package_cmd = subprocess.Popen(
      ["rospack", "list"],
      shell=False,
      stdout=subprocess.PIPE,
      stderr=subprocess.PIPE)
    package_line = package_cmd.stdout.readline()
    while len(package_line):
      # Print only if all is set or this package is in the rosws
      if options.all or (ws_path in package_line.split(' ')[1]):
        PrintPackage(package_line.split(' ')[0], options.no_version)
      package_line = package_cmd.stdout.readline()
  else:
    for package in options.packages:
      # Check that package exists
      if subprocess.call(
            ["rospack", "find", package],
            shell=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE):
        print >> sys.stderr, "[ERROR] Package " + package + " does not exist"
      else:
        PrintPackage(package, options.no_version)
  sys.exit(0)

if __name__ == '__main__':
  main()
