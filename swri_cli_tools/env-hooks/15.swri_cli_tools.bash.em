

@[if DEVELSPACE]@
. "@(CMAKE_CURRENT_SOURCE_DIR)/scripts/swri_cli_tools.bash"
@[else]@
if [ -z "$CATKIN_ENV_HOOK_WORKSPACE" ]; then
  CATKIN_ENV_HOOK_WORKSPACE="@(CMAKE_INSTALL_PREFIX)"
fi
. "$CATKIN_ENV_HOOK_WORKSPACE/share/swri_cli_tools/scripts/swri_cli_tools.bash"
@[end if]@

