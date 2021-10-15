

@[if DEVELSPACE]@
. "@(CMAKE_CURRENT_SOURCE_DIR)/scripts/rosman.bash"
@[else]@
if [ -z "$CATKIN_ENV_HOOK_WORKSPACE" ]; then
  CATKIN_ENV_HOOK_WORKSPACE="@(CMAKE_INSTALL_PREFIX)"
fi
. "$CATKIN_ENV_HOOK_WORKSPACE/share/rosman/scripts/rosman.bash"
@[end if]@

