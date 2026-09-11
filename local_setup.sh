#!/usr/bin/env sh

# Workspace-root compatibility shim.
if [ -n "${BASH_SOURCE:-}" ]; then
  _this_file="${BASH_SOURCE[0]}"
elif [ -n "${ZSH_VERSION:-}" ]; then
  _this_file="${(%):-%N}"
else
  _this_file="$0"
fi
_ws_dir="$(CDPATH= cd -- "$(dirname -- "${_this_file}")" && pwd)"

if [ -f "${_ws_dir}/install/local_setup.sh" ]; then
  COLCON_CURRENT_PREFIX="${_ws_dir}/install"
  export COLCON_CURRENT_PREFIX
  . "${_ws_dir}/install/local_setup.sh"
  unset COLCON_CURRENT_PREFIX
elif [ -f "${_ws_dir}/install/setup.sh" ]; then
  COLCON_CURRENT_PREFIX="${_ws_dir}/install"
  export COLCON_CURRENT_PREFIX
  . "${_ws_dir}/install/setup.sh"
  unset COLCON_CURRENT_PREFIX
else
  echo "not found: \"${_ws_dir}/install/local_setup.sh\"" 1>&2
  return 1 2>/dev/null || exit 1
fi
