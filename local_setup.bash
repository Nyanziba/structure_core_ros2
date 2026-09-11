#!/usr/bin/env bash

# Workspace-root compatibility shim.
if [ -n "${BASH_SOURCE[0]:-}" ]; then
  _this_file="${BASH_SOURCE[0]}"
elif [ -n "${ZSH_VERSION:-}" ]; then
  _this_file="${(%):-%N}"
else
  _this_file="$0"
fi
_ws_dir="$(cd "$(dirname "${_this_file}")" && pwd)"

if [ -f "${_ws_dir}/install/local_setup.bash" ]; then
  COLCON_CURRENT_PREFIX="${_ws_dir}/install"
  export COLCON_CURRENT_PREFIX
  # shellcheck disable=SC1090
  source "${_ws_dir}/install/local_setup.bash"
  unset COLCON_CURRENT_PREFIX
elif [ -f "${_ws_dir}/install/setup.bash" ]; then
  COLCON_CURRENT_PREFIX="${_ws_dir}/install"
  export COLCON_CURRENT_PREFIX
  # shellcheck disable=SC1090
  source "${_ws_dir}/install/setup.bash"
  unset COLCON_CURRENT_PREFIX
else
  echo "not found: \"${_ws_dir}/install/local_setup.bash\"" 1>&2
  return 1 2>/dev/null || exit 1
fi
