#!/usr/bin/env bash
set -euo pipefail

usage() {
  printf 'Usage: %s <hardware-profile> [real|nonreal]\n' "$0" >&2
  exit 2
}

[[ $# -ge 1 && $# -le 2 ]] || usage
profile=$1
mode=${2:-nonreal}
case "$mode" in
  real) mode_args=(real) ;;
  nonreal) mode_args=() ;;
  *) usage ;;
esac

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
project_root=$(cd -- "$script_dir/.." && pwd)
runtime_prefix=${RAISIN_NODE_PREFIX:-"$project_root/install"}
node="$runtime_prefix/bin/raisin_raipal_node"
[[ -x "$node" ]] || {
  printf 'RAIPAL node is not executable: %s\n' "$node" >&2
  exit 1
}

export LD_LIBRARY_PATH="$runtime_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$node" "${mode_args[@]}" --profile "$profile"
