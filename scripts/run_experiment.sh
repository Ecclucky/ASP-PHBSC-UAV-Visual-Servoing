#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
case_name="${1:-case1}"
if [[ $# -gt 0 ]]; then shift; fi
case "$case_name" in
  case1|case2) ;;
  *) printf 'Usage: bash scripts/run_experiment.sh case1|case2 [roslaunch arguments]\n' >&2; exit 2 ;;
esac
if [[ ! -f "$workspace_dir/devel/setup.bash" ]]; then
  printf 'Build this workspace with catkin_make first: %s\n' "$workspace_dir" >&2
  exit 1
fi
# Catkin-generated setup scripts can reference unset variables.
set +u
source "$workspace_dir/devel/setup.bash"
set -u
exec roslaunch asp_phbsc experiment.launch case:="$case_name" "$@"
