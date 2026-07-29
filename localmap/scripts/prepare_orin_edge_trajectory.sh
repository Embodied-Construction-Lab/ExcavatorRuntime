#!/usr/bin/env bash
# PC-only planning entry: creates no policy-action sender and performs no SSH copy.
set -eo pipefail

AIRY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

source /opt/ros/jazzy/setup.bash
if [[ -f "${AIRY_ROOT}/ros2_ws/install/setup.bash" ]]; then
  source "${AIRY_ROOT}/ros2_ws/install/setup.bash"
fi

exec /usr/bin/python3 \
  "${AIRY_ROOT}/localmap/apps/planning/prepare_orin_edge_trajectory.py" \
  "$@"
