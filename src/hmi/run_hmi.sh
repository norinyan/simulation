#!/usr/bin/env bash
set -e
HMI_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$HMI_ROOT/install/setup.bash"
exec python3 "$HMI_ROOT/src/hmi/backend/app.py"
