#!/usr/bin/env bash
set -euo pipefail
package_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
workspace_dir=$(cd "$package_dir/../../.." && pwd)
camera_build_dir=${D405_BUILD_DIR:-"$workspace_dir/.build/d405"}
camera_install_dir=${D405_INSTALL_DIR:-"$workspace_dir/install"}
cmake -S "$package_dir" -B "$camera_build_dir" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$camera_install_dir" -DCMAKE_PREFIX_PATH="$workspace_dir/install"
cmake --build "$camera_build_dir" -j1
cmake --install "$camera_build_dir"
