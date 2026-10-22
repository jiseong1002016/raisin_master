SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
library_path="${SCRIPT_DIR}/raisim2Lib/raisim/lib:${SCRIPT_DIR}/raisim2Lib/rayrai/lib:${SCRIPT_DIR}/raisim2Lib/raisim/lib/openusd:${SCRIPT_DIR}/install/lib"
IFS=: read -r -a previous_library_dirs <<< "${LD_LIBRARY_PATH:-}"
for library_dir in "${previous_library_dirs[@]}"; do
  case "$library_dir" in
    ""|*/runtime/grasp_294/lib|*/runtime/gui_294/lib|*/runtime/raisim2Lib/*) continue ;;
  esac
  if [[ ":${library_path}:" != *":${library_dir}:"* ]]; then
    library_path="${library_path}:${library_dir}"
  fi
done
export LD_LIBRARY_PATH="$library_path"
unset library_path library_dir previous_library_dirs
