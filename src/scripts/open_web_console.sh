#!/usr/bin/env bash
# Open the existing console; never launch ROS nodes or change robot state.
set -euo pipefail

gpu_mode=default
port=8766
while [[ $# -gt 0 ]]; do
  case "$1" in
    --nvidia) gpu_mode=nvidia; shift ;;
    --port)
      [[ $# -ge 2 ]] || { echo '--port requires a port number' >&2; exit 2; }
      port=$2; shift 2 ;;
    --help|-h)
      echo 'Usage: bash src/scripts/open_web_console.sh [--nvidia] [--port 8766]'
      echo 'Uses a separate local Chrome profile; existing browser windows stay open.'
      exit 0 ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
done
if [[ ! $port =~ ^[0-9]{4,5}$ ]] || (( 10#$port < 1024 || 10#$port > 65535 )); then
  echo 'Port must be 1024..65535' >&2
  exit 2
fi
port=$((10#$port))
browser=$(command -v google-chrome || command -v google-chrome-stable || command -v chromium || command -v chromium-browser || true)
[[ -n $browser ]] || { echo 'Chrome/Chromium is not installed.' >&2; exit 1; }
url="http://127.0.0.1:$port"
if ! curl --fail --silent --max-time 3 "$url/api/bootstrap" >/dev/null; then
  echo "Console is unavailable at $url; start ui.launch.py first." >&2
  exit 1
fi
profile="${XDG_CONFIG_HOME:-$HOME/.config}/ieir-web-console/chrome-$gpu_mode"
args=("--user-data-dir=$profile" --no-first-run --no-default-browser-check)
if [[ $gpu_mode == nvidia ]]; then
  egl_vendor=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
  [[ -r $egl_vendor ]] || { echo 'NVIDIA EGL driver is not installed.' >&2; exit 1; }
  export __NV_PRIME_RENDER_OFFLOAD=1
  export __GLX_VENDOR_LIBRARY_NAME=nvidia
  export __EGL_VENDOR_LIBRARY_FILENAMES="$egl_vendor"
  args+=(--use-gl=angle --use-angle=gl)
fi
echo "Opening $url ($gpu_mode graphics, separate Chrome profile)"
exec "$browser" "${args[@]}" --new-window "$url"
