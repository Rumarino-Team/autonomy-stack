#!/usr/bin/env bash
set -euo pipefail

# Thin wrapper around the installed Kalibr Docker image.
# Usage examples:
#   ./tools/kalibr/run_kalibr.sh kalibr_calibrate_cameras --bag /data/my.bag ...
#   ./tools/kalibr/run_kalibr.sh kalibr_calibrate_imu_camera --bag /data/my.bag ...

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
IMAGE="stereolabs/kalibr:latest"

# X11 forwarding is optional. It is useful for target detection visualization.
XSOCK="/tmp/.X11-unix"
XAUTH="/tmp/.docker.xauth"

if [[ ! -f "${XAUTH}" ]]; then
  touch "${XAUTH}"
  xauth nlist "${DISPLAY:-:0}" | sed -e 's/^..../ffff/' | xauth -f "${XAUTH}" nmerge - >/dev/null 2>&1 || true
fi

docker run --rm -it \
  --net=host \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY="${XAUTH}" \
  -v "${XSOCK}:${XSOCK}:rw" \
  -v "${XAUTH}:${XAUTH}:rw" \
  -v "${ROOT_DIR}:/work:rw" \
  -w /work \
  "${IMAGE}" "$@"
