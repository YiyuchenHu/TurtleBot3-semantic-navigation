#!/usr/bin/env bash
#
# Container entrypoint: bring up the virtual desktop (Xtigervnc + openbox) and the
# noVNC bridge, then hand over to CMD (`sleep infinity` by default — the
# container just stays up so you can `docker compose exec tb3 bash` into it).
#
# Tunables (set them in docker-compose.yml → environment):
#   DISPLAY        X display to create            (default :1)
#   VNC_GEOMETRY   initial desktop size           (default 1600x900; the
#                  browser can resize it later via noVNC "remote resizing")
#   NOVNC_PORT     port noVNC listens on inside the container (default 6080)
set -e

: "${DISPLAY:=:1}"
: "${VNC_GEOMETRY:=1600x900}"
: "${NOVNC_PORT:=6080}"
export DISPLAY

mkdir -p -m 700 "${XDG_RUNTIME_DIR:-/tmp/runtime-root}"

# Xtigervnc is an X server with the VNC server built in (TigerVNC). It only
# accepts connections from inside the container (-localhost); the browser
# reaches it through websockify below. No VNC password: port 6080 is
# published on 127.0.0.1 only (see docker-compose.yml).
Xtigervnc "$DISPLAY" \
    -geometry "$VNC_GEOMETRY" -depth 24 \
    -rfbport 5901 -localhost -SecurityTypes None -AlwaysShared \
    -desktop "tb3-course" &

# Wait for the X socket before starting clients.
sock="/tmp/.X11-unix/X${DISPLAY#:}"
for _ in $(seq 1 100); do
    [ -S "$sock" ] && break
    sleep 0.1
done
if [ ! -S "$sock" ]; then
    echo "entrypoint: Xtigervnc did not create $sock" >&2
    exit 1
fi

# A neutral grey root window, so an empty desktop is visibly "connected"
# rather than a black page (Xvnc's default root is black).
command -v xsetroot >/dev/null && xsetroot -solid '#4a4f55' || true
openbox &

# websockify serves the noVNC web client and bridges ws://:6080 -> vnc://:5901
websockify --web=/usr/share/novnc "$NOVNC_PORT" "localhost:5901" &

echo "tb3-course: virtual desktop $DISPLAY (${VNC_GEOMETRY}) is up."
echo "tb3-course: open http://localhost:${NOVNC_PORT}/ in a browser on the host."

exec "$@"
