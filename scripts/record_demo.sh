#!/bin/bash
# record_demo.sh — R05 recording pipeline (NVENC, native-1080p region).
#
# Why this exists (R05 step 1 findings, full numbers in R05 report):
#   * The R04 pipeline (6144x3456 full-screen grab -> CPU scale -> libx264)
#     produced ~4 fps of DISTINCT frames. The encoder was innocent: gzclient
#     itself repaints at ~4-5 fps when its window is 3072x3406 (~10 MP) —
#     repaint rate scales inversely with window area (~45 MP/s budget).
#     glxgears at 400x400 recorded at 55/60 fps through the same pipeline.
#   * Fix = small windows tiling a native 1920x1080 region (no scaling at
#     all) + GPU encode. Measured 26-29 fps effective during navigation.
#   * x11grab records the TOP-MOST pixels of the region: any gap between
#     windows leaks the desktop into the video (R02c/R04 privacy incidents;
#     R05 caught two more during layout bring-up). The `arrange` step must
#     end with the three windows EXACTLY tiling the region, and `check`
#     must be run on every finished recording.
#
# Window layout (client-rect truth via host xwininfo, NOT xdotool's
# getwindowgeometry, which adds phantom frame offsets on this WM):
#   Gazebo             (   0,138) 960x1080   left column
#   RViz (la_demo)     ( 960,138) 960x540    top right
#   Detector Debug Img ( 960,678) 960x540    bottom right
#   capture region     :1.0+0,138 1920x1080
# Y origin 138: the WM refuses to place decorated windows above y~138.
#
# Usage:
#   scripts/record_demo.sh arrange
#   scripts/record_demo.sh record  out.mp4 [max_seconds]   (runs until INT)
#   scripts/record_demo.sh check   video.mp4               (15 s frame scan)
set -u
REPO="$(cd "$(dirname "$0")/.." && pwd)"
DISP=":1"
OX=0; OY=138; CW=1920; CH=1080

true_pos() {  # window-id -> "x y" from host xwininfo (ground truth)
  DISPLAY=$DISP xwininfo -id "$1" 2>/dev/null \
    | awk '/Absolute upper-left X/{x=$NF} /Absolute upper-left Y/{y=$NF} END{print x, y}'
}

arrange() {
  local IDS GZ RV PN
  IDS=$(docker exec docker-sim-1 bash -c '
    export DISPLAY=:1 XAUTHORITY=/tmp/.host_xauth
    GZ=$(xdotool search --class gazebo 2>/dev/null | while read w; do
          [ "$(xdotool getwindowname $w 2>/dev/null)" = Gazebo ] && echo $w && break; done)
    RV=$(xdotool search --name "la_demo.rviz.*RViz$" 2>/dev/null | while read w; do
          case "$(xdotool getwindowname $w 2>/dev/null)" in *"(on "*) ;; *RViz) echo $w; break;; esac; done)
    PN=$(xdotool search --name "^Detector Debug Image$" 2>/dev/null | tail -1)
    echo "$GZ $RV $PN"')
  read -r GZ RV PN <<< "$IDS"
  if [ -z "$GZ" ] || [ -z "$RV" ] || [ -z "$PN" ]; then
    echo "arrange: missing window (GZ=$GZ RV=$RV PN=$PN)"; return 1
  fi
  docker exec docker-sim-1 bash -c "export DISPLAY=:1 XAUTHORITY=/tmp/.host_xauth
    xdotool windowsize $GZ 960 1080; xdotool windowsize $RV 960 540; xdotool windowsize $PN 960 540"
  local spec tgt w tx ty px py sx sy i
  for spec in "$GZ:0:138" "$RV:960:138" "$PN:960:678"; do
    IFS=: read -r w tx ty <<< "$spec"
    sx=$tx; sy=$ty
    for i in 1 2 3 4 5 6; do
      docker exec docker-sim-1 bash -c "export DISPLAY=:1 XAUTHORITY=/tmp/.host_xauth; xdotool windowmove $w $sx $sy"
      sleep 0.4
      read -r px py <<< "$(true_pos "$w")"
      [ "$px" = "$tx" ] && [ "$py" = "$ty" ] && break
      sx=$((sx + tx - px)); sy=$((sy + ty - py))
    done
    echo "window $w at ($px,$py) target ($tx,$ty)"
    [ "$px" = "$tx" ] && [ "$py" = "$ty" ] || { echo "arrange: FAILED to place $w"; return 1; }
  done
  docker exec docker-sim-1 bash -c "export DISPLAY=:1 XAUTHORITY=/tmp/.host_xauth
    xdotool windowraise $GZ; xdotool windowraise $RV; xdotool windowraise $PN"
  echo "arranged; capture region ${CW}x${CH}+${OX},${OY}"
}

record() {
  local out=$1 dur=${2:-1200}
  mkdir -p "$(dirname "$out")"
  ffmpeg -loglevel error -f x11grab -framerate 30 -video_size ${CW}x${CH} \
    -i ${DISP}.0+${OX},${OY} -t "$dur" \
    -pix_fmt yuv420p -c:v h264_nvenc -preset p4 -rc vbr -cq 23 -b:v 0 "$out" &
  FFPID=$!
  echo "recording pid=$FFPID -> $out (INT to stop)"
  sleep 3
  ffmpeg -loglevel error -y -f x11grab -video_size ${CW}x${CH} -i ${DISP}.0+${OX},${OY} \
    -frames:v 1 "${out%.mp4}_startcheck.jpg"
  echo "start-check frame: ${out%.mp4}_startcheck.jpg — EYEBALL IT NOW"
  wait $FFPID
}

check() {  # extract a frame every 15 s for the post-recording privacy scan
  local vid=$1 outdir
  outdir="${vid%.mp4}_scan"
  mkdir -p "$outdir"
  ffmpeg -loglevel error -y -i "$vid" -vf fps=1/15 "$outdir/scan_%03d.jpg"
  echo "frames in $outdir — eyeball every one before publishing"
}

case "${1:-}" in
  arrange) arrange ;;
  record)  shift; record "$@" ;;
  check)   shift; check "$@" ;;
  *) echo "usage: $0 arrange | record out.mp4 [secs] | check video.mp4"; exit 1 ;;
esac
