# Setting up on macOS (Docker)

This page gets you to a running course container with the Gazebo and RViz
windows showing in a browser tab. Everything after that — building the
workspace and the six-terminal flow — is identical to Ubuntu and is written
down once, in the [README](../README.md); this page only covers what the
container changes.

It works on Apple Silicon (M1–M4) and on Intel Macs. The image is
`linux/amd64`: Gazebo Classic 11 has no arm64 packages for Ubuntu 22.04, so
neither do ROS Humble's `gazebo-ros-pkgs` or `turtlebot3-gazebo`. Apple
Silicon therefore runs the container through Docker Desktop's Rosetta
translation. Everything works, just slower than on a native Ubuntu machine —
see [What to expect](#what-to-expect-on-apple-silicon) before you judge the
robot's driving.

## 1. Docker Desktop

1. Install [Docker Desktop for Mac](https://docs.docker.com/desktop/setup/install/mac-install/)
   and start it once.
2. **Settings → General** — on Apple Silicon, both of these must be ticked:
   **Use Virtualization framework** and **Use Rosetta for x86_64/amd64
   emulation on Apple Silicon**. Without Rosetta, Docker falls back to QEMU,
   which is several times slower and unreliable with Gazebo.
3. **Settings → Resources** — give the Docker VM at least **6 CPUs and
   8 GB of memory**; more helps, because Gazebo, SLAM, Nav2 and the detector
   all share it. The default 60 GB virtual disk is plenty.
4. **Apply & restart**, then check from a terminal:

```bash
docker version                                          # Client and Server both answer
docker run --rm --platform linux/amd64 alpine uname -m  # prints x86_64
```

## 2. Get the code

Clone straight into your home directory — the same place the README
assumes — and **not** into Desktop or Documents:

```bash
cd ~
git clone -b yolo26n-finetuned https://github.com/YiyuchenHu/TurtleBot3-semantic-navigation.git
cd TurtleBot3-semantic-navigation
```

Why not Desktop/Documents: if iCloud's *Desktop & Documents Folders* sync is
on (it is by default on many Macs), iCloud uploads every one of the
thousands of files `colcon build` writes into `build/` and `install/`, slows
the container's view of the checkout, and resurrects deleted build folders
as `setup 2.bash`-style copies. Any folder outside those two is fine
(Docker Desktop shares all of `/Users` with containers). An existing clone
can simply be moved: `mv ~/Desktop/TurtleBot3-semantic-navigation ~/`.

The detector weights ship with the repository (README → Setup by platform →
step 3): there is nothing to download.

## 3. Build the image — once

```bash
cd docker
docker compose build
```

This takes roughly 10–20 minutes and downloads about 3 GB: ROS 2 Humble
desktop, Gazebo 11, Nav2, SLAM Toolbox and the CPU build of torch — the
exact apt and pip lists from README → Setup by platform steps 1–2, plus a
virtual desktop that you will open in the browser. Rebuild only when
something in [`docker/`](../docker/) changes (it is quick after the first
time: downloads are cached); the image does **not** contain the repository,
so editing code never requires a rebuild.

## 4. Start the container and open the desktop

```bash
docker compose up -d
docker compose ps          # STATUS reaches "healthy" after ~10 s
```

Now open **<http://localhost:6080/>** in a browser. You get an empty grey
desktop: that is the virtual screen the Gazebo and RViz windows will appear
on once you launch them. The browser window's size becomes the desktop's
resolution, so resize the browser window and the desktop follows.

- The desktop is only reachable from this Mac (the port is bound to
  `127.0.0.1`), which is why it has no password.
- Keep the tab open while you work. If it ever shows *Disconnected* (for
  example after `docker compose restart`), reload the page.

## 5. Open terminals

Every "terminal" in the README is a shell inside the container. Open a tab in
Terminal or iTerm on the Mac and run:

```bash
cd TurtleBot3-semantic-navigation/docker
docker compose exec tb3 bash
```

The shell starts in `~/TurtleBot3-semantic-navigation` — your checkout,
bind-mounted live — with `/opt/ros/humble/setup.bash` sourced,
`install/setup.bash` sourced as soon as it exists, and `TURTLEBOT3_MODEL`
set. The four preparation lines the README puts at the top of every terminal
are still correct; running them again is harmless.

Six tabs are the README's six terminals. A `ros2 launch` in any of them puts
its windows on the browser desktop. Edit code with your usual Mac editor —
the container sees `src/` live, nothing is copied in or out.

(From the repository root, `docker compose -f docker/docker-compose.yml exec tb3 bash`
does the same without the `cd`.)

## 6. Continue in the README

- **Build the workspace:** README → Setup by platform → **step 4, Build**, run
  inside a container shell. Steps 1–3 are already done by the image, and the
  conda note does not apply.
- **Run the stack:** README → **Running**, T1
  to T6 in order, each in its own `docker compose exec tb3 bash` tab.
  [docs/running-reference.md](running-reference.md) and
  [docs/troubleshooting.md](troubleshooting.md) apply unchanged in the container.
- [INSTRUCTIONS.md](../INSTRUCTIONS.md) applies unchanged; its Step 1 apt/pip
  part is what the image already did.

## Day to day

| Want to … | Run in `docker/` |
|---|---|
| Stop the container (keeps the checkout and its `build/`) | `docker compose down` |
| Start it again | `docker compose up -d` |
| Restart it — the fastest "clean restart" (kills every ROS process) | `docker compose restart` |
| See what the desktop plumbing is doing | `docker compose logs` |
| Start with a bigger initial desktop | `VNC_GEOMETRY=1920x1080 docker compose up -d` |
| Throw away the build artefacts | inside a container shell: `rm -rf build install log` |
| Remove the image entirely | `docker compose down --rmi all` |

## What to expect on Apple Silicon

The container is x86-64 code translated by Rosetta, with software OpenGL
(there is no GPU inside the VM), so everything runs slower than on a native
Ubuntu machine. Reference run on an M3 Max with 8 CPUs and 16 GB given to
Docker Desktop (macOS 15.1, Docker Desktop 4.37), `world:=warehouse_models`:

| | Measured in the container |
|---|---|
| `docker compose build` | ~17 min, of which the apt step 14 min |
| `colcon build --symlink-install` (8 packages) | 27 s |
| T1 spawn → T2 Nav2 active → T3 exploration → T4 → T5 | 4 s → 6 s → 25 s → 2 s → 4 s |
| Gazebo real-time factor | 0.45 |
| Camera frames reaching the detector | 1–4 per second wall-clock (30 Hz nominal); ~4 with the Gazebo window closed, see below |
| Detector, one 640×480 frame | 55 ms (the README's reference laptop: 32 ms) |

What that means in practice:

- **The robot drives at less than half speed** (real-time factor), so the
  README's "exploration takes 2–8 minutes" becomes roughly 5–20 minutes,
  and every "ready when …" time stretches a little.
- **Landmarks appear later.** The semantic map memory wants 8–12
  observations of the same candidate within 45 s before it promotes it, and
  at ~4 camera frames per second that takes a few seconds of the object
  staying in view instead of a fraction of a second. A `go to …` sent early
  fails with `no active <target> in memory` more often than on Ubuntu: wait
  for the marker to show up in RViz, then send the command.
- **Behaviour is otherwise unchanged.** In the reference run the
  six-terminal flow, exploration, landmark promotion and `go to trash can`
  (`TARGET_REACHED`, exploration resumed) all worked with no changes, and no
  node crashed. The acceptance criteria in INSTRUCTIONS.md are about where
  the robot ends up, not how fast it gets there.
- **Rosetta can, rarely, kill a process** with `Illegal instruction` — the
  same glitch described under Troubleshooting for the build. It did not
  happen during the reference run; if a terminal's launch dies that way,
  `Ctrl-C` it and start that terminal again (the README's one-subsystem-
  per-terminal design exists for exactly this).

**Close the Gazebo window once the robot has spawned.** The Gazebo GUI
(`gzclient`) is the single most expensive process in the container — more
than the physics — and nothing in the course needs it after T1's ready
signal: RViz shows the map, the robot, the detections and the landmarks.
Closing it (the window's ✕, or `pkill -x gzclient` in any container shell)
roughly triples the camera frame rate; T1 keeps running, and the line
`[ERROR] [gzclient-2]: process has died` it prints is just that window
going away. To look at the simulation again later, in a spare shell:

```bash
ros2 launch gazebo_ros gzclient.launch.py
```

Beyond that, in this order: give Docker more CPUs in Settings → Resources
(the reference run kept all 8 of its vCPUs busy), shrink the browser window
(fewer pixels for the software renderer), and quit other heavy applications.

## Troubleshooting

- **`docker compose build` stops at once with "this image must be built for
  linux/amd64"** — you ran `docker build` without `--platform`. Use
  `docker compose build`, which pins the platform.
- **The build dies with `… subprocess was killed by signal (Illegal
  instruction)`** — Rosetta occasionally kills an x86-64 process for no
  reason of its own; dpkg's helpers during the big apt step are the usual
  victims (Docker rates amd64 emulation as "best effort", see
  [docker/for-mac#7255](https://github.com/docker/for-mac/issues/7255)).
  The image's install steps already retry on their own. If the build still
  fails, run `docker compose build` again — it resumes from the last good
  layer. Keeping macOS and Docker Desktop up to date helps: Rosetta ships
  with macOS.
- **The browser says "Failed to connect to server"** — the container is not
  running or not healthy yet: `docker compose ps`, then `docker compose logs`.
- **Port 6080 is already in use** — change the host side of the port mapping
  in [`docker-compose.yml`](../docker/docker-compose.yml) to, say,
  `"127.0.0.1:6081:6080"` and open that port instead.
- **T5 prints `Could not initialize NNPACK! Reason: Unsupported hardware`**
  — torch probing a CPU feature Rosetta does not expose. Harmless; inference
  runs on its regular kernels.
- **A window is black or empty for a few seconds after launching** — normal
  with software rendering; RViz in particular takes a moment to draw its
  first frame.
- **`ros2 launch` says `Package 'tb3_bringup' not found` and
  `install/setup.bash` does not exist, yet `install/` is full of files named
  `setup 2.bash`, `tb3_bringup 2`, …** — iCloud restored a deleted build as
  conflict copies: the checkout is inside an iCloud-synced folder (see
  step 2). Move it out (`docker compose down` first, then `mv`), delete
  `build/ install/ log/`, run `docker compose up -d` and `colcon build` again.
- **The checkout is empty inside the container** — the clone lives outside
  the directories Docker Desktop shares. Settings → Resources → File sharing
  must include it (`/Users` is shared by default).
- **Text you paste is going into the browser desktop** — you rarely need to
  type there at all; the terminals are on the Mac. If you do, noVNC's side
  panel (left edge) has a clipboard box.
- **Orphan processes after a crash** — [docs/troubleshooting.md](troubleshooting.md) → *Clean
  restart* works unchanged inside a container shell, and
  `docker compose restart` is the bigger hammer: it restarts the whole
  container, keeping the checkout and the build.
