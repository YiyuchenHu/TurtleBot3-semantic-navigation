# Setting up on Windows 10/11 (WSL 2)

The recommended Windows setup is **WSL 2 with Ubuntu 22.04**, running ROS 2
Humble and Gazebo natively inside it. It is not a container and not an
emulator: the same amd64 Linux binaries the README installs on a native Ubuntu
machine, on the same CPU, at close to the same speed. Windows supplies the
kernel VM and the GPU; everything above that is stock Ubuntu.

Everything after the setup below — building the workspace and the six-terminal
flow — is identical to Ubuntu and is written down once, in the
[README](../README.md); this page only covers what Windows changes.

A Docker route also exists and is described at the [bottom](#alternative-docker).
Prefer WSL 2: it is faster, and it is the one validated for this page.

> **One Windows-specific step is not optional.** Read
> [Graphics: pick a renderer](#3-graphics-pick-a-renderer-required) before your
> first run. Skipping it produces a simulation that looks healthy — Gazebo
> runs, SLAM maps the room, Nav2 drives — while the robot's camera silently
> returns nothing, so no landmark is ever found and every `go to …` fails.

## 1. Install WSL 2 and Ubuntu 22.04

In PowerShell **as Administrator**:

```powershell
wsl --install -d Ubuntu-22.04
```

Reboot if it asks, then open the new Ubuntu terminal and let it create your
Linux username and password.

**It must be 22.04.** ROS 2 Humble is built for Ubuntu 22.04 (Jammy) only, and
the Microsoft Store's plain "Ubuntu" entry now installs 24.04, which has no
`ros-humble-*` packages at all. Check what you actually got:

```powershell
wsl -l -v                      # STATE Running/Stopped, VERSION must be 2
```

```bash
lsb_release -a                 # inside WSL: must say 22.04
```

If `wsl -l -v` lists a distribution called `Ubuntu` that turns out to be 22.04,
that is fine — but remember its **registered name**, because every `wsl -d …`
command needs it and it may not be `Ubuntu-22.04`. If you got 24.04, install
22.04 alongside it: `wsl --install -d Ubuntu-22.04`.

Confirm the graphical layer (WSLg) is present — it is what puts the Gazebo and
RViz windows on your Windows desktop, with no X server to install:

```bash
sudo apt update && sudo apt install -y x11-apps mesa-utils
xeyes                          # a window should appear on your desktop
glxinfo -B | grep -E 'Device|Accelerated'
```

## 2. Get the code

Clone into your Linux home directory, the same place the README assumes:

```bash
cd ~
git clone -b yolo26n-finetuned https://github.com/YiyuchenHu/TurtleBot3-semantic-navigation.git
cd TurtleBot3-semantic-navigation
```

**Do not clone into `/mnt/c/...`** (that is your Windows `C:` drive seen from
Linux). Files there are reached through a translation layer that is an order of
magnitude slower for the many-small-files work `colcon build` does, Linux file
permissions and symlinks do not survive properly — and `--symlink-install`,
which the README requires on every build, depends on symlinks. Your Linux home
lives in the WSL virtual disk and behaves like a normal ext4 filesystem. If you
already cloned into `/mnt/c`, just `mv` it into `~` and rebuild.

You do not need to configure `core.autocrlf`. Git inside WSL is Linux Git and
leaves line endings alone; the setting only causes trouble if you clone with
*Windows* Git and then build in WSL. Verify with `git config core.autocrlf` —
empty or `false` is what you want.

The detector weights ship with the repository (README → Setup by platform → step 3):
there is nothing to download.

## 3. Graphics: pick a renderer (required)

WSLg exposes the GPU to Linux through a Direct3D 12 translation layer. On a
laptop with switchable graphics it defaults to the **integrated** GPU, and on
the Intel integrated path Gazebo's camera sensor frequently fails in a way that
produces no error at all: the camera publishes at its normal rate, but every
frame contains only the empty sky background, with none of the room, the walls
or the objects in it. LiDAR, physics, SLAM and Nav2 are unaffected, so the
simulation looks completely healthy while the detector receives blank images,
promotes no landmarks, and answers every `go to …` with
`no active <target> in memory`.

Measured on the reference machine (see [What to expect](#what-to-expect-on-wsl-2)),
starting the simulation and checking whether the camera image ever changes:

| Renderer | How | Camera worked |
|---|---|---|
| Intel integrated, D3D12 *(the default)* | — | **2 of 11 starts** |
| Discrete NVIDIA, D3D12 | `export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` | **6 of 6 starts** |
| Software (llvmpipe) | `export LIBGL_ALWAYS_SOFTWARE=1` | **9 of 9 starts** |

So pick one and put it in `~/.bashrc` so every terminal inherits it:

```bash
# If you have a discrete GPU (NVIDIA shown; use the vendor string in glxinfo).
# Fastest option, and it fixes the camera.
echo 'export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA' >> ~/.bashrc

# If you do NOT have a discrete GPU. Slower, but correct.
echo 'export LIBGL_ALWAYS_SOFTWARE=1' >> ~/.bashrc
```

Open a new terminal and confirm the renderer changed:

```bash
glxinfo -B | grep Device       # should name your discrete GPU, or llvmpipe
```

**Check the camera before you trust a run.** With T1 up, open
`rqt_image_view` (it comes with the apt list) on `/camera/image_raw`, or watch
the **Detector Debug Image** in RViz once T2 and T5 are running. A featureless
pale-grey rectangle that never changes as the robot drives is this failure; a
picture of the room is a healthy camera. It is worth ten seconds at the start
of a session, because every downstream symptom looks like a broken detector.

## 4. Continue in the README

From here on Windows is Ubuntu. Work through
[README → Setup by platform](../README.md#setup-by-platform) — the apt list, the pip
list and `colcon build --symlink-install`, all four steps exactly as written —
and then [README → Running](../README.md#running).

Each of the README's six terminals is a WSL tab. Windows Terminal opens one
with `Ctrl-Shift-T`, or run `wsl` in any new tab. The four preparation lines
the README puts at the top of every terminal apply unchanged.

`INSTRUCTIONS.md` and `NOTES.md` apply unchanged.

## What to expect on WSL 2

WSL 2 is a real virtual machine with GPU access, so the simulation runs at
roughly native speed — much closer to a native Ubuntu machine than to the
macOS container. Reference run on a Gigabyte laptop, i7-12700H (20 threads),
64 GB RAM, RTX 3070 Laptop + Intel Iris Xe, Windows 11 26200, WSL 2.7.12 /
WSLg 1.0.73, `world:=warehouse_models`, discrete GPU selected as in step 3:

| | WSL 2 | native Ubuntu, same laptop | macOS container (M3 Max) |
|---|---|---|---|
| `apt install` (README step 1) | 7 min 32 s | — | image build ~17 min |
| `pip install` (README step 2) | 40 s | — | (in the image) |
| `colcon build --symlink-install` | 89 s | — | 27 s |
| Gazebo real-time factor | 0.96 | ≈1.0 | 0.45 |
| Camera frames reaching the detector | 18.7 /s | — | 1–4 /s |
| Detector output rate, full stack | 8.1 /s | — | — |
| Detector, one 640×480 frame | 35–44 ms | 32 ms (39 loaded) | 55 ms |
| T1 → T2 → T3 → T4 → T5 (medians of 10) | 2.4 → 4.3 → 11.4 → 1.6 → 6.2 s | — | 4 → 6 → 25 → 2 → 4 s |
| `acceptance_run.sh --world warehouse_models` | PASS 3/3, 159 s | — | — |

Notes on those numbers:

- **Software rendering costs real throughput.** The same machine forced to
  llvmpipe gives real-time factor 0.84 and only 4.7 camera frames per second,
  and the acceptance run scored 1/3 because exploration ran out of its timeout
  before all three landmarks were promoted. It is the correct choice only when
  there is no discrete GPU — and on such a machine, pass
  `--timeout-scale 2` to `acceptance_run.sh` and expect the README's
  "exploration takes 2–8 minutes" to stretch.
- **Do not bother closing the Gazebo window.** The macOS page says closing
  `gzclient` roughly triples the frame rate; on WSL 2 with a working GPU
  renderer it does not help (measured 22.7 /s with the window open, 19.5 /s
  with it closed — the difference is noise, and the real-time factor is capped
  at 1.0 either way).
- **The detector benchmark drifts upward** across repeated runs on a laptop
  (35.4 → 38.4 → 44.0 ms in one session) as the CPU heats up. NOTES.md §6.4
  reports the same effect on the native machine. Compare first runs to first
  runs.
- **Nav2's documented spin-in-place wedge happens here too**, at about the
  same rate as anywhere else: 1 of 10 cold starts logged
  `Failed to make progress` and recovered on its own. [docs/troubleshooting.md](troubleshooting.md)
  covers it.
- **SLAM freezes were not observed** — 10 of 10 cold starts built a growing
  map. That is too few runs to claim WSL 2 is better than the native machine's
  occasional stall; it only says WSL 2 adds no obvious new failure of that
  kind.

## Troubleshooting

- **`wsl --install` fails, or WSL 2 refuses to start** — hardware
  virtualization is off. Reboot into UEFI/BIOS and enable **Intel VT-x /
  AMD-V** (often called *Virtualization Technology*, *SVM Mode*, or hidden
  under *Advanced → CPU Configuration*). On Windows also confirm
  **Virtual Machine Platform** and **Windows Subsystem for Linux** are ticked
  in *Turn Windows features on or off*. Docker Desktop or another hypervisor
  running at the same time can also hold the VM stack; a reboot clears it.
- **`ros-humble-desktop` has no installation candidate** — either you are on
  Ubuntu 24.04, not 22.04 (step 1: install 22.04, there is no other fix), or
  the ROS 2 apt repository has not been added yet: follow
  [the ROS 2 Humble install guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
  up to `sudo apt update`, then retry.
- **The robot explores the whole room but no landmark ever appears, and every
  `go to …` says `no active <target> in memory`** — the blank-camera failure
  from step 3. Check the camera image, then set a renderer.
- **Gazebo or RViz opens a black or empty window** — usually just the first
  frame taking a moment under software rendering. If it stays black, you are
  hitting the same renderer problem: try the other option in step 3.
- **`colcon build` is very slow, or `install/setup.bash` never appears** —
  the checkout is on `/mnt/c`. Move it to `~` (step 2). Real-time antivirus
  scanning makes this much worse, because every file `colcon` writes is
  scanned; the reference timings above come from a machine with Microsoft
  Defender real-time protection switched off, so a machine with it on can be
  slower. If you keep the checkout in `~` as instructed, Defender does not see
  those writes as ordinary files at all; if you must work on `/mnt/c`, add that
  folder to *Virus & threat protection → Exclusions*.
- **WSL is eating all the RAM** — by default WSL 2 may claim up to half of
  physical memory (32 GB of 64 GB on the reference machine); the full stack
  actually used about 5 GB, visible on the Windows side as the `vmmemWSL`
  process. To cap it, create `C:\Users\<you>\.wslconfig`:

  ```ini
  [wsl2]
  memory=16GB
  processors=8
  ```

  then `wsl --shutdown` and reopen. Memory is released back to Windows lazily,
  so `vmmemWSL` looking large is not by itself a problem.
- **`wsl --shutdown` is the big hammer.** It restarts the whole VM and clears
  GPU, WSLg and networking state. Worth trying once when something is wedged
  in a way no ROS-level clean restart fixes. Note that it also clears `/tmp`
  and kills every running process in every distribution.
- **Orphan processes after a crash** — [docs/troubleshooting.md](troubleshooting.md)
  → *Clean restart* works unchanged.

## Alternative: Docker

The repository also ships the container the macOS page uses, and it runs on
Windows through Docker Desktop's WSL 2 backend. It is the fallback if WSL 2
native does not work for you; it is slower and adds a browser desktop between
you and the windows.

1. Install [Docker Desktop for Windows](https://docs.docker.com/desktop/setup/install/windows-install/),
   and in **Settings → General** make sure *Use the WSL 2 based engine* is on.
2. In **Settings → Resources → WSL Integration**, tick your Ubuntu-22.04
   distribution. Without this, `docker` is not on `PATH` inside WSL. (Ticking
   only *Enable integration with my default WSL distro* is not enough unless
   Ubuntu-22.04 actually is your default — check the `*` in `wsl -l -v`.)
3. From then on, [docs/setup-macos.md](setup-macos.md) applies verbatim from
   its step 3 onwards — same `docker/` directory, same
   `docker compose build` / `up -d`, same desktop at
   <http://localhost:6080/>, same `docker compose exec tb3 bash` per README
   terminal. **The compose file needs no Windows-specific changes**; the
   `platform: linux/amd64` line that means Rosetta emulation on Apple Silicon
   is simply native here, so the image builds and runs faster than on a Mac.

Measured on the same reference machine, for comparison with the WSL 2 table
above: `docker compose build` 10 min 33 s (Mac: ~17 min), `colcon build`
inside the container 25 s, real-time factor 0.86 with the full stack up,
6.2 camera frames per second, 5.9 detections per second. The container has no
trouble with the camera — it renders on its own software OpenGL inside the
virtual desktop and never touches the WSLg D3D12 path, so the failure in
step 3 does not apply here. What you pay for that is throughput and memory:
the WSL VM held 22.4 GB resident (`vmmemWSL`) with Docker Desktop running,
against 5.0 GB for the WSL 2 native route.

Two Windows-specific things to know if you take this route:

- **`docker` has to be on `PATH` inside WSL, and only the checkbox in step 2
  puts it there.** Enabling the integration by hand-editing Docker Desktop's
  `settings.json` is not enough: bind mounts then resolve against the wrong
  filesystem and your checkout shows up **empty** inside the container.
- **The container runs as root**, so a `colcon build` done inside it leaves
  root-owned `build/`, `install/` and `log/` directories in your WSL
  checkout. A later native build in WSL cannot delete them
  (`rm: cannot remove ...: Permission denied`); clear them with
  `sudo rm -rf build install log` and rebuild. Pick one route and stay on it,
  or clean in between.
