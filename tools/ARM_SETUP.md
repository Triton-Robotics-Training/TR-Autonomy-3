# Running the ManiSkill simulator on ARM64 (Jetson, or an Ubuntu VM on an Apple-silicon Mac)

`tools/setup_arm_sim.sh` automates everything below. This file explains *why*
each step exists so the script can be repaired when upstream moves.

```bash
./tools/setup_arm_sim.sh                     # full install, builds SAPIEN natively (hours)
./tools/setup_arm_sim.sh --wheel sapien.whl  # reuse a prebuilt SAPIEN wheel (minutes)
./tools/setup_arm_sim.sh --verify-only       # re-run the checklist on an existing install
```

Target: Ubuntu 22.04 arm64, ROS 2 Humble, Python 3.10. The VM should have at
least 8 GB RAM and ~30 GB free disk; give it 4 cores or more.

## Why this is needed at all

`pip install mani_skill` pulls `sapien`, and SAPIEN publishes **no aarch64
wheel on PyPI**. So SAPIEN has to be built from source, and a handful of things
around it assume x86 or CUDA. The script pins SAPIEN `fd977123` and ManiSkill
`21551f2b` because those are the commits the patches below were verified on.

## Step-by-step

### 1. System packages

Build toolchain, X11 dev headers (for the human GUI window), Vulkan loader and
tools, and **`mesa-vulkan-drivers`**. The last one provides the llvmpipe
software Vulkan driver (`/usr/share/vulkan/icd.d/lvp_icd.aarch64.json`), which
is what actually renders on a VM with no GPU. Ubuntu Desktop usually has it;
Ubuntu Server does not.

### 1b. Python packaging toolchain

SAPIEN's `setup.py` does `from setuptools.command.build import build`. That
module only exists in setuptools >= 62.4 and Ubuntu 22.04 ships 59.6.0, so a
native build dies with `ModuleNotFoundError` before compiling anything. The
script upgrades pip, setuptools and packaging (into the active venv, or the user
site when system site-packages is not writable) *before*
`install_sapien` (the later `install_pydeps` step would be too late).

### 1c. torch (CPU-only, installed before ManiSkill)

On aarch64 the PyPI `torch` wheels declare the `nvidia-*-cu*` packages as hard
dependencies, so `pip install torch` on a GPU-less VM downloads ~5 GB of CUDA
libraries and can fill the disk. ManiSkill's `pip install -e` resolves torch as
a dependency, so the CPU wheel is installed first from
`https://download.pytorch.org/whl/cpu`. On a Jetson this step is skipped: use
NVIDIA's JetPack-matched wheel instead.

### 2. SAPIEN source patches

All patches are idempotent and print `already patched` on a re-run.

| Patch | Problem |
|---|---|
| `include(zlib)` -> `find_package(ZLIB REQUIRED)` | bundled zlib does not configure on this toolchain |
| `option(SAPIEN_CUDA ...)` declared before `project()` | `project()` enables the CUDA language unconditionally off-Apple, so CMake demands nvcc even with `-DSAPIEN_CUDA=OFF` |
| `simsense` import made optional | `sapien/__init__.py` imports simsense on every non-Darwin platform, but it is only compiled when `SAPIEN_CUDA=ON` |
| `physx_engine.cpp`: `#if PX_SUPPORT_GPU_PHYSX` -> `#if PX_SUPPORT_GPU_PHYSX && defined(SAPIEN_CUDA)` | the CUDA *calls* are guarded by `PX_SUPPORT_GPU_PHYSX` (true on all 64-bit Linux) but the matching *includes* by `SAPIEN_CUDA`, so a CPU build compiles calls against nothing declared |
| `pinocchio/cmake/pinocchio.cmake`: `cytranet.dl.sourceforge.net` -> `downloads.sourceforge.net` | the hardcoded mirror no longer answers and the tinyxml download times out; the redirector serves the byte-identical tarball so the MD5 check is unchanged |

CMake >= 3.24 is required by `sapien-vulkan-2`; 22.04 ships 3.22, so the script
pip-installs `cmake==3.31.x` (deliberately not 4.x, which drops
`cmake_minimum_required(VERSION <3.5)` still used by bundled deps).

### 3. ManiSkill source patches

- `mplib` is removed from `setup.py`: it has no aarch64 wheel, its sdist does
  not build, and nothing in `sim_node` uses it.
- The `sapien>=3.0.0` pin is loosened to `>=3.0.0.dev0` so pip accepts the
  locally built wheel instead of looking on PyPI.
- **Render device**: on a box with no CUDA, ManiSkill falls back to
  `sapien.Device("cpu")` for the renderer. That device is not renderable
  (SAPIEN's matcher only understands `cuda`, `cuda:N` and `pci:...`), so
  `RenderSystem()` dies with `Failed to find a supported physical device "cpu"`
  even though llvmpipe is present. The patch lets SAPIEN pick the device itself.
  `sim_node` requests `render_backend="sapien_cpu"` when `cpu_sim:=true`, so
  this patch is required for the simulator, not just for ManiSkill demos.

#### mani_skill shadowing

A leftover `site-packages/mani_skill/` from an older non-editable install
shadows the editable one (setuptools *appends* its finder to `sys.meta_path`),
yielding an empty namespace package and `mani_skill.__file__ == None`. The
script moves such a directory aside.

### 4. Python deps

`coacd` is **not optional**: `sim_node` convex-decomposes the arena meshes on
load. `fast_kinematics` has aarch64 wheels (glibc >= 2.34) and is installed if
possible; it is optional.

### 5. Environment file (`sim_env.sh`, generated, not tracked)

SAPIEN's `_vulkan_tricks.py` probes `/usr/share/vulkan/icd.d` and
`/usr/lib/x86_64-linux-gnu`, neither of which is right on ARM, and otherwise
falls back to a **bundled x86-64 libvulkan**. `sim_env.sh` sets
`VK_ICD_FILENAMES` and prepends the aarch64 multiarch dir to `LD_LIBRARY_PATH`
so both probes short-circuit. Source it before launching.

`VK_ICD_FILENAMES` is only written when an ICD was actually found; pointing it
at a missing file makes the loader report `ERROR_INCOMPATIBLE_DRIVER` and
enumerate zero devices, which is worse than leaving it unset.

#### libshm

torch ships `torch/lib/libshm.so`; this repo's `shm` package also builds a
`libshm.so`, and `install/setup.bash` puts it on `LD_LIBRARY_PATH`. On torch
builds where `LD_LIBRARY_PATH` takes precedence over the library's own search
path (`DT_RUNPATH` rather than `DT_RPATH`), `import torch` binds to the wrong
library and fails. This was observed on an aarch64 Ubuntu VM; on an x86 machine
with a `DT_RPATH` torch build it does not reproduce. Check 8 of
`--verify-only` tells you which case you are in.

The fix is **not** a global `LD_LIBRARY_PATH` export: `zero_copy_forwarder`
and every other C++ node that links our `shm` would then bind to torch's copy
instead. `launch/sim_node_launch.py` prepends `torch/lib` via `additional_env`
for the `sim_node` process only. Always start the simulator through that launch
file. (`--verify-only` check 8 reports whether the collision exists on your
machine.)

### 6. ROS workspace

`rosdep` is scoped to `--packages-up-to sim_node` because other vendored
packages in `src/` carry keys rosdep cannot resolve. `python3-pynput` is
skipped because pip already installed a newer pynput.

## Vulkan

`vulkaninfo --summary` must list a device. With no GPU it should be
`llvmpipe (LLVM ...)`. If it lists nothing: install `mesa-vulkan-drivers`,
check `VK_ICD_FILENAMES` points at an existing file, and make sure
`libvulkan.so.1` is in `/usr/lib/aarch64-linux-gnu`.

## Headless render

Check 3 renders a 1280x720 frame with no window. If it dies with an
out-of-memory or `vk::DeviceLostError` signature, the VM has too little RAM
for the default 1920x1200 CV camera: launch with `cv_resolution_x:=1280
cv_resolution_y:=720` and give the VM more memory.

## Launching

```bash
source sim_env.sh
ros2 launch sim_node sim_node_launch.py \
  human_gui:=true cpu_sim:=true \
  cv_resolution_x:=1280 cv_resolution_y:=720 \
  lidar_pointcloud_resolution:=10
```

The first launch stalls for minutes while coacd decomposes the arena meshes.
It is not hung. Expect low frame rates: everything is software-rendered.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `ModuleNotFoundError: setuptools.command.build` during SAPIEN build | setuptools < 62.4; re-run the script (gate 1b) |
| `pip install torch` downloads nvidia-* packages, disk fills | CUDA torch resolved; `pip uninstall torch nvidia-*`, re-run (step 1c) |
| `Failed to find a supported physical device "cpu"` | ManiSkill render-device patch missing (step 3) |
| `import torch` fails only after `source install/setup.bash` | libshm collision; launch through `sim_node_launch.py` (see "libshm") |
| `mani_skill.__file__` is `None` | shadowing; see "mani_skill shadowing" |
| Build hangs fetching `tinyxml_2_6_2.tar.gz` | dead sourceforge mirror; pinocchio.cmake patch missing (step 2) |
| Robots do not respond to keyboard teleop in the VM | `keyboard_controls.py` uses pynput, which needs an X11 session. Log into the VM desktop with "Ubuntu on Xorg", not Wayland. Known open issue on Mac VMs. |

## Distributing a prebuilt wheel

A native SAPIEN build takes hours on a VM. Build it once, then hand the wheel
to everyone else:

```bash
ls ~/Documents/SAPIEN/dist/sapien-*.whl        # produced by the native build
./tools/setup_arm_sim.sh --wheel sapien-3.0.0.dev*-cp310-cp310-linux_aarch64.whl
```

The wheel is not manylinux-repaired, so it only ports between machines running
the same distro release and Python version (Ubuntu 22.04, Python 3.10).
