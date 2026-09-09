# TR-Autonomy03 on an Apple-silicon Mac (Ubuntu 22.04 arm64 VM)

This branch (`mac-arm64`) exists because the simulator's physics/render engine,
SAPIEN, publishes no prebuilt package for ARM64 Linux. Inside your VM
`pip install mani_skill` fails, so SAPIEN has to be compiled from source with a
few patches. `tools/setup_arm_sim.sh` does all of that for you.

The assignment (Parts 1 to 3 in the README) is unchanged. Only the setup differs.

## 0. VM requirements

- UTM or Parallels running __Ubuntu 22.04 Desktop arm64__ (ROS 2 Humble needs 22.04)
- __8 GB RAM minimum__, 4+ CPU cores, __30 GB free disk__ (SAPIEN's build tree is large)
- ROS 2 Humble installed: <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html>
- Log in with __"Ubuntu on Xorg"__ (gear icon on the login screen). The keyboard
  teleop node uses `pynput`, which does not receive keys under Wayland.
- GitHub SSH access to the TR organisation (ask the Autonomy lead), needed for
  the HuskyBot submodule.

## 1. Clone

```bash
git clone -b mac-arm64 git@github.com:Triton-Robotics-Training/TR-Autonomy-3.git
cd TR-Autonomy-3
git submodule update --init          # pulls src/tr-huskybot-cv
```

## 2. Build SAPIEN + ManiSkill + the simulator

### Fast path: a prebuilt SAPIEN wheel

Ask in `#auto-support` whether someone has a `sapien-*-cp310-cp310-linux_aarch64.whl`
built on Ubuntu 22.04. If so:

```bash
./tools/setup_arm_sim.sh --wheel ~/Downloads/sapien-*.whl
```

### Slow path: build it yourself

```bash
./tools/setup_arm_sim.sh
```

This clones SAPIEN and ManiSkill into `~/Documents`, patches them, compiles
SAPIEN (__1 to 3 hours__ on a VM, keep the lid open), installs CPU-only torch,
coacd and the other Python deps, builds the `sim_node` ROS package, writes
`sim_env.sh` in the repo root and runs a verification checklist. It asks for
`sudo` once for apt.

When it finishes, share your wheel so the next person can take the fast path:

```bash
ls ~/Documents/SAPIEN/dist/sapien-*.whl
```

Re-run `./tools/setup_arm_sim.sh --verify-only` any time to diagnose an install.
`tools/ARM_SETUP.md` explains every step and lists the error signatures.

## 3. HuskyBot dependencies and the rest of the workspace

HuskyBot's own `requirements.txt` pins `onnxruntime-gpu`, which has no wheel for
Python 3.10 on ARM64. HuskyBot only ever uses the CPU execution provider, so
install the CPU build instead:

```bash
pip install -r requirements-arm.txt
source /opt/ros/humble/setup.bash
rosdep install -i --from-path src --rosdistro humble -y --skip-keys python3-pynput
colcon build
```

## 4. Running

In __every__ terminal, source these three files first:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source sim_env.sh          # Vulkan software-rendering environment written by the setup script
```

Then, each in its own terminal:

```bash
# simulator: CPU physics + software rendering, reduced camera resolution to save RAM
ros2 launch sim_node sim_node_launch.py "cpu_sim:=true" "cv_exposure:=0.8" \
    "cv_resolution_x:=1280" "cv_resolution_y:=720" "lidar_pointcloud_resolution:=10"

ros2 run huskybot_cv huskybot_cv --ros-args -p "use_sim_time:=true"

ros2 run sim_node keyboard_controls.py
```

Add `"human_gui:=false"` once you no longer need the 3D window; it roughly
doubles the frame rate. Expect low FPS regardless: everything is rendered on the
CPU. The first launch stalls for a few minutes while the arena meshes are
convex-decomposed.

Always launch the simulator through `sim_node_launch.py`, never with
`ros2 run sim_node main.py`. The launch file works around a library-name
collision between torch and the workspace's `shm` package that otherwise makes
`import torch` fail on this platform.

## Known issues on Mac VMs

- __Very low frame rate.__ Software rendering. Lower `cv_resolution_*`, disable
  `human_gui`, and set `enable_lidar:=false` for Parts 1 and 2 (the lidar is not
  used in this training).
- __Robots not responding to teleop.__ Make sure you logged in with "Ubuntu on
  Xorg". If keys still do nothing, check that the `keyboard_controls.py` terminal
  has focus. Beyblade and chassis controls were reported flaky on the first Mac
  bring-up; report what you see in `#auto-support`.
- __`vulkaninfo --summary` shows no device.__ `sudo apt install mesa-vulkan-drivers`
  and re-run `./tools/setup_arm_sim.sh --verify-only`.
- __Disk filled during setup.__ A CUDA build of torch was pulled in. Run
  `pip uninstall torch` and any `nvidia-*` packages, then re-run the setup script.
