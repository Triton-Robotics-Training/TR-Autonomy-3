#!/usr/bin/env bash
#
# setup_arm_sim.sh -- provision SAPIEN + ManiSkill + sim_node on an ARM64 device
# (Jetson, or an Ubuntu 22.04 VM on an Apple-silicon Mac).
#
# See tools/ARM_SETUP.md for the reasoning behind every step here.
#
# Quick start:
#   ./tools/setup_arm_sim.sh                        # full install, build SAPIEN natively
#   ./tools/setup_arm_sim.sh --wheel path/to.whl    # reuse a prebuilt SAPIEN wheel (fast)
#   ./tools/setup_arm_sim.sh --docker-build         # build a portable manylinux wheel
#   ./tools/setup_arm_sim.sh --verify-only          # diagnose an existing install
#
set -euo pipefail

# ----------------------------------------------------------------------------
# defaults
# ----------------------------------------------------------------------------
SAPIEN_COMMIT="fd977123"
MANISKILL_COMMIT="21551f2b"
SAPIEN_SRC="${HOME}/Documents/SAPIEN"
MANISKILL_SRC="${HOME}/Documents/ManiSkill"
SAPIEN_WHEEL=""
BUILD_MODE="native"          # native | docker | wheel | skip
WANT_CUDA="auto"             # auto | yes | no
JOBS="$(nproc)"; [ "$JOBS" -gt 4 ] && JOBS=4
SKIP_APT=0
SKIP_ROS=0
VERIFY_ONLY=0
ROS_DISTRO_DEFAULT="humble"
PY="python3"

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="${WS_ROOT}/sim_env.sh"

# ----------------------------------------------------------------------------
# output helpers
# ----------------------------------------------------------------------------
if [ -t 1 ]; then
  R=$'\033[31m'; G=$'\033[32m'; Y=$'\033[33m'; B=$'\033[36m'; BOLD=$'\033[1m'; N=$'\033[0m'
else
  R=""; G=""; Y=""; B=""; BOLD=""; N=""
fi
step()  { printf '\n%s==> %s%s\n' "${B}${BOLD}" "$*" "${N}"; }
ok()    { printf '  %s[ ok ]%s %s\n' "${G}" "${N}" "$*"; }
warn()  { printf '  %s[warn]%s %s\n' "${Y}" "${N}" "$*"; }
fail()  { printf '  %s[FAIL]%s %s\n' "${R}" "${N}" "$*"; }
die()   { fail "$*"; exit 1; }

usage() {
  sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'
  cat <<EOF

Options:
  --sapien-src DIR      SAPIEN checkout location   (default: ${SAPIEN_SRC})
  --maniskill-src DIR   ManiSkill checkout location(default: ${MANISKILL_SRC})
  --wheel PATH          Install this prebuilt SAPIEN wheel instead of building
  --docker-build        Build SAPIEN via upstream aarch64 docker (portable wheel)
  --skip-sapien         Assume SAPIEN is already installed
  --cuda / --no-cuda    Force SAPIEN CUDA support on/off (default: auto-detect)
  --jobs N              Parallel compile jobs (default: ${JOBS}; keep low on 8GB boards)
  --skip-apt            Don't touch apt (no sudo needed)
  --skip-ros            Don't run colcon build
  --verify-only         Run the verification checklist and exit
  -h, --help            This message
EOF
}

# ----------------------------------------------------------------------------
# arg parsing
# ----------------------------------------------------------------------------
while [ $# -gt 0 ]; do
  case "$1" in
    --sapien-src)     SAPIEN_SRC="$2"; shift 2 ;;
    --maniskill-src)  MANISKILL_SRC="$2"; shift 2 ;;
    --wheel)          SAPIEN_WHEEL="$2"; BUILD_MODE="wheel"; shift 2 ;;
    --docker-build)   BUILD_MODE="docker"; shift ;;
    --skip-sapien)    BUILD_MODE="skip"; shift ;;
    --cuda)           WANT_CUDA="yes"; shift ;;
    --no-cuda)        WANT_CUDA="no"; shift ;;
    --jobs)           JOBS="$2"; shift 2 ;;
    --skip-apt)       SKIP_APT=1; shift ;;
    --skip-ros)       SKIP_ROS=1; shift ;;
    --verify-only)    VERIFY_ONLY=1; shift ;;
    -h|--help)        usage; exit 0 ;;
    *)                die "unknown option: $1 (try --help)" ;;
  esac
done

# ----------------------------------------------------------------------------
# 0. platform detection
# ----------------------------------------------------------------------------
detect_platform() {
  step "Detecting platform"
  ARCH="$(uname -m)"
  ok "arch: ${ARCH}"
  [ "$ARCH" = "aarch64" ] || [ "$ARCH" = "arm64" ] || \
    warn "this script targets ARM64; on ${ARCH} you can just 'pip install sapien'"

  PYVER="$(${PY} -c 'import sys; print("%d.%d"%sys.version_info[:2])')"
  ok "python: ${PYVER}"

  IS_JETSON=0
  if [ -f /etc/nv_tegra_release ]; then
    IS_JETSON=1
    ok "Jetson detected: $(head -1 /etc/nv_tegra_release)"
  else
    warn "not a Jetson -- no NVIDIA GPU assumed unless nvidia-smi says otherwise"
  fi

  # Test for nvcc, not just the directory: a runtime-only CUDA install creates
  # /usr/local/cuda with no bin/nvcc, which would set SAPIEN_CUDA=ON and then
  # fail in CMakeDetermineCUDACompiler.
  HAS_CUDA=0
  if [ -x /usr/local/cuda/bin/nvcc ]; then
    HAS_CUDA=1; ok "CUDA toolkit: /usr/local/cuda (nvcc present)"
  elif [ -d /usr/local/cuda ]; then
    warn "/usr/local/cuda exists but has no bin/nvcc -- treating as no CUDA"
  fi

  # Locate the Vulkan ICD. ARM/Jetson puts it in /etc, x86 in /usr/share.
  ICD=""
  for cand in /etc/vulkan/icd.d/nvidia_icd.json \
              /usr/share/vulkan/icd.d/nvidia_icd.json \
              /usr/share/vulkan/icd.d/lvp_icd.${ARCH}.json; do
    [ -f "$cand" ] && { ICD="$cand"; break; }
  done
  if [ -n "$ICD" ]; then ok "vulkan ICD: ${ICD}"; else warn "no Vulkan ICD found -- rendering will not work (see tools/ARM_SETUP.md, 'Vulkan')"; fi

  # libvulkan lives in the arch-specific multiarch dir; SAPIEN only probes the x86 one.
  VKLIBDIR=""
  for d in /usr/lib/aarch64-linux-gnu /usr/lib/${ARCH}-linux-gnu /usr/lib64 /usr/lib; do
    [ -f "$d/libvulkan.so.1" ] && { VKLIBDIR="$d"; break; }
  done
  if [ -n "$VKLIBDIR" ]; then ok "libvulkan: ${VKLIBDIR}"; else warn "libvulkan.so.1 not found -- install libvulkan1"; fi

  if [ "$WANT_CUDA" = "auto" ]; then
    if [ "$HAS_CUDA" = "1" ]; then WANT_CUDA="yes"; else WANT_CUDA="no"; fi
  fi
  ok "SAPIEN CUDA support: ${WANT_CUDA}"
}

# ----------------------------------------------------------------------------
# 1. system packages
# ----------------------------------------------------------------------------
install_apt() {
  step "Installing system packages"
  if [ "$SKIP_APT" = "1" ]; then warn "skipped (--skip-apt)"; return; fi
  sudo apt-get update
  # mesa-vulkan-drivers provides the llvmpipe software Vulkan ICD
  # (/usr/share/vulkan/icd.d/lvp_icd.<arch>.json) that a GPU-less VM renders with.
  # Ubuntu Desktop usually has it already; Ubuntu Server does not.
  sudo apt-get install -y \
    build-essential cmake git git-lfs "python${PYVER}-dev" python3-pip \
    zlib1g-dev libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev \
    libvulkan1 vulkan-tools mesa-vulkan-drivers libglvnd-dev
  ok "apt deps installed"
  # Re-run ICD detection: mesa-vulkan-drivers may have just been installed.
  if [ -z "${ICD:-}" ]; then
    for cand in /usr/share/vulkan/icd.d/lvp_icd.${ARCH}.json; do
      [ -f "$cand" ] && { ICD="$cand"; ok "vulkan ICD (post-apt): ${ICD}"; break; }
    done
  fi
}

# ----------------------------------------------------------------------------
# 1b. python packaging toolchain
# ----------------------------------------------------------------------------
# SAPIEN's setup.py does `from setuptools.command.build import build`, which only
# exists in setuptools >= 62.4. Ubuntu 22.04 ships 59.6.0, so a native build dies
# with ModuleNotFoundError before compiling anything. install_pydeps also pip
# installs setuptools, but that runs AFTER install_sapien -- the gate has to be here.
SETUPTOOLS_MIN="62.4"
PIP_MIN="21.3"
PACKAGING_MIN="22.0"

# version_ge <have> <min>  -- 0 if have >= min (sort -V semantics)
version_ge() { [ "$(printf '%s\n%s\n' "$2" "$1" | sort -V | head -1)" = "$2" ]; }

py_pkg_version() {
  ${PY} -c "import importlib.metadata as m; print(m.version('$1'))" 2>/dev/null || echo 0
}

ensure_pip() {
  local have; have="$(py_pkg_version pip)"
  if version_ge "$have" "$PIP_MIN"; then ok "pip ${have} (>= ${PIP_MIN})"; return; fi
  warn "pip ${have} is too old -- upgrading (user site)"
  ${PY} -m pip install --upgrade "pip>=${PIP_MIN}"
  have="$(py_pkg_version pip)"
  version_ge "$have" "$PIP_MIN" || die "pip is still ${have} after upgrade"
  ok "pip ${have}"
}

ensure_setuptools() {
  local have; have="$(py_pkg_version setuptools)"
  if version_ge "$have" "$SETUPTOOLS_MIN"; then ok "setuptools ${have} (>= ${SETUPTOOLS_MIN})"; return; fi
  warn "setuptools ${have} lacks setuptools.command.build (SAPIEN setup.py needs >= ${SETUPTOOLS_MIN}) -- upgrading"
  ${PY} -m pip install --upgrade "setuptools>=${SETUPTOOLS_MIN}" wheel
  have="$(py_pkg_version setuptools)"
  version_ge "$have" "$SETUPTOOLS_MIN" \
    || die "setuptools is still ${have}; is ~/.local/lib/python${PYVER}/site-packages shadowed by the system copy?"
  ok "setuptools ${have}"
}

ensure_packaging() {
  local have; have="$(py_pkg_version packaging)"
  if version_ge "$have" "$PACKAGING_MIN"; then ok "packaging ${have} (>= ${PACKAGING_MIN})"; return; fi
  warn "packaging ${have} is too old for modern wheel tags -- upgrading"
  ${PY} -m pip install --upgrade "packaging>=${PACKAGING_MIN}"
  ok "packaging $(py_pkg_version packaging)"
}

# ----------------------------------------------------------------------------
# 1c. torch
# ----------------------------------------------------------------------------
# On aarch64, PyPI's torch wheels declare the nvidia-*-cu* packages as hard
# dependencies (bare Linux marker), so a plain `pip install torch` on a GPU-less
# VM downloads ~5 GB of CUDA libraries and can fill the disk. ManiSkill's
# `pip install -e` would resolve torch that way, so the CPU-only wheel from the
# PyTorch index must be installed BEFORE ManiSkill. Jetsons are skipped: they
# need NVIDIA's JetPack-matched wheel, not the CPU one.
ensure_torch() {
  if ${PY} -c 'import torch' 2>/dev/null; then
    ok "torch $(py_pkg_version torch) already installed -- leaving it alone"
    return
  fi
  if [ "$IS_JETSON" = "1" ] || [ "$WANT_CUDA" = "yes" ]; then
    warn "torch not installed; install NVIDIA's JetPack-matched wheel yourself (CPU wheel skipped on CUDA/Jetson)"
    return
  fi
  step "Installing CPU-only torch (avoids the ~5 GB CUDA dependency chain on aarch64)"
  ${PY} -m pip install torch --index-url https://download.pytorch.org/whl/cpu
  ok "torch $(py_pkg_version torch) (cpu)"
}

# 3rd_party/sapien-vulkan-2 needs CMake >= 3.24; Ubuntu 22.04 ships 3.22.1 and
# never will. pip has aarch64 wheels. Deliberately NOT 4.x: CMake 4 dropped
# support for cmake_minimum_required(VERSION <3.5), which SAPIEN's bundled
# openvr and friends still declare.
CMAKE_MIN="3.24"
CMAKE_PIP_VERSION="3.31.10"
ensure_cmake() {
  local have
  have="$(cmake --version 2>/dev/null | head -1 | grep -oE '[0-9]+\.[0-9]+(\.[0-9]+)?' || echo 0)"
  if [ "$(printf '%s\n%s\n' "$CMAKE_MIN" "$have" | sort -V | head -1)" = "$CMAKE_MIN" ]; then
    ok "cmake ${have} (>= ${CMAKE_MIN})"
    return
  fi
  warn "cmake ${have} is too old (sapien-vulkan-2 needs >= ${CMAKE_MIN}) -- installing via pip"
  ${PY} -m pip install --user "cmake==${CMAKE_PIP_VERSION}"
  local userbin
  userbin="$(${PY} -c 'import site,os; print(os.path.join(os.path.dirname(site.getusersitepackages()), "..", "..", "bin"))' 2>/dev/null)"
  userbin="$(cd "$userbin" 2>/dev/null && pwd || echo "${HOME}/.local/bin")"
  case ":${PATH}:" in
    *":${userbin}:"*) ;;
    *) export PATH="${userbin}:${PATH}"
       warn "prepended ${userbin} to PATH for this run -- add it to your shell profile" ;;
  esac
  have="$(cmake --version 2>/dev/null | head -1 | grep -oE '[0-9]+\.[0-9]+(\.[0-9]+)?' || echo 0)"
  [ "$(printf '%s\n%s\n' "$CMAKE_MIN" "$have" | sort -V | head -1)" = "$CMAKE_MIN" ] \
    || die "cmake is still ${have} after pip install -- check that ${userbin} precedes /usr/bin on PATH"
  ok "cmake ${have} (pip, ${userbin})"
}

# ----------------------------------------------------------------------------
# 2. SAPIEN
# ----------------------------------------------------------------------------
# project() enables the CUDA language unconditionally off-Apple, so CMake
# demands nvcc even when -DSAPIEN_CUDA=OFF was passed. Every other CUDA block
# in the file is already guarded by SAPIEN_CUDA; only project() is not.
# Declaring the option before project() lets the guard apply there too.
patch_sapien_cuda_lang() {
  local f="${SAPIEN_SRC}/CMakeLists.txt"
  if grep -q '^if (APPLE OR NOT SAPIEN_CUDA)' "$f"; then
    ok "CMakeLists.txt CUDA-language guard already patched"
  elif grep -q '^if (APPLE)$' "$f"; then
    sed -i 's|^if (APPLE)$|# Declared before project() so the CUDA language is only enabled when it is\n# actually wanted -- otherwise CMake requires nvcc even for a CPU-only build.\noption(SAPIEN_CUDA "Enable SAPIEN CUDA functionalities, including dlpack, CUDA buffer, denoiser, and simsense" ON)\n\nif (APPLE OR NOT SAPIEN_CUDA)|' "$f"
    ok "patched CMakeLists.txt: CUDA language now guarded by SAPIEN_CUDA"
  else
    warn "CMakeLists.txt: project() block not recognized -- upstream may have changed"
  fi
}

# The one real source patch: the bundled zlib doesn't configure on this
# toolchain, so use the system one. Idempotent.
patch_sapien_cmake() {
  local f="${SAPIEN_SRC}/CMakeLists.txt"
  if grep -q '^find_package(ZLIB REQUIRED)' "$f"; then
    ok "CMakeLists.txt already patched"
  elif grep -q '^include(zlib)' "$f"; then
    sed -i 's/^include(zlib)$/find_package(ZLIB REQUIRED)/' "$f"
    ok "patched CMakeLists.txt: include(zlib) -> find_package(ZLIB REQUIRED)"
  else
    warn "CMakeLists.txt: neither include(zlib) nor find_package(ZLIB) found -- upstream may have changed"
  fi
}

# sapien/__init__.py imports simsense on every non-Darwin platform, but simsense
# is only compiled when SAPIEN_CUDA=ON -- so a CPU-only build is unimportable.
patch_sapien_simsense_import() {
  local f="${SAPIEN_SRC}/python/py_package/__init__.py"
  if grep -q 'simsense is only compiled when SAPIEN_CUDA=ON' "$f"; then
    ok "py_package/__init__.py simsense import already guarded"
  elif grep -q '^    from \.pysapien import simsense$' "$f"; then
    ${PY} - "$f" <<'PYEOF'
import sys
path = sys.argv[1]
src = open(path).read()
old = 'if platform.system() != "Darwin":\n    from .pysapien import simsense\n'
new = ('if platform.system() != "Darwin":\n'
       '    try:\n'
       '        from .pysapien import simsense\n'
       '    except ImportError:\n'
       '        # simsense is only compiled when SAPIEN_CUDA=ON\n'
       '        pass\n')
open(path, "w").write(src.replace(old, new, 1))
PYEOF
    ok "patched py_package/__init__.py: simsense import now optional"
  else
    warn "py_package/__init__.py: simsense import not recognized -- upstream may have changed"
  fi
}

# physx_engine.cpp guards its CUDA calls with PX_SUPPORT_GPU_PHYSX (true on every
# 64-bit Linux) but guards the matching #includes with SAPIEN_CUDA (false without
# nvcc), so a CPU-only build compiles calls against nothing declared. Tie the
# call guard to SAPIEN_CUDA as well. Idempotent.
patch_sapien_physx_cuda_guard() {
  local f="${SAPIEN_SRC}/src/physx/physx_engine.cpp"
  [ -f "$f" ] || { warn "physx_engine.cpp not found -- upstream may have changed"; return; }
  if grep -q '^#if PX_SUPPORT_GPU_PHYSX && defined(SAPIEN_CUDA)' "$f"; then
    ok "physx_engine.cpp CUDA guard already patched"
  elif [ "$(grep -c '^#if PX_SUPPORT_GPU_PHYSX$' "$f")" = "1" ]; then
    sed -i 's|^#if PX_SUPPORT_GPU_PHYSX$|#if PX_SUPPORT_GPU_PHYSX \&\& defined(SAPIEN_CUDA)|' "$f"
    ok "patched physx_engine.cpp: CUDA context manager now also guarded by SAPIEN_CUDA"
  else
    warn "physx_engine.cpp: expected exactly one '#if PX_SUPPORT_GPU_PHYSX' -- upstream may have changed"
  fi
}

# pinocchio/cmake/pinocchio.cmake downloads tinyxml 2.6.2 from a hardcoded
# sourceforge mirror (cytranet) that no longer answers, so the build hangs until
# the fetch times out. The downloads.sourceforge.net redirector serves the
# byte-identical tarball, so the MD5 check is unchanged. Idempotent.
patch_sapien_tinyxml_url() {
  local f="${SAPIEN_SRC}/pinocchio/cmake/pinocchio.cmake"
  [ -f "$f" ] || { warn "pinocchio.cmake not found -- upstream may have changed"; return; }
  if grep -q 'downloads.sourceforge.net/project/tinyxml/' "$f"; then
    ok "pinocchio.cmake tinyxml URL already patched"
  elif grep -q 'cytranet.dl.sourceforge.net/project/tinyxml/' "$f"; then
    sed -i 's|https://cytranet.dl.sourceforge.net/project/tinyxml/|https://downloads.sourceforge.net/project/tinyxml/|' "$f"
    ok "patched pinocchio.cmake: tinyxml URL -> downloads.sourceforge.net redirector"
  else
    warn "pinocchio.cmake: tinyxml sourceforge URL not recognized -- upstream may have changed"
  fi
}

fetch_sapien_src() {
  if [ ! -d "${SAPIEN_SRC}/.git" ]; then
    step "Cloning SAPIEN -> ${SAPIEN_SRC}"
    git clone https://github.com/haosulab/SAPIEN.git "${SAPIEN_SRC}"
    git -C "${SAPIEN_SRC}" checkout "${SAPIEN_COMMIT}"
  else
    ok "SAPIEN source present at ${SAPIEN_SRC}"
  fi
  step "Initializing SAPIEN submodules (sapien-vulkan-2, simsense)"
  git -C "${SAPIEN_SRC}" submodule update --init --recursive
  patch_sapien_cmake
  patch_sapien_cuda_lang
  patch_sapien_simsense_import
  patch_sapien_physx_cuda_guard
  patch_sapien_tinyxml_url
}

install_sapien() {
  step "Installing SAPIEN (mode: ${BUILD_MODE})"

  if [ "$BUILD_MODE" = "skip" ]; then warn "skipped (--skip-sapien)"; return; fi

  if [ "$BUILD_MODE" = "wheel" ]; then
    [ -f "$SAPIEN_WHEEL" ] || die "wheel not found: ${SAPIEN_WHEEL}"
    ${PY} -m pip install --force-reinstall "$SAPIEN_WHEEL"
    ok "installed ${SAPIEN_WHEEL}"
    warn "native-tagged wheels are not manylinux-repaired; they only port to similar distros"
    return
  fi

  fetch_sapien_src

  if [ "$BUILD_MODE" = "docker" ]; then
    command -v docker >/dev/null || die "docker not installed"
    step "Building SAPIEN in upstream aarch64 docker image (produces a portable wheel)"
    ( cd "${SAPIEN_SRC}" && CMAKE_BUILD_PARALLEL_LEVEL="${JOBS}" \
        ./scripts/docker_build_wheels_aarch64.sh "${PYVER//./}" )
  else
    step "Building SAPIEN natively (this takes a long time -- dozens of C++ deps)"
    # setup.py keys CUDA support purely off the presence of $CUDA_PATH.
    if [ "$WANT_CUDA" = "yes" ]; then
      export CUDA_PATH="${CUDA_PATH:-/usr/local/cuda}"
      ok "CUDA_PATH=${CUDA_PATH} -> SAPIEN_CUDA=ON"
    else
      unset CUDA_PATH || true
      ok "CUDA_PATH unset -> SAPIEN_CUDA=OFF"
    fi
    ( cd "${SAPIEN_SRC}" && CMAKE_BUILD_PARALLEL_LEVEL="${JOBS}" ${PY} setup.py bdist_wheel )
  fi

  local whl
  whl="$(ls -t "${SAPIEN_SRC}"/dist/sapien-*.whl 2>/dev/null | head -1)" \
    || die "no wheel produced in ${SAPIEN_SRC}/dist"
  [ -n "$whl" ] || die "no wheel produced in ${SAPIEN_SRC}/dist"
  ${PY} -m pip install --force-reinstall "$whl"
  ok "installed $(basename "$whl")"
}

# ----------------------------------------------------------------------------
# 3. ManiSkill
# ----------------------------------------------------------------------------
# mplib has no aarch64 wheel and its sdist won't build; nothing in sim_node
# uses it. fast_kinematics DOES have aarch64 wheels, so it stays. The sapien
# pin is loosened so pip accepts our locally-built wheel.
patch_maniskill_setup() {
  local f="${MANISKILL_SRC}/setup.py"
  ${PY} - "$f" <<'PYEOF'
import sys

path = sys.argv[1]
lines = open(path).read().splitlines(keepends=True)
out, changed, saw_mplib, saw_pin = [], False, False, False

for line in lines:
    stripped = line.lstrip()
    indent = line[: len(line) - len(stripped)]

    # mplib: no aarch64 wheel and its sdist will not build. Nothing in
    # sim_node uses it, so comment the dependency out entirely.
    if stripped.startswith('"mplib=='):
        saw_mplib = True
        out.append(indent + "# no aarch64 wheel; unused by sim_node -- " + stripped)
        changed = True
        continue

    # Loosen the sapien pin so pip accepts our locally built wheel instead
    # of trying PyPI, which publishes no aarch64 build.
    if '"sapien>=3.0.0;' in line:
        saw_pin = True
        out.append(line.replace('"sapien>=3.0.0;', '"sapien>=3.0.0.dev0;'))
        changed = True
        continue

    if stripped.startswith("#") and '"mplib==' in line:
        saw_mplib = True
    if '"sapien>=3.0.0.dev0;' in line:
        saw_pin = True

    out.append(line)

if changed:
    open(path, "w").write("".join(out))
    print("PATCHED")
else:
    print("ALREADY_PATCHED")

if not saw_mplib:
    print("WARNING: no mplib dependency line found -- upstream may have changed")
if not saw_pin:
    print("WARNING: no sapien pin found -- upstream may have changed")
PYEOF
}

# On a box with no CUDA, ManiSkill falls back to sapien.Device("cpu") for the
# renderer. That device is not renderable: SAPIEN's physical-device matcher only
# understands "cuda", "cuda:N" and "pci:...", so RenderSystem() then dies with
#   RuntimeError: Failed to find a supported physical device "cpu"
# even though a perfectly good software rasterizer (llvmpipe) is present.
# Letting SAPIEN pick the device itself resolves to that rasterizer.
patch_maniskill_render_device() {
  local f="${MANISKILL_SRC}/mani_skill/envs/utils/system/backend.py"
  [ -f "$f" ] || { warn "backend.py not found -- upstream may have changed"; return; }
  ${PY} - "$f" <<'PYEOF'
import sys

path = sys.argv[1]
src = open(path).read()

if "_default_render_device" in src:
    print("ALREADY_PATCHED")
    sys.exit(0)

helper = '''
def _default_render_device():
    """Let SAPIEN pick a renderable device.

    sapien.Device("cpu") is not renderable -- SAPIEN's physical-device matcher
    only understands "cuda", "cuda:N" and "pci:...". Constructing a RenderSystem
    with no hint picks the best supported device, which on a CUDA-less box is
    the software rasterizer (llvmpipe).
    """
    try:
        return sapien.render.RenderSystem().device
    except Exception:
        return None


def parse_sim_and_render_backend('''

anchor = "\ndef parse_sim_and_render_backend("
if anchor not in src:
    print("WARNING: parse_sim_and_render_backend not found -- upstream may have changed")
    sys.exit(0)
src = src.replace(anchor, helper, 1)

# The sapien_cpu branch (non-Darwin: Darwin is handled by an earlier branch).
old_cpu = ('        elif render_backend == "sapien_cpu":\n'
           '            render_device = sapien.Device("cpu")\n')
new_cpu = ('        elif render_backend == "sapien_cpu":\n'
           '            render_device = _default_render_device()\n')
if old_cpu in src:
    src = src.replace(old_cpu, new_cpu, 1)
else:
    print("WARNING: sapien_cpu branch not found -- upstream may have changed")

# The "no CUDA device" fallback. Disambiguated from the identical two lines in
# the Darwin branch by the trailing `else:`.
old_fb = ('            render_device = sapien.Device("cpu")\n'
          '            render_backend = "sapien_cpu"\n'
          '        else:')
new_fb = ('            render_device = _default_render_device()\n'
          '            render_backend = "sapien_cpu"\n'
          '        else:')
if old_fb in src:
    src = src.replace(old_fb, new_fb, 1)
else:
    print("WARNING: CUDA fallback branch not found -- upstream may have changed")

open(path, "w").write(src)
print("PATCHED")
PYEOF
}

install_maniskill() {
  step "Installing ManiSkill"
  if [ ! -d "${MANISKILL_SRC}/.git" ]; then
    git clone https://github.com/haosulab/ManiSkill.git "${MANISKILL_SRC}"
    git -C "${MANISKILL_SRC}" checkout "${MANISKILL_COMMIT}"
  else
    ok "ManiSkill source present at ${MANISKILL_SRC}"
  fi
  patch_maniskill_setup
  patch_maniskill_render_device
  ${PY} -m pip install -e "${MANISKILL_SRC}"
  ok "ManiSkill installed (editable)"
  clear_maniskill_shadow
}

# A leftover site-packages/mani_skill/ from an old non-editable install shadows
# the editable one: setuptools APPENDS its finder to sys.meta_path, so the
# normal path finder wins and yields an empty namespace package.
clear_maniskill_shadow() {
  step "Checking for the mani_skill shadowing trap"
  local loc
  loc="$(${PY} -c 'import mani_skill; print(mani_skill.__file__)' 2>/dev/null || echo ERROR)"
  if [ "$loc" = "None" ] || [ "$loc" = "ERROR" ]; then
    local sp shadow
    sp="$(${PY} -c 'import site; print(site.getusersitepackages())')"
    shadow="${sp}/mani_skill"
    if [ -d "$shadow" ]; then
      mv "$shadow" "${shadow}.shadow.bak.$(date +%s)"
      warn "moved shadowing dir aside: ${shadow}"
      loc="$(${PY} -c 'import mani_skill; print(mani_skill.__file__)' 2>/dev/null || echo ERROR)"
    fi
  fi
  case "$loc" in
    ""|None|ERROR)
      # Not always shadowing -- show the real traceback rather than guessing.
      ${PY} -c 'import mani_skill' 2>&1 | tail -5
      fail "mani_skill still not importable (traceback above)"
      ;;
    *) ok "mani_skill -> ${loc}" ;;
  esac
}

# ----------------------------------------------------------------------------
# 4. python deps
# ----------------------------------------------------------------------------
install_pydeps() {
  step "Installing Python dependencies"
  # coacd is NOT optional: sim_node convex-decomposes the arena meshes on load.
  ${PY} -m pip install coacd pynput 'numpy>=1.22,<2'
  # aarch64 wheels exist (manylinux_2_34, glibc>=2.34); harmless if it can't install.
  ${PY} -m pip install 'fast_kinematics==0.2.2' || warn "fast_kinematics unavailable -- optional, continuing"
  ${PY} -m pip install catkin_pkg 'empy==3.3.4' lark PyYAML setuptools
  ok "python deps installed"

  # torch is normally already present here (ensure_torch runs before ManiSkill);
  # this is only a fallback for the Jetson/CUDA path.
  if ! ${PY} -c 'import torch' 2>/dev/null; then
    ${PY} -m pip install torch
  fi
  if ${PY} -c 'import torch,sys; sys.exit(0 if torch.cuda.is_available() else 1)' 2>/dev/null; then
    ok "torch has CUDA -> GPU sim available (cpu_sim:=false)"
    CPU_SIM="false"
  else
    warn "torch is CPU-only -> you MUST launch with cpu_sim:=true"
    warn "  for GPU sim on Jetson, install NVIDIA's JetPack-matched torch wheel"
    CPU_SIM="true"
  fi
}

# ----------------------------------------------------------------------------
# 5. env file
# ----------------------------------------------------------------------------
# SAPIEN's _vulkan_tricks.py probes x86-only paths and otherwise falls back to
# a bundled x86-64 libvulkan. These two vars make both probes short-circuit.
write_env_file() {
  step "Writing ${ENV_FILE}"
  cat > "${ENV_FILE}" <<EOF
# Generated by tools/setup_arm_sim.sh -- source before launching sim_node.
source /opt/ros/\${ROS_DISTRO:-${ROS_DISTRO_DEFAULT}}/setup.bash
[ -f "${WS_ROOT}/install/setup.bash" ] && source "${WS_ROOT}/install/setup.bash"
EOF

  # SAPIEN probes /usr/share/vulkan/icd.d and /usr/lib/x86_64-linux-gnu, neither
  # of which exists on ARM. Without these it loads a bundled x86-64 libvulkan.
  #
  # Only emit these when detection actually found something. Pointing
  # VK_ICD_FILENAMES at a path that does not exist is worse than leaving it
  # unset: the loader then reports ERROR_INCOMPATIBLE_DRIVER and enumerates
  # zero devices, whereas unset lets it discover the system ICDs on its own.
  if [ -n "${ICD:-}" ]; then
    echo "export VK_ICD_FILENAMES=${ICD}" >> "${ENV_FILE}"
  else
    warn "no Vulkan ICD detected -- leaving VK_ICD_FILENAMES unset for loader autodiscovery"
  fi
  if [ -n "${VKLIBDIR:-}" ]; then
    echo "export LD_LIBRARY_PATH=${VKLIBDIR}\${LD_LIBRARY_PATH:+:\${LD_LIBRARY_PATH}}" >> "${ENV_FILE}"
  fi
  # NOTE: torch/lib is deliberately NOT prepended here. torch ships its own
  # libshm.so and so does this repo's `shm` package; sourcing install/setup.bash
  # puts ours first on LD_LIBRARY_PATH and `import torch` then binds to the wrong
  # one. Prepending torch/lib globally would break the other way round for every
  # C++ node that links our shm (zero_copy_forwarder, viewer, yolox...). The
  # override is scoped to the Python sim_node process in
  # src/tr-simulation-maniskill/launch/sim_node_launch.py instead.
  ok "wrote ${ENV_FILE}"
}

# ----------------------------------------------------------------------------
# 6. ROS workspace
# ----------------------------------------------------------------------------
build_ros() {
  step "Building the ROS workspace"
  if [ "$SKIP_ROS" = "1" ]; then warn "skipped (--skip-ros)"; return; fi
  if [ ! -f "/opt/ros/${ROS_DISTRO:-$ROS_DISTRO_DEFAULT}/setup.bash" ]; then
    warn "ROS not found at /opt/ros/${ROS_DISTRO:-$ROS_DISTRO_DEFAULT} -- skipping colcon build"
    return
  fi
  # ROS's setup.bash reads AMENT_TRACE_SETUP_FILES et al. before assigning them,
  # so it trips `set -u`. Relax it for the source only, then restore.
  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO:-$ROS_DISTRO_DEFAULT}/setup.bash"
  set -u
  # Don't let a colcon/rosdep failure abort the script under set -e: verify()
  # still has useful things to say about SAPIEN after a multi-hour build, and
  # a missing `rosdep init` shouldn't cost you the whole verification pass.
  # Resolve deps only for what we actually build. src/ also holds vendored
  # camera/lidar/yolox packages whose package.xml carry keys rosdep cannot
  # resolve (`OpenCV` and `pcl` are CMake names, not rosdep keys; bboxes_ex_msgs
  # is a sibling package that isn't vendored here). Those packages are not in
  # sim_node's dependency tree, so scoping rosdep to `--packages-up-to sim_node`
  # keeps their breakage from failing the build we care about.
  #
  # python3-pynput is skipped because install_pydeps already pip-installed
  # pynput (and a newer version than apt ships), so letting rosdep apt-install
  # it is redundant -- and it would otherwise be the only step here needing sudo.
  if ( cd "${WS_ROOT}" \
       && mapfile -t PKG_PATHS < <(colcon list --packages-up-to sim_node --paths-only) \
       && [ "${#PKG_PATHS[@]}" -gt 0 ] \
       && rosdep install -i --from-paths "${PKG_PATHS[@]}" --rosdistro "${ROS_DISTRO}" -y \
            --skip-keys python3-pynput \
       && colcon build --packages-up-to sim_node ); then
    ok "sim_node built"
  else
    fail "ROS build failed -- continuing to verification (SAPIEN checks below still apply)"
  fi
}

# ----------------------------------------------------------------------------
# 7. verification
# ----------------------------------------------------------------------------
verify() {
  step "Verification checklist"
  local rc=0

  [ -n "${ICD:-}" ] && export VK_ICD_FILENAMES="${ICD}"
  [ -n "${VKLIBDIR:-}" ] && export LD_LIBRARY_PATH="${VKLIBDIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

  if command -v vulkaninfo >/dev/null && vulkaninfo --summary 2>/dev/null | grep -q deviceName; then
    ok "1. vulkan: $(vulkaninfo --summary 2>/dev/null | grep -m1 deviceName | sed 's/.*= *//')"
  else
    fail "1. vulkan: no device"; rc=1
  fi

  if out="$(${PY} -c 'import sapien; print(sapien.__version__, hasattr(sapien.render,"RenderCudaMeshComponent"))' 2>/dev/null)"; then
    ok "2. sapien import: ${out}  (version, cuda-support)"
  else
    fail "2. sapien import failed"; rc=1
  fi

  if ${PY} - <<'PYEOF' >/dev/null 2>&1
import sapien, numpy as np
s = sapien.Scene(); s.add_ground(0); s.set_ambient_light([.5]*3)
c = s.add_camera("c", 1280, 720, 1, 0.1, 10)
s.update_render(); c.take_picture()
assert np.asarray(c.get_picture("Color")).shape == (720, 1280, 4)
PYEOF
  then ok "3. headless render 1280x720"
  else fail "3. headless render failed (see tools/ARM_SETUP.md, 'Headless render')"; rc=1; fi

  loc="$(${PY} -c 'import mani_skill; print(mani_skill.__file__)' 2>/dev/null || echo ERROR)"
  if [ "$loc" = "None" ] || [ "$loc" = "ERROR" ]; then
    fail "4. mani_skill shadowed or missing (tools/ARM_SETUP.md, 'mani_skill shadowing')"; rc=1
  else
    ok "4. mani_skill -> ${loc}"
  fi

  if ${PY} - <<'PYEOF' >/dev/null 2>&1
import mani_skill.envs, gymnasium as gym
e = gym.make('PickCube-v1', num_envs=1, sim_backend='cpu', obs_mode='state')
e.reset(seed=0); e.step(e.action_space.sample()); e.close()
PYEOF
  then ok "5. maniskill cpu sim steps"
  else fail "5. maniskill cpu sim failed"; rc=1; fi

  if ${PY} -c 'import coacd' 2>/dev/null; then ok "6. coacd importable"
  else fail "6. coacd missing -- sim_node will die loading the arena"; rc=1; fi

  local torch_ok=0
  if ${PY} -c 'import torch' 2>/dev/null; then
    torch_ok=1; ok "7. torch importable ($(py_pkg_version torch))"
  else
    fail "7. torch import failed"; rc=1
  fi

  # The libshm.so collision (tools/ARM_SETUP.md, 'libshm'): with the workspace
  # sourced, `import torch` must still resolve torch's own libshm.so. The launch
  # file scopes the fix to sim_node, so this only warns. Only meaningful if
  # check 7 passed, otherwise a missing torch would be misreported as a collision.
  local ws_setup="${WS_ROOT}/install/setup.bash"
  if [ "$torch_ok" = "1" ] && [ -f "$ws_setup" ]; then
    if ( set +u; source "$ws_setup"; ${PY} -c 'import torch' ) >/dev/null 2>&1; then
      ok "8. torch imports with the workspace sourced (no libshm collision)"
    else
      warn "8. 'import torch' fails once install/setup.bash is sourced (libshm.so collision)."
      warn "   Always start the sim through sim_node_launch.py, which prepends torch/lib for that process only."
    fi
  fi

  return $rc
}

# ----------------------------------------------------------------------------
# main
# ----------------------------------------------------------------------------
detect_platform

if [ "$VERIFY_ONLY" = "1" ]; then
  verify && { printf '\n%sAll checks passed.%s\n' "${G}${BOLD}" "${N}"; exit 0; } \
         || { printf '\n%sSome checks failed -- see tools/ARM_SETUP.md.%s\n' "${R}${BOLD}" "${N}"; exit 1; }
fi

install_apt
# after install_apt (apt may provide cmake) but before any build; runs even
# under --skip-apt, since the version gates apply regardless.
step "Checking the Python packaging toolchain"
ensure_pip
ensure_setuptools
ensure_packaging
if [ "$BUILD_MODE" = "native" ]; then ensure_cmake; fi
install_sapien
# torch must be resolved before `pip install -e ManiSkill` pulls the CUDA build.
ensure_torch
install_maniskill
install_pydeps
write_env_file
build_ros

if verify; then
  cat <<EOF

${G}${BOLD}Setup complete.${N}

Launch with:

  source ${ENV_FILE}
  ros2 launch sim_node sim_node_launch.py \\
    human_gui:=true \\
    cpu_sim:=${CPU_SIM:-true} \\
    cv_resolution_x:=1280 \\
    cv_resolution_y:=720 \\
    lidar_pointcloud_resolution:=10

The reduced camera resolution is deliberate: the 1920x1200 default exhausts
unified memory on an 8GB board. First launch stalls for minutes while coacd
decomposes the arena meshes -- it is not hung.
EOF
else
  printf '\n%sSetup finished with failures -- see tools/ARM_SETUP.md, "Troubleshooting".%s\n' "${R}${BOLD}" "${N}"
  exit 1
fi
