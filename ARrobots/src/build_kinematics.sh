#!/usr/bin/env bash
# Build the compiled kinematics module (robot_kinematics.*.so / .pyd) from kinematics.cpp +
# bindings.cpp into ../ (the ARrobots package), for the Python that runs AR4.py.
#
#   cd ARrobots/src && ./build_kinematics.sh            # uses ../../.venv/bin/python if present
#   PYTHON=/path/to/python ./build_kinematics.sh
#
# Needs: a C++14 compiler, pybind11 (pip install pybind11) and the Python headers
# (python3-dev / python3.x-devel). Uses CMake + Ninja when they are installed, otherwise
# compiles directly with the compiler (no CMake needed).
#
# The solver is shared with the Teensy firmware (sketches/AR4_teensy41_sketch_v8): when
# kinematics.cpp changes, change the sketch the same way and bump API_VERSION in
# bindings.cpp. Without a rebuild AR4.py notices the stale module and falls back to
# ARrobots/robot_kinematics_py.py (same solver in pure Python, slower).
set -euo pipefail
cd "$(dirname "$0")"

PYTHON="${PYTHON:-}"
if [ -z "$PYTHON" ]; then
  if [ -x ../../.venv/bin/python ]; then PYTHON=../../.venv/bin/python; else PYTHON=python3; fi
fi

if command -v cmake >/dev/null && command -v ninja >/dev/null; then
  rm -rf build CMakeFiles CMakeCache.txt
  mkdir -p build
  cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
    -DPYTHON_EXECUTABLE="$("$PYTHON" -c 'import sys; print(sys.executable)')" \
    -Dpybind11_DIR="$("$PYTHON" -m pybind11 --cmakedir)"
  cmake --build build --parallel
  cp build/robot_kinematics*.so ../ 2>/dev/null || cp build/Release/robot_kinematics*.pyd ../
else
  SUFFIX="$("$PYTHON" -c 'import sysconfig; print(sysconfig.get_config_var("EXT_SUFFIX"))')"
  INCLUDES="$("$PYTHON" -m pybind11 --includes)"
  CXX="${CXX:-c++}"
  echo "cmake/ninja not found: compiling directly with $CXX -> ../robot_kinematics$SUFFIX"
  # shellcheck disable=SC2086
  "$CXX" -O3 -shared -std=c++14 -fPIC $INCLUDES ${CXXFLAGS:-} bindings.cpp -o "../robot_kinematics$SUFFIX"
fi
"$PYTHON" -c 'import sys; sys.path.insert(0, ".."); sys.path.insert(0, "../.."); import ARrobots.robot_kinematics as k; print("built robot_kinematics API", k.API_VERSION)'
