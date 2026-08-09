# VectorWatch

VectorWatch 0.2 is a C++20 terminal simulation for predicting the closest point
of approach (CPA) between two aircraft. It includes a real-time update loop and
an ASCII radar while keeping the simulation independent of any graphics or web
framework.

All positions and distances use meters. Velocities use meters per second and
times use seconds.

## Build and run

Requirements:

- Linux
- CMake 3.20 or newer
- A C++20 compiler such as GCC 10 or newer or Clang 10 or newer

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
./build/vectorwatch
```

Running without an argument evaluates every built-in scenario. Run one
scenario by name with:

```bash
./build/vectorwatch head-on
./build/vectorwatch parallel
./build/vectorwatch crossing
./build/vectorwatch different-altitudes
```

Use `./build/vectorwatch --help` to list the available scenarios. A successful
run exits with status 0. A scenario that disagrees with its expected result
exits with status 1.

## Terminal simulation

Start the head-on simulation with:

```bash
./build/vectorwatch --simulate head-on
```

The terminal redraws at 20 Hz and shows aircraft positions, projected paths,
the predicted CPA point, current separation, TCPA, and conflict status. The
default speed is 5x, so the 60 simulated seconds complete in approximately 12
real seconds.

Pass a different speed as the final argument:

```bash
./build/vectorwatch --simulate crossing 1
./build/vectorwatch --simulate different-altitudes 10
```

At 1x, one real second equals one simulated second. Higher values accelerate
the same simulation calculations; they do not skip directly to a stored result.
Use `Ctrl+C` to stop a running simulation early.

## CPA model

For aircraft A and B, the detector first changes the problem into relative
motion:

```text
relative position r = positionB - positionA
relative velocity v = velocityB - velocityA
```

At a future time `t`, their separation vector is:

```text
r(t) = r + v * t
```

The squared distance is smallest when its rate of change is zero. Solving that
condition gives:

```text
TCPA = -(r dot v) / (v dot v)
```

The implementation clamps this time to the interval from now through the
120-second lookahead. It then calculates the horizontal and vertical
separations separately at that time. A result is classified as a conflict only
when the original TCPA lies inside the lookahead and both project-specific
thresholds are crossed:

```text
horizontal separation < 1000 m
vertical separation   < 150 m
```

These values are simulation settings, not operational aviation standards.

When relative velocity is zero or extremely small, division by zero is avoided.
The aircraft keep their current separation, so the current instant is used as
their CPA.

## Project layout

```text
CMakeLists.txt
include/vectorwatch/
  Aircraft.hpp
  ConflictDetector.hpp
  TerminalSimulation.hpp
  Vector3.hpp
src/
  ConflictDetector.cpp
  TerminalSimulation.cpp
  main.cpp
```

- `Vector3` supplies the vector operations needed by the CPA calculation.
- `Aircraft` owns an ID, position, velocity, and the basic position update.
- `ConflictDetector` performs relative-motion and CPA calculations.
- `TerminalSimulation` owns the timed loop and ASCII radar rendering.
- `main.cpp` contains the CLI and four intentionally hardcoded scenarios.

## Current scope

This version intentionally has no SFML, web framework, GoogleTest, JSON
dependency, conflict lifecycle, spatial grid, or threads. It currently
simulates one aircraft pair at a time. Scenario loading and simulation of many
aircraft should come before the browser visualization.
