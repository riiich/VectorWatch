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
ctest --test-dir build --output-on-failure
./build/vectorwatch
```

Running without an argument evaluates every built-in scenario. Run one
scenario by name with:

```bash
./build/vectorwatch head-on
./build/vectorwatch parallel
./build/vectorwatch crossing
./build/vectorwatch different-altitudes
./build/vectorwatch near-miss
./build/vectorwatch staggered-crossing
./build/vectorwatch offset-convergence
./build/vectorwatch vertical-convergence
./build/vectorwatch diverging
./build/vectorwatch outside-lookahead
```

Use `./build/vectorwatch --help` to list the available scenarios. A successful
run exits with status 0. A scenario that disagrees with its expected result
exits with status 1.

Use the dedicated scenario catalog for an at-a-glance list with descriptions:

```bash
./build/vectorwatch --list-scenarios
```

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

## Collision handling

Conflict prediction and physical collision are separate concepts. A conflict
uses the configurable horizontal and vertical separation thresholds described
below. A collision occurs when the aircraft centers come within 50 meters in
3D space. This is a project-specific simulation value, not an aircraft dimension
or an operational aviation standard.

Before applying each movement update, the simulation checks the complete motion
segment for a collision. This prevents accelerated simulation steps from
skipping over an impact. On collision, the terminal plays a short ASCII
explosion animation at the impact position and ends the simulation immediately.

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
120-second lookahead for CPA reporting. It calculates horizontal and vertical
separation separately at that time.

Conflict classification does not require the aircraft to meet at one exact
coordinate or cross both thresholds at the reported 3D CPA. The detector solves
for the future interval during which horizontal separation is below its
threshold and the interval during which vertical separation is below its
threshold. A conflict exists when those intervals overlap inside the lookahead:

```text
horizontal separation < 1000 m
vertical separation   < 150 m
```

These values are simulation settings, not operational aviation standards.

This interval approach handles near misses, different arrival times, vertical
convergence, and cases where the minimum 3D distance is not itself inside both
thresholds. When relative velocity is zero or extremely small, division by zero
is avoided. The aircraft keep their current separation, so the current instant
is used as their CPA.

## Project layout

```text
CMakeLists.txt
include/vectorwatch/
  app/
    Application.hpp
  detection/
    CollisionDetector.hpp
    ConflictDetector.hpp
  math/
    Vector3.hpp
  model/
    Aircraft.hpp
  scenarios/
    Scenario.hpp
    ScenarioCatalog.hpp
  simulation/
    TerminalRadarRenderer.hpp
    TerminalSimulation.hpp
    TerminalSimulationOptions.hpp
src/
  app/Application.cpp
  detection/CollisionDetector.cpp
  detection/ConflictDetector.cpp
  main.cpp
  model/Aircraft.cpp
  scenarios/ScenarioCatalog.cpp
  simulation/TerminalRadarRenderer.cpp
  simulation/TerminalSimulation.cpp
tests/
  CollisionDetectorTests.cpp
  ConflictDetectorTests.cpp
```

- `Vector3` supplies the vector operations needed by the CPA calculation.
- `Aircraft` owns an ID, position, velocity, and the basic position update.
- `ConflictDetector` performs relative-motion and CPA calculations.
- `CollisionDetector` detects physical impacts across simulation updates.
- `ScenarioCatalog` owns the built-in scenario definitions and lookup.
- `TerminalSimulation` owns the timed update loop.
- `TerminalRadarRenderer` owns the ASCII radar presentation.
- `Application` handles CLI commands and coordinates the other components.
- `main.cpp` is the minimal executable entry point.

## Current scope

This version intentionally has no web framework, GoogleTest, JSON dependency,
conflict lifecycle, spatial grid, or worker threads. It currently simulates one
aircraft pair at a time. SFML is not planned; the terminal remains the working
interface until the engine is ready for a browser visualization. Scenario
loading and simulation of many aircraft should come before that web interface.
