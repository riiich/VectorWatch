# VectorWatch (Jan 2026)

VectorWatch is an aircraft-conflict simulation with deterministic and
Monte Carlo prediction, built in C++14. Two aircraft travel through a shared waypoint while the
engine continuously estimates loss-of-separation and physical-collision risk.
It supports a sequential reference loop and a threaded real-time pipeline while
keeping the engine independent of the terminal frontend.

All positions and distances use meters. Velocity uses m/s and time use seconds.

## Video Demonstration


https://github.com/user-attachments/assets/8bcafbd3-1b0f-4124-841a-a4bb4a570c59





## Build and run

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
ctest --test-dir build --output-on-failure
./build/vectorwatch
```

```powershell
cmake -S . -B build
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
.\build\Debug\vectorwatch.exe --help
```

Running without an argument evaluates every built-in scenario. Run one
scenario by name with:

```bash
./build/vectorwatch head-on
./build/vectorwatch parallel
./build/vectorwatch crossing
./build/vectorwatch initial-collision
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

Generate a new shared-waypoint encounter with randomized aircraft airspeed,
crossing approach headings, vertical rates, wind strength and direction, and arrival
timing on every run:

```bash
./build/vectorwatch --simulate random-encounter
```

Both nominal ground-velocity paths reach the waypoint. Collision scenarios give
the aircraft the same arrival time; pass scenarios use different arrival times
and crossing paths. This deliberately creates useful demo outcomes and is not a
model of real-world collision frequency. The terminal shows each aircraft's
original through-air velocity, actual ground-velocity vector and speed,
whether the wind acts as a headwind, tailwind, or crosswind, and an air-current
direction indicator inside the radar. Ground velocity is calculated as:

```text
ground velocity = air velocity + wind velocity
```

The radar prints the generated seed. Pass it after the speed multiplier to
replay the exact encounter:

```bash
./build/vectorwatch --simulate random-encounter --scenario-seed 10
```

Request a collision or pass construction explicitly with `--outcome collision`
or `--outcome pass`.

## Probabilistic prediction

Run the sequential Monte Carlo reference implementation with:

```bash
./build/vectorwatch --simulate random-encounter \
  --prediction probabilistic \
  --execution sequential \
  --samples 10000 \
  --scenario-seed 42 \
  --uncertainty-seed 99 \
  --uncertainty medium
```

Each sample perturbs horizontal position, altitude, airspeed, heading, vertical
rate, and shared wind, then runs the deterministic detectors over the same
120-second lookahead. Samples use truncated Gaussian demonstration profiles;
they are not calibrated aviation measurement models.

The risk banner maps estimated loss-of-separation probability as follows:

```text
below 5%       LOW RISK
5% to 69.9%   POTENTIAL CONFLICT
70% or above  HIGHLY LIKELY CONFLICT
```

Physical-collision probability is displayed separately. Only collision along
the nominal simulated trajectory triggers the explosion.

## Collision handling

Conflict prediction and physical collision are separate concepts. A conflict
uses the configurable horizontal and vertical separation thresholds described
below. A collision occurs when the aircraft centers come within 50 meters in
3D space. This is a project-specific simulation value, not an aircraft dimension
or an operational aviation standard.

Before applying each movement update, the simulation checks the complete motion
segment for a collision. This prevents accelerated simulation steps from
skipping over an impact. On collision, the terminal plays a short ASCII
explosion animation with orange particles at the impact position, reports
`Status: CRASHED`, and ends the simulation immediately. Every collision path
uses the same reusable explosion-frame renderer.

## CPA (Closest Point of Approach) model

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

## Web interface

The React + ASP.NET Core 10 MVP reuses the C++ engine through a native bridge. It supports built-in and seeded random encounters, deterministic/probabilistic predictions, independent browser tabs, Start/Stop/Replay and live SVG radar/telemetry. See [web/README.md](web/README.md) for prerequisites, local startup, native library loading and integration checks.
