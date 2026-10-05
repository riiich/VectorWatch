# VectorWatch web MVP

A React/TypeScript interface to the existing C++ simulation, served by ASP.NET Core 10 and streamed over SignalR WebSockets. Risk classification, conflict/collision detection, random scenarios, Monte Carlo sampling and simulation state all come from the C++ engine.

## Prerequisites

- CMake 3.20+, a C++14 compiler and native threads
- .NET 10 SDK
- Node.js 22 and npm
- Python 3 for the native integration checks

## Local startup

Run from the repository root, on Linux:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --parallel
npm --prefix web/client ci
npm --prefix web/client run build
dotnet build web/server
export VECTORWATCH_NATIVE_PATH="$PWD/build/libvectorwatch_native.so"
dotnet run --no-build --project web/server --urls http://127.0.0.1:5080
```

Open **http://127.0.0.1:5080**. The Vite build writes generated files into `web/server/wwwroot`; ASP.NET Core serves them. Build the client before `dotnet publish` if you need a local publish folder. No deployment is configured.

For client development, leave the server running and open a second terminal:

```bash
npm --prefix web/client run dev
```

Open **http://127.0.0.1:5173**. Vite proxies `/api` and `/hubs` (including WebSockets) to port 5080. Client edits reload through Vite; rebuild/restart the server or native library after changing their sources. Stop the server before replacing a loaded native library, particularly on Windows.

### Native library loading

`VECTORWATCH_NATIVE_PATH` is the absolute filename of the library, not its directory. The server validates loading at startup. Alternatively, place the library beside `VectorWatch.Web.dll`, or configure the platform's normal shared-library search path.

- Linux: `build/libvectorwatch_native.so`
- macOS: `build/libvectorwatch_native.dylib`
- Windows/MSVC multi-configuration build: `build/Debug/vectorwatch_native.dll`; build with `cmake --build build --config Debug`. In PowerShell: `$env:VECTORWATCH_NATIVE_PATH = (Resolve-Path build/Debug/vectorwatch_native.dll).Path`.

The library and .NET process must target the same architecture. Use a Python interpreter compatible with your compiler's C++ runtime for ABI tests; on Linux, `/usr/bin/python3` avoids an older Conda `libstdc++` taking precedence. The server depends only on the ASP.NET Core shared framework; there are no extra NuGet packages.

## Using the interface

Choose a built-in scenario or `random-encounter`, configure settings and select **Start**. Settings cannot change during a run. **Stop** requests native cancellation; the last snapshot remains visible. **Replay** starts a new run with the previous run's settings and resolved seeds, even if the settings form has since changed. A blank scenario seed generates a new one on Start; the resolved seed appears in telemetry.

Defaults match the existing engine: 5× speed, 60 simulated seconds for built-ins, 120 seconds plus encounter bounds for random scenarios, 20 Hz updates, 5 Hz predictions, a 120-second lookahead, 10,000 probabilistic samples, medium uncertainty and uncertainty seed 1. Zero prediction workers uses the prediction coordinator without a worker pool, preserving the engine's default. Simulation and prediction coordination still use the threaded pipeline. Web inputs cap speed at 1000×, samples at 100,000 and workers at 16.

Positions, altitude and separation are in meters; velocity and wind are in m/s. The radar is a top view with a fixed center and scale for each run, using the same bounds as the terminal renderer. Dashed lines and CPA markers come from the native prediction's own snapshot. Telemetry explicitly labels prediction time and age because simulation and prediction progress independently. A loss of separation is not necessarily a physical collision. All displayed risk and collision decisions come from the engine.

Replay reproduces the encounter, wind, settings and random seeds. The existing pipeline advances using wall-clock deltas, so frame times and intermediate Monte Carlo results are not promised to be bit-for-bit identical between runs. The same snapshot and uncertainty seed produce reproducible prediction counts. No new simulation clock or prediction algorithm was introduced.

## Sessions and cleanup

Each page instance has a fresh tab identity in memory, including reloads and duplicated tabs. A temporary connection loss retains its run for 60 seconds; automatic reconnect rejoins it and retrieves the latest state. A disconnected run continues during that grace period. Expired sessions are removed and their native threads are cancelled and joined. A page reload starts a new session; its old run is abandoned and cleaned up after the grace period.

Completed, stopped, collided or failed runs release the native handle immediately; only their last view and replay settings remain while the page is connected. Server shutdown disposes all active runs. A single service serializes native-handle access and bounds sessions to 128 and concurrent runs to 16. `Runs__ReconnectGraceSeconds` overrides the grace period for tests. This is a local MVP with no authentication or persistence; restarting the server clears all sessions.

`GET /api/scenarios` lists the native catalog. `/hubs/simulation` exposes `Join(tabId)`, `Start(tabId, settings)`, `Stop(tabId, runId)` and `Replay(tabId)`, and emits `RunUpdated`. Updates carry a run ID, immutable settings, a send timestamp and the native JSON view. The client ignores older updates and buffers updates that arrive before a Start/Replay response. Stop includes the run ID so a delayed Stop cannot affect a later run.

The C ABI is documented in `include/vectorwatch/bridge/VectorWatchBridge.h`. Strings have explicit ownership and errors never cross the ABI as C++ exceptions. The .NET wrapper uses `SafeHandle`; disposal joins native threads. `ScenarioSetup` shares random encounter duration, wind, waypoint and bounds between the terminal and web applications.

## Verification

After building:

```bash
ctest --test-dir build --output-on-failure
python3 web/tests/native_integration.py "$VECTORWATCH_NATIVE_PATH"
npm --prefix web/client run test:integration
```

On Linux, substitute `/usr/bin/python3` if your default interpreter is Conda. The integration command starts and shuts down its own server on port 5081 with a two-second reconnect grace period; `VECTORWATCH_TEST_PORT` changes that port. It requires the Debug .NET build above. `VECTORWATCH_NATIVE_PATH` can point to any matching native build.

Checks cover native error handling, seeded generation, initial-snapshot prediction repeatability, collision/non-collision completion, Stop, native destruction, SignalR streaming, server validation, replay, independent sessions, stale Stop, reconnect, expired-session cleanup and graceful shutdown with an active Monte Carlo run.

For browser verification, run `head-on` and `parallel` at 100×, confirm collision and completed states, start a slow scenario and Stop, then use seeded `random-encounter` in probabilistic mode and Replay. Open two pages, run slow scenarios and stop one while the other continues. The UI should show distinct run IDs, disabled settings during a run, prediction timestamps and no console errors.

Implementation references: [SignalR JavaScript client and reconnect behavior](https://learn.microsoft.com/en-us/aspnet/core/signalr/javascript-client?view=aspnetcore-10.0), [Vite build output](https://vite.dev/guide/static-deploy).
