"""Exercise the exported C ABI (no third-party Python dependencies)."""
import ctypes as c
import json
import sys
import time

lib = c.CDLL(sys.argv[1])
class Options(c.Structure):
    _fields_ = [('scenario', c.c_char_p), ('speed', c.c_double), ('seed', c.c_uint32),
                ('uncertainty_seed', c.c_uint32), ('has_seed', c.c_int32),
                ('probabilistic', c.c_int32), ('samples', c.c_int32), ('workers', c.c_int32),
                ('uncertainty', c.c_int32), ('outcome', c.c_int32)]
lib.vw_create.argtypes = [c.POINTER(Options), c.POINTER(c.c_void_p)]
lib.vw_create.restype = c.c_void_p
lib.vw_read.argtypes = [c.c_void_p, c.POINTER(c.c_void_p)]
lib.vw_read.restype = c.c_void_p
lib.vw_scenarios.argtypes = [c.POINTER(c.c_void_p)]
lib.vw_scenarios.restype = c.c_void_p
for name in ['vw_stop', 'vw_destroy', 'vw_free_string']:
    getattr(lib, name).argtypes = [c.c_void_p]
    getattr(lib, name).restype = None

def consume(pointer):
    assert pointer
    try:
        return c.string_at(pointer).decode()
    finally:
        lib.vw_free_string(pointer)

def create(scenario, speed=1000000, seed=None, probabilistic=0, outcome=0):
    error = c.c_void_p()
    options = Options(scenario.encode(), speed, seed or 0, 1, seed is not None, probabilistic, 1000, 2, 1, outcome)
    handle = lib.vw_create(c.byref(options), c.byref(error))
    assert handle, consume(error.value) if error.value else 'Missing handle'
    return handle

def read(handle):
    error = c.c_void_p()
    value = lib.vw_read(handle, c.byref(error))
    assert value, consume(error.value)
    return json.loads(consume(value))

def finish(handle):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        view = read(handle)
        if view['finished']:
            return view
        time.sleep(.01)
    raise AssertionError('Native run did not finish')

def run(name, **kwargs):
    handle = create(name, **kwargs)
    try:
        initial = read(handle)
        final = finish(handle)
        assert final['radarBounds'] == initial['radarBounds'], 'Radar must not pan or zoom during a run'
        assert final['waypoint'] == initial['waypoint'], 'Target world position must remain fixed'
        return final
    finally:
        lib.vw_destroy(handle)

error = c.c_void_p()
assert len(json.loads(consume(lib.vw_scenarios(c.byref(error))))) == 12
assert not lib.vw_create(None, c.byref(error))
assert 'Invalid' in consume(error.value)
assert not lib.vw_read(None, c.byref(error))
assert 'Missing' in consume(error.value)
assert run('initial-collision')['result']['reason'] == 'collision'
assert run('head-on')['result']['reason'] == 'collision'
assert run('parallel')['result']['reason'] == 'completed'
first = run('random-encounter', seed=8, outcome=2, probabilistic=1)
replay = run('random-encounter', seed=8, outcome=2, probabilistic=1)
assert first['initialAircraft'] == replay['initialAircraft']
assert first['wind'] == replay['wind']
assert first['result']['reason'] == replay['result']['reason'] == 'completed'
assert first['prediction']['sampleCount'] == 1000
assert first['prediction']['snapshotSequence'] == first['snapshot']['sequence']
# Identical initial snapshots and uncertainty seeds give identical Monte Carlo counts.
initial_predictions = []
for _ in range(2):
    handle = create('near-miss', speed=.1, probabilistic=1)
    try:
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            view = read(handle)
            if view['prediction']:
                assert view['prediction']['snapshotSequence'] == 0
                initial_predictions.append(view['prediction'])
                break
            time.sleep(.001)
        else:
            raise AssertionError('Missing initial prediction')
    finally:
        lib.vw_destroy(handle)
assert initial_predictions[0]['conflictCount'] == initial_predictions[1]['conflictCount']
assert initial_predictions[0]['collisionCount'] == initial_predictions[1]['collisionCount']
handle = create('parallel', speed=.1)
try:
    lib.vw_stop(handle)
    assert finish(handle)['result']['reason'] == 'stopped'
finally:
    lib.vw_destroy(handle)
# Dispose active worker pools repeatedly; failure to join leaks threads/hangs shutdown.
for _ in range(20):
    handle = create('random-encounter', speed=.1, probabilistic=1)
    lib.vw_destroy(handle)
# Stop after completion must also finish if it cancels the final prediction.
handle = create('parallel', probabilistic=1)
try:
    time.sleep(.055)
    lib.vw_stop(handle)
    assert finish(handle)['finished']
finally:
    lib.vw_destroy(handle)
print('Native ABI: catalog, errors, collision, completion, seeds, predictions, Stop and cleanup passed.')
