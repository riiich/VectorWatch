// Builds must exist first. Starts an isolated server and closes it after the checks.
import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { createRequire } from 'node:module';
import { fileURLToPath } from 'node:url';
import path from 'node:path';
import { setTimeout as delay } from 'node:timers/promises';
const root = fileURLToPath(new URL('../../', import.meta.url));
const require = createRequire(new URL('../client/package.json', import.meta.url));
const { HubConnectionBuilder, HttpTransportType, LogLevel } = require('@microsoft/signalr');
const port = process.env.VECTORWATCH_TEST_PORT ?? '5081';
const url = `http://127.0.0.1:${port}`;
const library = process.platform === 'win32' ? 'vectorwatch_native.dll' : process.platform === 'darwin' ? 'libvectorwatch_native.dylib' : 'libvectorwatch_native.so';
let logs = '';
const server = spawn('dotnet', ['bin/Debug/net10.0/VectorWatch.Web.dll', '--urls', url], {
  cwd: path.join(root, 'web/server'),
  env: { ...process.env, VECTORWATCH_NATIVE_PATH: process.env.VECTORWATCH_NATIVE_PATH ?? path.join(root, 'build', library),
    Runs__ReconnectGraceSeconds: '2', Logging__LogLevel__Default: 'Warning', Logging__LogLevel__VectorWatch: 'Debug' },
  stdio: ['ignore', 'pipe', 'pipe'],
});
server.stdout.on('data', data => { logs += data; });
server.stderr.on('data', data => { logs += data; });
const clients = [];
const defaults = { scenario: 'head-on', speed: 1000, seed: null, prediction: 'deterministic', samples: 1000, workers: 2, uncertaintySeed: 1, uncertainty: 'medium', outcome: 'any' };
async function until(check, timeout = 8000) {
  const deadline = Date.now() + timeout;
  while (Date.now() < deadline) { const value = await check(); if (value) return value; await delay(20); }
  throw new Error('Timed out waiting for expected state');
}
async function client(tab = crypto.randomUUID()) {
  const hub = new HubConnectionBuilder().withUrl(`${url}/hubs/simulation`, { transport: HttpTransportType.WebSockets, skipNegotiation: true }).configureLogging(LogLevel.None).build();
  const updates = [];
  hub.on('RunUpdated', value => updates.push(value));
  clients.push(hub);
  await hub.start();
  const restored = await hub.invoke('Join', tab);
  return { hub, tab, updates, restored,
    start: options => hub.invoke('Start', tab, { ...defaults, ...options }),
    final: id => until(() => updates.find(u => u.runId === id && u.engine.finished)),
  };
}
try {
  await until(async () => {
    if (server.exitCode !== null) throw new Error(logs);
    try { return (await fetch(`${url}/api/scenarios`)).ok; } catch { return false; }
  });
  assert.equal((await (await fetch(`${url}/api/scenarios`)).json()).length, 12);
  const a = await client();
  await assert.rejects(a.start({ scenario: 'missing' }), /Unknown scenario/);
  await assert.rejects(a.start({ speed: -1 }), /Invalid settings/);
  const collision = await a.start({});
  assert.equal((await a.final(collision.runId)).engine.result.reason, 'collision');
  const pass = await a.start({ scenario: 'parallel' });
  const completed = await a.final(pass.runId);
  assert.equal(completed.engine.result.reason, 'completed');
  assert.equal(completed.engine.snapshot.sequence, completed.engine.prediction.snapshotSequence);
  assert.equal(completed.engine.snapshot.simulationTimeSeconds, completed.engine.prediction.simulationTimeSeconds);
  const random = await a.start({ scenario: 'random-encounter', seed: 8, outcome: 'pass', prediction: 'probabilistic' });
  const randomFinal = await a.final(random.runId);
  assert.equal(randomFinal.engine.prediction.sampleCount, 1000);
  assert.equal(randomFinal.engine.result.reason, 'completed');
  const replay = await a.hub.invoke('Replay', a.tab);
  assert.notEqual(replay.runId, random.runId);
  assert.deepEqual(replay.settings, random.settings);
  assert.deepEqual(replay.engine.initialAircraft, random.engine.initialAircraft);
  assert.deepEqual(replay.engine.wind, random.engine.wind);
  await a.final(replay.runId);
  const generated = await a.start({ scenario: 'random-encounter' });
  assert.equal(generated.settings.seed, generated.engine.seed);
  assert.ok(Number.isInteger(generated.settings.seed));
  await a.final(generated.runId);
  const b = await client();
  const slow = await a.start({ scenario: 'parallel', speed: .1 });
  const other = await b.start({ scenario: 'head-on', speed: .1 });
  await assert.rejects(a.start({}), /Stop the current run/);
  await a.hub.invoke('Stop', a.tab, slow.runId);
  assert.equal((await a.final(slow.runId)).engine.result.reason, 'stopped');
  await until(() => b.updates.some(u => u.runId === other.runId && u.engine.snapshot?.sequence > 2));
  assert.ok(b.updates.every(u => u.runId !== slow.runId));
  assert.ok(!b.updates.at(-1).engine.finished);
  await b.hub.stop();
  const reconnected = await client(b.tab);
  assert.equal(reconnected.restored.runId, other.runId);
  await reconnected.hub.invoke('Stop', reconnected.tab, crypto.randomUUID());
  await delay(100);
  assert.ok(!reconnected.updates.at(-1).engine.finished, 'Stale Stop must not stop a new run');
  await reconnected.hub.stop();
  await until(() => logs.includes(`Released abandoned session ${b.tab}`), 5000);
  const expired = await client(b.tab);
  assert.equal(expired.restored, null, 'Expired session must release its native run and settings');
  const newRun = await expired.start({ scenario: 'parallel' });
  await expired.final(newRun.runId);
  // Leave an active Monte Carlo run for graceful server shutdown to cancel and join.
  await a.start({ scenario: 'parallel', speed: .1, prediction: 'probabilistic', samples: 100000 });
  console.log('SignalR: catalog, validation, collision, completion, probabilities, replay, independent tabs, stale Stop, reconnect and abandoned-run cleanup passed.');
} catch (error) {
  console.error(logs);
  throw error;
} finally {
  await Promise.all(clients.map(c => c.stop()));
  server.kill('SIGTERM');
  await until(() => server.exitCode !== null || server.signalCode !== null, 5000);
  assert.equal(server.exitCode, 0, 'Server must shut down cleanly with an active native run');
}
