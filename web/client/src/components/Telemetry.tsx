import type { RunUpdate } from '../types/simulation';
export function Telemetry({ update }: { update: RunUpdate | null }) {
  const engine = update?.engine; const p = engine?.prediction; const s = engine?.snapshot;
  const status = update?.error ? 'error' : engine?.result?.reason ?? (engine ? 'running' : 'ready');
  return <section className="panel telemetry">
    <div className="panel-heading"><h2>Telemetry</h2><strong className={`status ${status}`} role="status">{status}</strong></div>
    <dl>
      <dt>Simulation time</dt><dd>{s?.simulationTimeSeconds.toFixed(2) ?? '0.00'} / {engine?.durationSeconds ?? 60} s</dd>
      <dt>Risk level</dt><dd>{p?.riskLevel ?? 'Awaiting prediction'}</dd>
      <dt>Nominal conflict</dt><dd>{p ? p.approach.conflict ? 'YES' : 'NO' : '—'}</dd>
      <dt>Predicted collision</dt><dd>{p ? p.nominalCollision ? `In ${p.nominalCollision.timeSeconds.toFixed(2)} s` : 'None in lookahead' : '—'}</dd>
      <dt>3D separation at prediction</dt><dd>{p ? `${p.approach.currentDistanceMeters.toFixed(0)} m` : '—'}</dd>
      <dt>Time to CPA</dt><dd>{p ? `${p.approach.timeSeconds.toFixed(2)} s` : '—'}</dd>
      <dt>Horizontal / vertical CPA</dt><dd>{p ? `${p.approach.horizontalSeparationMeters.toFixed(0)} / ${p.approach.verticalSeparationMeters.toFixed(0)} m` : '—'}</dd>
      <dt>CPA inside lookahead</dt><dd>{p ? p.approach.isWithinLookahead ? 'Yes' : 'No' : '—'}</dd>
      <dt>Conflict window</dt><dd>{p?.approach.conflictWindow.exists ? `${p.approach.conflictWindow.startSeconds.toFixed(1)}–${p.approach.conflictWindow.endSeconds.toFixed(1)} s` : 'None'}</dd>
      {update?.settings.prediction === 'probabilistic' && <>
        <dt>Conflict probability</dt><dd>{p ? `${(p.conflictProbability * 100).toFixed(1)}%` : '—'}</dd>
        <dt>Collision probability</dt><dd>{p ? `${(p.collisionProbability * 100).toFixed(1)}%` : '—'}</dd>
        <dt>Samples / workers</dt><dd>{p ? `${p.sampleCount.toLocaleString()} / ${p.workerCount}` : '—'}</dd>
      </>}
    </dl>
    {p && <p className={`prediction-age ${s && p.snapshotSequence !== s.sequence ? 'stale' : ''}`}>
      <span>Prediction at {p.simulationTimeSeconds.toFixed(2)} s</span>
      <span>Snapshot #{p.snapshotSequence}</span>
      <span>{s && p.snapshotSequence !== s.sequence ? `Snapshot is ${Math.max(0, s.simulationTimeSeconds - p.simulationTimeSeconds).toFixed(3)} s older` : 'Matches current snapshot'}</span>
      <span>{p.calculationMilliseconds.toFixed(1)} ms compute</span>
    </p>}
    {s?.aircraft.map(a => <div className="aircraft-telemetry" key={a.id}><strong>Aircraft {a.id}</strong><p>Position: {[a.position.x, a.position.y, a.position.z].map(v => v.toFixed(0)).join(', ')} m</p><p>Velocity: {[a.velocity.x, a.velocity.y, a.velocity.z].map(v => v.toFixed(1)).join(', ')} m/s</p></div>)}
    {update && <div className="run-meta"><p>Scenario: {update.settings.scenario} · {update.settings.speed}×</p><p>Scenario seed: {engine?.seed ?? 'Built-in'} · Uncertainty seed: {update.settings.uncertaintySeed}</p><p>Run: <code>{update.runId}</code></p></div>}
  </section>;
}
