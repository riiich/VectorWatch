import type { RunSettings, Scenario } from '../types/simulation';
interface Props { settings: RunSettings; scenarios: Scenario[]; disabled: boolean; onChange: (value: RunSettings) => void }
export function ScenarioSettings({ settings: s, scenarios, disabled, onChange }: Props) {
  const set = <K extends keyof RunSettings>(key: K, value: RunSettings[K]) => onChange({ ...s, [key]: value });
  return <fieldset disabled={disabled}>
    <legend>Scenario settings</legend>
    <label>Scenario<select value={s.scenario} onChange={e => set('scenario', e.target.value)}>
      {scenarios.map(s => <option key={s.name} value={s.name}>{s.name}</option>)}
    </select></label>
    <p className="hint description">{scenarios.find(x => x.name === s.scenario)?.description}</p>
    <label>Speed multiplier<input type="number" min="0.1" max="1000" step="0.1" required value={s.speed} onChange={e => set('speed', e.target.valueAsNumber)} /></label>
    {s.scenario === 'random-encounter' && <>
      <label>Scenario seed<input type="number" min="0" max="4294967295" step="1" placeholder="Generate a seed" value={s.seed ?? ''} onChange={e => set('seed', e.target.value === '' ? null : e.target.valueAsNumber)} /></label>
      <label>Requested outcome<select value={s.outcome} onChange={e => set('outcome', e.target.value as RunSettings['outcome'])}>
        <option value="any">Any</option><option value="collision">Collision</option><option value="pass">Pass</option>
      </select></label>
    </>}
    <label>Prediction<select value={s.prediction} onChange={e => set('prediction', e.target.value as RunSettings['prediction'])}>
      <option value="deterministic">Deterministic</option><option value="probabilistic">Probabilistic</option>
    </select></label>
    {s.prediction === 'probabilistic' && <>
      <label>Samples<input type="number" required min="1" max="100000" step="1" value={s.samples} onChange={e => set('samples', e.target.valueAsNumber)} /></label>
      <label>Uncertainty<select value={s.uncertainty} onChange={e => set('uncertainty', e.target.value as RunSettings['uncertainty'])}>
        <option value="low">Low</option><option value="medium">Medium</option><option value="high">High</option>
      </select></label>
      <label>Uncertainty seed<input type="number" required min="0" max="4294967295" step="1" value={s.uncertaintySeed} onChange={e => set('uncertaintySeed', e.target.valueAsNumber)} /></label>
      <label>Prediction workers<input type="number" required min="0" max="16" step="1" value={s.workers} onChange={e => set('workers', e.target.valueAsNumber)} /></label>
      <p className="hint">0 uses the prediction coordinator without a worker pool.</p>
    </>}
    <p className="hint">Settings are fixed during a run. Distances use meters; velocities use m/s.</p>
  </fieldset>;
}
