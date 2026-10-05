import { useEffect, useState } from 'react';
import { fetchScenarios } from './api/scenarios';
import { ConnectionStatus } from './components/ConnectionStatus';
import { Radar } from './components/Radar';
import { RunControls } from './components/RunControls';
import { ScenarioSettings } from './components/ScenarioSettings';
import { Telemetry } from './components/Telemetry';
import { useSimulation } from './hooks/useSimulation';
import { defaultSettings, type Scenario } from './types/simulation';

export default function App() {
  const [settings, setSettings] = useState(defaultSettings);
  const [scenarios, setScenarios] = useState<Scenario[]>([]);
  const [catalogError, setCatalogError] = useState<string | null>(null);
  const [attempt, setAttempt] = useState(0);
  const { connection, update, error, busy, command } = useSimulation();
  useEffect(() => {
    const abort = new AbortController();
    setCatalogError(null);
    fetchScenarios(abort.signal).then(setScenarios).catch(e => { if (!abort.signal.aborted) setCatalogError(String(e)); });
    return () => abort.abort();
  }, [attempt]);
  const active = !!update && !update.engine.finished && !update.error;
  return <>
    <header><div><h1><span className="brand-mark">⊕</span> VectorWatch</h1><p>Aircraft encounters & conflict prediction</p></div><ConnectionStatus status={connection}/></header>
    {(error || update?.error) && <div className="error" role="alert">{error || update?.error}</div>}
    {connection !== 'connected' && <p className="notice">Connecting to the simulation server. Runs can reconnect for 60 seconds; expired sessions start fresh.</p>}
    <main>
      <aside className="panel settings"><form onSubmit={e => { e.preventDefault(); void command('Start', settings); }}>
        {catalogError ? <div role="alert"><p>{catalogError}</p><button type="button" onClick={() => setAttempt(x => x + 1)}>Retry scenarios</button></div> : null}
        <ScenarioSettings settings={active ? update!.settings : settings} scenarios={scenarios} disabled={active || busy || !scenarios.length} onChange={setSettings}/>
        <RunControls active={active} connected={connection === 'connected' && scenarios.length > 0} busy={busy} canReplay={!!update} onStop={() => void command('Stop')} onReplay={() => void command('Replay')}/>
        <p className="hint">Replay uses the previous run’s settings and seeds. Each tab runs independently.</p>
      </form></aside>
      <Radar engine={update?.engine}/>
      <Telemetry update={update}/>
    </main>
    <footer>20 Hz state updates · 5 Hz predictions · 120 s lookahead</footer>
  </>;
}
