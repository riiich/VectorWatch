interface Props { active: boolean; connected: boolean; busy: boolean; canReplay: boolean; onStop: () => void; onReplay: () => void }
export function RunControls({ active, connected, busy, canReplay, onStop, onReplay }: Props) {
  return <div className="controls">
    <button type="submit" disabled={active || !connected || busy}>Start</button>
    <button type="button" className="secondary" disabled={!active || !connected || busy} onClick={onStop}>Stop</button>
    <button type="button" className="secondary" disabled={active || !connected || busy || !canReplay} onClick={onReplay}>Replay</button>
  </div>;
}
