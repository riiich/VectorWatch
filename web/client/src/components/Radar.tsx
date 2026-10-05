import type { EngineView, Vec3 } from '../types/simulation';
export function Radar({ engine }: { engine?: EngineView }) {
  if (!engine) return <section className="panel radar empty"><div className="empty-radar">＋</div><h2>Ready to track</h2><p>Choose a scenario and start a run.</p></section>;
  const aircraft = engine.snapshot?.aircraft ?? engine.initialAircraft;
  const paths = engine.prediction?.paths ?? [];
  // Native bounds are fixed at run creation, so stationary world points stay
  // stationary on screen as aircraft and prediction endpoints advance.
  const { minimumX: minX, maximumX: maxX, minimumY: minY, maximumY: maxY } = engine.radarBounds;
  const span = Math.max(maxX - minX, maxY - minY);
  const x = (p: Vec3) => 350 + (p.x - (maxX + minX) / 2) * 600 / span;
  const y = (p: Vec3) => 350 - (p.y - (maxY + minY) / 2) * 600 / span;
  const colors = ['#65d7ef', '#f9c77b'];
  const impact = engine.result?.collision;
  const wind = engine.wind;
  const windAngle = Math.atan2(-wind.y, wind.x) * 180 / Math.PI;
  return <section className="panel radar">
    <div className="panel-heading"><h2>Encounter radar</h2><span>Top view · north ↑</span></div>
    <svg viewBox="0 0 700 700" role="img" aria-label="Aircraft radar with projected paths, closest approach and wind">
      <defs><pattern id="grid" width="70" height="70" patternUnits="userSpaceOnUse"><path d="M 70 0 L 0 0 0 70" fill="none" stroke="#26363e" strokeWidth="1"/></pattern></defs>
      <rect width="700" height="700" fill="url(#grid)"/>
      {[100, 200, 300].map(r => <circle key={r} cx="350" cy="350" r={r} fill="none" stroke="#2c424a"/>)}
      {paths.map((p, i) => <g key={i} stroke={colors[i]} fill="none">
        <line x1={x(p.start)} y1={y(p.start)} x2={x(p.end)} y2={y(p.end)} strokeDasharray="7 7" opacity="0.5"/>
        <circle cx={x(p.cpa)} cy={y(p.cpa)} r="8" strokeWidth="2"/>
      </g>)}
      {paths.length === 2 && <g stroke="#b4c3c9"><line x1={x(paths[0].cpa)} y1={y(paths[0].cpa)} x2={x(paths[1].cpa)} y2={y(paths[1].cpa)} strokeDasharray="3 4"/><text x={x(paths[0].cpa) + 12} y={y(paths[0].cpa) - 12} fill="#b4c3c9" stroke="none">CPA</text></g>}
      {engine.waypoint && <g transform={`translate(${x(engine.waypoint)},${y(engine.waypoint)})`}><path d="M -7 0 L 0 -7 L 7 0 L 0 7 Z" fill="none" stroke="#aaa"/><text x="12" y="20" fill="#aaa">Waypoint</text></g>}
      {aircraft.map((a, i) => <g key={a.id} transform={`translate(${x(a.position)},${y(a.position)})`}>
        <path d="M 11 0 L -8 -7 L -4 0 L -8 7 Z" fill={colors[i]} transform={`rotate(${Math.atan2(-a.velocity.y, a.velocity.x) * 180 / Math.PI})`}/>
        <text x="14" y={i ? 30 : -26} fill={colors[i]}>
          <tspan x="14">AC {a.id} · {a.position.z.toFixed(0)} m</tspan>
          <tspan x="14" dy="16">{a.speedMetersPerSecond.toFixed(1)} m/s</tspan>
        </text>
      </g>)}
      {impact && <g transform={`translate(${x(impact.position)},${y(impact.position)})`} stroke="#ff7f83" strokeWidth="3"><circle r="22" fill="#ff7f8322"/><path d="M -10 -10 L 10 10 M 10 -10 L -10 10"/></g>}
      <g transform="translate(48,640)"><path d="M -15 0 L 15 0 L 8 -6 M 15 0 L 8 6" fill="none" stroke="#9eb6a8" strokeWidth="2" transform={`rotate(${windAngle})`}/><text x="30" y="5" fill="#9eb6a8">Wind {wind.x.toFixed(1)}, {wind.y.toFixed(1)} m/s</text></g>
      <line x1="480" y1="640" x2="620" y2="640" stroke="#9cb1bd"/><text x="480" y="665" fill="#9cb1bd">{(span * 140 / 600 / 1000).toFixed(1)} km</text>
    </svg>
    <p className="hint">● Aircraft · dashed lines: {engine.lookaheadSeconds}s projection · rings: CPA</p>
  </section>;
}
