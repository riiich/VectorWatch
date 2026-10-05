export interface Vec3 { x: number; y: number; z: number }
export interface Aircraft { id: number; position: Vec3; velocity: Vec3; speedMetersPerSecond: number }
export interface Scenario { name: string; description: string }
export interface RunSettings {
  scenario: string; speed: number; seed: number | null;
  prediction: 'deterministic' | 'probabilistic'; samples: number; workers: number;
  uncertaintySeed: number; uncertainty: 'low' | 'medium' | 'high'; outcome: 'any' | 'collision' | 'pass';
}
export const defaultSettings: RunSettings = {
  scenario: 'head-on', speed: 5, seed: null, prediction: 'deterministic', samples: 10000,
  workers: 0, uncertaintySeed: 1, uncertainty: 'medium', outcome: 'any',
};
export interface Collision { timeSeconds: number; position: Vec3 }
export interface TimeWindow { exists: boolean; startSeconds: number; endSeconds: number }
export interface Prediction {
  snapshotSequence: number; simulationTimeSeconds: number;
  sampleCount: number; conflictCount: number; collisionCount: number;
  conflictProbability: number; collisionProbability: number; riskLevel: string;
  workerCount: number; calculationMilliseconds: number; nominalCollision: Collision | null;
  approach: {
    currentDistanceMeters: number; timeSeconds: number;
    horizontalSeparationMeters: number; verticalSeparationMeters: number;
    minimumHorizontalSeparationMeters: number; minimumHorizontalTimeSeconds: number;
    minimumVerticalSeparationMeters: number; minimumVerticalTimeSeconds: number;
    hasRelativeMotion: boolean; isWithinLookahead: boolean; conflict: boolean;
    horizontalViolationWindow: TimeWindow; verticalViolationWindow: TimeWindow; conflictWindow: TimeWindow;
  };
  paths: { start: Vec3; end: Vec3; cpa: Vec3 }[];
}
export interface EngineView {
  radarBounds: { minimumX: number; maximumX: number; minimumY: number; maximumY: number };
  seed: number | null; durationSeconds: number; lookaheadSeconds: number; wind: Vec3; waypoint: Vec3 | null;
  initialAircraft: Aircraft[];
  snapshot: { sequence: number; simulationTimeSeconds: number; aircraft: Aircraft[] } | null;
  prediction: Prediction | null; finished: boolean;
  result: { reason: 'collision' | 'completed' | 'stopped'; simulationTimeSeconds: number; collision: Collision | null } | null;
}
export interface RunUpdate { runId: string; settings: RunSettings; engine: EngineView; sentAt: string; error: string | null }
export type ConnectionStatus = 'connecting' | 'connected' | 'reconnecting' | 'disconnected';
