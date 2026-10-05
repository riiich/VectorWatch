import type { Scenario } from '../types/simulation';
export async function fetchScenarios(signal: AbortSignal): Promise<Scenario[]> {
  const response = await fetch('/api/scenarios', { signal });
  if (!response.ok) throw new Error('Could not load scenarios. Check the server and retry.');
  return response.json();
}
