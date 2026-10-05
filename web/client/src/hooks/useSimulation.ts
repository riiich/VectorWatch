import { useEffect, useRef, useState } from 'react';
import { HubConnection, HubConnectionBuilder, HttpTransportType, LogLevel } from '@microsoft/signalr';
import type { ConnectionStatus, RunSettings, RunUpdate } from '../types/simulation';

export function useSimulation() {
  // Intentionally not persisted: reloads and duplicated tabs start independent sessions.
  const [tabId] = useState(() => crypto.randomUUID());
  const hub = useRef<HubConnection | null>(null);
  const currentId = useRef<string | null>(null);
  const latestTime = useRef('');
  const received = useRef(new Map<string, RunUpdate>());
  const [connection, setConnection] = useState<ConnectionStatus>('connecting');
  const [update, setUpdate] = useState<RunUpdate | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);
  const operation = useRef(false);

  useEffect(() => {
    let disposed = false;
    let retry: ReturnType<typeof setTimeout> | undefined;
    const client = new HubConnectionBuilder()
      .withUrl('/hubs/simulation', { transport: HttpTransportType.WebSockets, skipNegotiation: true })
      .withAutomaticReconnect([0, 1000, 2000, 5000, 10000])
      .configureLogging(LogLevel.Warning).build();
    hub.current = client;
    function accept(value: RunUpdate) {
      if (disposed) return;
      received.current.set(value.runId, value);
      if (received.current.size > 16) received.current.delete(received.current.keys().next().value!);
      if (value.runId !== currentId.current || value.sentAt < latestTime.current) return;
      latestTime.current = value.sentAt;
      setUpdate(value);
    }
    async function join() {
      let value = await client.invoke<RunUpdate | null>('Join', tabId);
      if (disposed) return;
      const streamed = value && received.current.get(value.runId);
      if (streamed && streamed.sentAt > value!.sentAt) value = streamed;
      currentId.current = value?.runId ?? null;
      latestTime.current = value?.sentAt ?? '';
      setUpdate(value);
      setConnection('connected');
      setError(null);
    }
    async function connect() {
      if (disposed) return;
      setConnection('connecting');
      try { await client.start(); await join(); }
      catch (e) {
        if (disposed) return;
        setConnection('disconnected');
        setError(message(e));
        // Also retry initial failures; SignalR automatic reconnect only covers established connections.
        await client.stop();
        clearTimeout(retry);
        if (!disposed) retry = setTimeout(connect, 3000);
      }
    }
    client.on('RunUpdated', accept);
    client.onreconnecting(() => { if (!disposed) setConnection('reconnecting'); });
    client.onreconnected(async () => {
      try { await join(); }
      catch (e) { if (!disposed) { setError(message(e)); await client.stop(); } }
    });
    client.onclose(() => {
      if (disposed) return;
      setConnection('disconnected');
      clearTimeout(retry);
      retry = setTimeout(connect, 3000);
    });
    void connect();
    return () => { disposed = true; clearTimeout(retry); hub.current = null; void client.stop(); };
  }, [tabId]);

  async function command(method: 'Start' | 'Replay' | 'Stop', settings?: RunSettings) {
    if (!hub.current || connection !== 'connected' || operation.current) return;
    operation.current = true;
    setBusy(true); setError(null);
    try {
      if (method === 'Stop') await hub.current.invoke('Stop', tabId, currentId.current);
      else {
        let value = method === 'Start'
          ? await hub.current.invoke<RunUpdate>('Start', tabId, settings)
          : await hub.current.invoke<RunUpdate>('Replay', tabId);
        const streamed = received.current.get(value.runId);
        if (streamed && streamed.sentAt > value.sentAt) value = streamed;
        currentId.current = value.runId;
        latestTime.current = value.sentAt;
        setUpdate(value);
      }
    } catch (e) { setError(message(e)); }
    finally { operation.current = false; setBusy(false); }
  }
  return { connection, update, error, busy, command };
}
function message(error: unknown) { return error instanceof Error ? error.message : String(error); }
