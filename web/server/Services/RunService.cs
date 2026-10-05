using Microsoft.AspNetCore.SignalR;
using VectorWatch.Web.DTOs;
using VectorWatch.Web.Engine;
using VectorWatch.Web.Hubs;
using VectorWatch.Web.Models;
namespace VectorWatch.Web.Services;

public sealed class RunService(IHubContext<SimulationHub> hub, IConfiguration configuration,
    ILogger<RunService> logger) : BackgroundService
{
    private readonly object gate = new();
    private readonly Dictionary<Guid, RunSession> sessions = [];
    private readonly TimeSpan grace = TimeSpan.FromSeconds(configuration.GetValue("Runs:ReconnectGraceSeconds", 60));

    public RunUpdate? Join(Guid tab, string connection)
    {
        if (tab == Guid.Empty) throw new HubException("Missing tab identity.");
        lock (gate)
        {
            // A connection belongs to exactly one tab, even if Join is called repeatedly.
            foreach (var pair in sessions.Where(p => p.Key != tab && p.Value.ConnectionId == connection))
            {
                pair.Value.ConnectionId = null;
                pair.Value.DisconnectedAt = DateTimeOffset.UtcNow;
            }
            if (!sessions.TryGetValue(tab, out var session))
            {
                if (sessions.Count >= 128) throw new HubException("Too many sessions. Try again shortly.");
                sessions[tab] = session = new RunSession();
            }
            session.ConnectionId = connection;
            session.DisconnectedAt = null;
            return session.Latest;
        }
    }

    private RunSession Owned(Guid tab, string connection)
    {
        if (!sessions.TryGetValue(tab, out var session) || session.ConnectionId != connection)
            throw new HubException("Session expired. Reconnect before starting a run.");
        return session;
    }

    public RunUpdate Start(Guid tab, string connection, RunSettings? settings, bool replay)
    {
        lock (gate)
        {
            var session = Owned(tab, connection);
            if (session.Native != null) throw new HubException("Stop the current run first.");
            settings = replay ? session.Latest?.Settings : settings;
            if (settings == null) throw new HubException("No run is available to replay.");
            if (sessions.Values.Count(s => s.Native != null) >= 16) throw new HubException("All run slots are busy.");
            NativeRun? native = null;
            try
            {
                settings.Validate();
                native = new NativeRun(settings);
                var view = native.View();
                var seed = view.GetProperty("seed");
                if (seed.ValueKind == System.Text.Json.JsonValueKind.Number)
                    settings = settings with { Seed = seed.GetUInt32() };
                session.Latest = new RunUpdate(Guid.NewGuid(), settings, view, DateTimeOffset.UtcNow);
                session.Native = native;
                return session.Latest;
            }
            catch (Exception e)
            {
                native?.Dispose();
                logger.LogWarning(e, "Could not start native run");
                throw new HubException(e is ArgumentException or InvalidOperationException ? e.Message : "Native engine unavailable. Check the server log.");
            }
        }
    }

    public void Stop(Guid tab, string connection, Guid runId)
    {
        lock (gate)
        {
            var session = Owned(tab, connection);
            if (session.Latest?.RunId == runId) session.Native?.RequestStop();
        }
    }
    public void Disconnect(string connection)
    {
        lock (gate)
        {
            foreach (var s in sessions.Values.Where(s => s.ConnectionId == connection))
            {
                s.ConnectionId = null;
                s.DisconnectedAt = DateTimeOffset.UtcNow;
            }
        }
    }
    protected override async Task ExecuteAsync(CancellationToken stoppingToken)
    {
        using var timer = new PeriodicTimer(TimeSpan.FromMilliseconds(50));
        try
        {
            while (await timer.WaitForNextTickAsync(stoppingToken))
            {
                var updates = new List<(string Connection, RunUpdate Update)>();
                lock (gate)
                {
                    foreach (var (id, session) in sessions.ToArray())
                    {
                        if (session.DisconnectedAt is { } at && DateTimeOffset.UtcNow - at > grace)
                        {
                            session.Native?.Dispose();
                            sessions.Remove(id);
                            logger.LogDebug("Released abandoned session {Tab}", id);
                            continue;
                        }
                        if (session.Native == null) continue;
                        try
                        {
                            var view = session.Native.View();
                            session.Latest = session.Latest! with { Engine = view, SentAt = DateTimeOffset.UtcNow };
                            if (view.GetProperty("finished").GetBoolean())
                            {
                                session.Native.Dispose();
                                session.Native = null;
                            }
                        }
                        catch (Exception e)
                        {
                            logger.LogError(e, "Native run failed");
                            session.Latest = session.Latest! with { Error = "Native run failed. Start or replay to try again.", SentAt = DateTimeOffset.UtcNow };
                            session.Native?.Dispose();
                            session.Native = null;
                        }
                        if (session.ConnectionId is { } connection) updates.Add((connection, session.Latest!));
                    }
                }
                foreach (var (connection, update) in updates)
                    await hub.Clients.Client(connection).SendAsync("RunUpdated", update, stoppingToken);
            }
        }
        catch (OperationCanceledException) when (stoppingToken.IsCancellationRequested) { }
        finally
        {
            lock (gate)
            {
                foreach (var session in sessions.Values) session.Native?.Dispose();
                sessions.Clear();
            }
        }
    }
}
