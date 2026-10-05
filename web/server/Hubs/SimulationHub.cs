using Microsoft.AspNetCore.SignalR;
using VectorWatch.Web.DTOs;
using VectorWatch.Web.Services;
namespace VectorWatch.Web.Hubs;

public sealed class SimulationHub(RunService runs) : Hub
{
    public RunUpdate? Join(Guid tabId) => runs.Join(tabId, Context.ConnectionId);
    public RunUpdate Start(Guid tabId, RunSettings settings) => runs.Start(tabId, Context.ConnectionId, settings, false);
    public RunUpdate Replay(Guid tabId) => runs.Start(tabId, Context.ConnectionId, null, true);
    public void Stop(Guid tabId, Guid runId) => runs.Stop(tabId, Context.ConnectionId, runId);
    public override Task OnDisconnectedAsync(Exception? exception)
    {
        runs.Disconnect(Context.ConnectionId);
        return base.OnDisconnectedAsync(exception);
    }
}
