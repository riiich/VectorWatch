using VectorWatch.Web.DTOs;
using VectorWatch.Web.Engine;
namespace VectorWatch.Web.Models;

internal sealed class RunSession
{
    public string? ConnectionId { get; set; }
    public DateTimeOffset? DisconnectedAt { get; set; }
    public NativeRun? Native { get; set; }
    public RunUpdate? Latest { get; set; }
}
