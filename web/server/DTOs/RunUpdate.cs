using System.Text.Json;
namespace VectorWatch.Web.DTOs;

// Native JSON contains the engine's snapshot, prediction and final result unchanged.
public sealed record RunUpdate(Guid RunId, RunSettings Settings, JsonElement Engine,
    DateTimeOffset SentAt, string? Error = null);
