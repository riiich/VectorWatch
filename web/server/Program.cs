using Microsoft.AspNetCore.Http.Connections;
using VectorWatch.Web.Engine;
using VectorWatch.Web.Hubs;
using VectorWatch.Web.Services;

var builder = WebApplication.CreateBuilder(args);
builder.Services.AddControllers();
builder.Services.AddSignalR(options => options.MaximumReceiveMessageSize = 16 * 1024);
builder.Services.AddSingleton<RunService>();
builder.Services.AddHostedService(provider => provider.GetRequiredService<RunService>());
var app = builder.Build();
// Fail early with a useful error if the shared library cannot be loaded.
try { _ = NativeRun.Catalog(); }
catch (Exception e) { throw new InvalidOperationException("Build vectorwatch_native and set VECTORWATCH_NATIVE_PATH to its absolute filename.", e); }
app.UseDefaultFiles();
app.UseStaticFiles();
app.MapControllers();
app.MapHub<SimulationHub>("/hubs/simulation", options => options.Transports = HttpTransportType.WebSockets);
app.MapFallbackToFile("index.html");
app.Run();
