using Microsoft.AspNetCore.Mvc;
using VectorWatch.Web.Engine;
namespace VectorWatch.Web.Controllers;

[ApiController]
[Route("api/scenarios")]
public sealed class ScenariosController : ControllerBase
{
    [HttpGet]
    public IActionResult Get() => Ok(NativeRun.Catalog());
}
