using System.Runtime.InteropServices;
using System.Text.Json;
using Microsoft.Win32.SafeHandles;
using VectorWatch.Web.DTOs;

namespace VectorWatch.Web.Engine;

internal sealed class RunHandle : SafeHandleZeroOrMinusOneIsInvalid
{
    internal RunHandle(IntPtr value) : base(true) => SetHandle(value);
    protected override bool ReleaseHandle() { NativeRun.Destroy(handle); return true; }
}

internal sealed class NativeRun : IDisposable
{
    private const string Library = "vectorwatch_native";
    private readonly RunHandle handle;

    static NativeRun()
    {
        NativeLibrary.SetDllImportResolver(typeof(NativeRun).Assembly, (name, assembly, path) =>
        {
            if (name != Library) return IntPtr.Zero;
            var configured = Environment.GetEnvironmentVariable("VECTORWATCH_NATIVE_PATH");
            if (!string.IsNullOrEmpty(configured)) return NativeLibrary.Load(Path.GetFullPath(configured));
            var file = OperatingSystem.IsWindows() ? "vectorwatch_native.dll" :
                OperatingSystem.IsMacOS() ? "libvectorwatch_native.dylib" : "libvectorwatch_native.so";
            var local = Path.Combine(AppContext.BaseDirectory, file);
            return File.Exists(local) ? NativeLibrary.Load(local) : NativeLibrary.Load(name, assembly, path);
        });
    }

    [StructLayout(LayoutKind.Sequential)]
    private struct Options
    {
        [MarshalAs(UnmanagedType.LPUTF8Str)] public string Scenario;
        public double Speed;
        public uint Seed, UncertaintySeed;
        public int HasSeed, Probabilistic, Samples, Workers, Uncertainty, Outcome;
    }
    [DllImport(Library, EntryPoint = "vw_create", CallingConvention = CallingConvention.Cdecl)]
    private static extern IntPtr Create(in Options options, out IntPtr error);
    [DllImport(Library, EntryPoint = "vw_read", CallingConvention = CallingConvention.Cdecl)]
    private static extern IntPtr Read(RunHandle run, out IntPtr error);
    [DllImport(Library, EntryPoint = "vw_scenarios", CallingConvention = CallingConvention.Cdecl)]
    private static extern IntPtr Scenarios(out IntPtr error);
    [DllImport(Library, EntryPoint = "vw_stop", CallingConvention = CallingConvention.Cdecl)]
    private static extern void Stop(RunHandle run);
    [DllImport(Library, EntryPoint = "vw_destroy", CallingConvention = CallingConvention.Cdecl)]
    internal static extern void Destroy(IntPtr run);
    [DllImport(Library, EntryPoint = "vw_free_string", CallingConvention = CallingConvention.Cdecl)]
    private static extern void Free(IntPtr text);

    public NativeRun(RunSettings settings)
    {
        settings.Validate();
        var options = new Options
        {
            Scenario = settings.Scenario, Speed = settings.Speed, Seed = settings.Seed ?? 0,
            HasSeed = settings.Seed.HasValue ? 1 : 0, UncertaintySeed = settings.UncertaintySeed,
            Probabilistic = settings.Prediction == "probabilistic" ? 1 : 0,
            Samples = settings.Samples, Workers = settings.Workers,
            Uncertainty = settings.Uncertainty switch { "low" => 0, "high" => 2, _ => 1 },
            Outcome = settings.Outcome switch { "collision" => 1, "pass" => 2, _ => 0 }
        };
        var pointer = Create(in options, out var error);
        if (pointer == IntPtr.Zero) throw new InvalidOperationException(TakeError(error));
        handle = new RunHandle(pointer);
    }

    private static string TakeError(IntPtr error)
    {
        try { return Marshal.PtrToStringUTF8(error) ?? "Native engine failure"; }
        finally { if (error != IntPtr.Zero) Free(error); }
    }
    private static JsonElement TakeJson(IntPtr value, IntPtr error)
    {
        if (value == IntPtr.Zero) throw new InvalidOperationException(TakeError(error));
        try
        {
            using var json = JsonDocument.Parse(Marshal.PtrToStringUTF8(value)!);
            return json.RootElement.Clone();
        }
        finally { Free(value); }
    }
    public static JsonElement Catalog()
    {
        var value = Scenarios(out var error);
        return TakeJson(value, error);
    }
    public JsonElement View()
    {
        var value = Read(handle, out var error);
        return TakeJson(value, error);
    }
    public void RequestStop() => Stop(handle);
    public void Dispose() => handle.Dispose();
}
