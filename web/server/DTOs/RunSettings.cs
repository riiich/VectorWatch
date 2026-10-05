namespace VectorWatch.Web.DTOs;

public sealed record RunSettings(
    string Scenario = "head-on",
    double Speed = 5,
    uint? Seed = null,
    string Prediction = "deterministic",
    int Samples = 10_000,
    int Workers = 0,
    uint UncertaintySeed = 1,
    string Uncertainty = "medium",
    string Outcome = "any")
{
    public void Validate()
    {
        if (string.IsNullOrWhiteSpace(Scenario) || Scenario.Length > 80 ||
            !double.IsFinite(Speed) || Speed < 0.1 || Speed > 1000 ||
            Samples < 1 || Samples > 100_000 || Workers < 0 || Workers > 16 ||
            Prediction is not ("deterministic" or "probabilistic") ||
            Uncertainty is not ("low" or "medium" or "high") ||
            Outcome is not ("any" or "collision" or "pass"))
            throw new ArgumentException("Invalid settings. Speed: 0.1–1000; samples: 1–100000; workers: 0–16.");
    }
}
