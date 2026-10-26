# Production noiseless / historical FRIT comparison

This harness links the production C# tuner directly. It supplies a synthetic
physical plant through the host measurement interface, without exposing plant
coefficients or simulator states to the tuner. The runtime algorithm is the one
shipped in 1.1.3 and unchanged in documentation release 1.1.4.

Six plant families, six conditions, two excitation seeds: 72 cases. Conditions
are baseline, nonzero initial state, 300 ms delivery delay, 20% actuator
effectiveness, four-times-slower dynamics, and a coupled physical disturbance.
There is no measurement noise. Each case starts a fresh window; accumulated
multi-record identification is not tested here.

Preparation uses actual production gain adjustment. Collection is the actual
30-second fixed square-wave policy. Identification, model selection, empirical
frequency gate, hierarchical objective and joint-neighborhood PID search are
the production implementations. The 180-second preparation timeout is only a
harness guard; production preparation has no such timeout. The excitation RNG
seed is fixed through reflection, solely for reproducibility.

Historical FRIT uses unchanged pure C# methods from commit `715d44b`, with
MathNet.Numerics, in `../LegacyFritNoiseless/Legacy.cs`. Its adapter recreates
the original mean-removed Hann/Welch spectra, automatic bandwidth target sweep,
controller seed, saturation exclusion and slider rounding. This compares
**tuners on identical current-policy records**, not the complete old excitation
and preparation workflow. Endpoint saturation flags are reconstructed from
actual clipped input. The historical reference-model and post-optimization D
caps are retained. A non-finite result retains the collection PID and is counted.

Evaluation starts all candidates at the same physical endpoint, actuator queue,
disturbance phase and controller state. Nine smooth-SP tasks combine amplitudes
0.1/0.3/0.6 and time constants 0.5/1.25/2 seconds, with both directions and a
15-second horizon. Primary reporting averages the nine worst-direction,
worst-neighborhood normalized tracking errors. Nominal recommendation errors
are also reported. A separate task matches the production search amplitude.
The reference best is a finite PID pool shared by both methods, including both
recommendations; it is **not a global optimum**. Changes within 1% are counted
as similar. Divergence is an explicit finite-horizon threshold, not a proof of
stability.

From the repository root, with .NET 10, NumPy and SciPy installed:

```powershell
python tests/prepare_production_noiseless.py
dotnet run --project tests/ProductionNoiseless/Audit.csproj -c Release -- tests/output/production-noiseless-20261026
python tests/grade_production_noiseless.py
python tests/report_production_noiseless.py
dotnet run --project tests/LegacyFritNoiseless/Audit.csproj -c Release -- tests/output/production-noiseless-20261026
python tests/compare_production_frit.py
```

The FRIT harness expects the original `MathNet.Numerics.dll` in the repository
root. Results and raw records are written below `tests/output/`; grading resumes
from existing case files. Use a clean output directory for a fresh experiment.
The production search has a wall-clock budget, so exact recommendations can
vary on slower hardware even with fixed excitation seeds. This is not an FTD
physics replay or an in-game frame-time benchmark.
