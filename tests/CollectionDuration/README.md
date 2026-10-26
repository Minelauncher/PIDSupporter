# Collection duration and accumulation (2026-10-27)

No production or installed mod files are changed by this study.

The 72 cases match ProductionNoiseless: six plant families, six physical
conditions and two seeds. There is no measurement noise. Physical disturbances,
coupling and nonlinearities remain. Plant coefficients/state are confined to the
synthetic environment and truth scorer; NativeAutoTune receives only records.

Five arms:

| Arm | Total recorded time | Fit / validation / final diagnostic time |
|---|---:|---|
| single30 | 30 s | 15 / 5 / 10 s |
| repeat2 | 60 s | 2 × (15 / 5 / 10 s) |
| continuous60 | 60 s | 30 / 10 / 20 s |
| repeat4 | 120 s | 4 × (15 / 5 / 10 s) |
| continuous120 | 120 s | 60 / 20 / 40 s |

Preparation is performed once with the production gain policy in each branch.
Repeated records keep the prepared PID and excitation amplitude; recommendations
are not applied between collections. Physical and controller states carry across
records. Each repeated record receives its own independent fixed phase/sign,
1-second endpoint fades and per-record initial histories/offsets. This isolates
additional collection from iterative controller changes; it is not a convergence
test or an exact replay of re-running gain preparation on every UI click.

Continuous recordings keep the same 0.2 Hz square frequency, phase, amplitude,
input limits and 1-second endpoint fades, stretching only the recording length.
Thus effective excitation duty cycle and realized input energy need not be equal
to repeated recordings of the same total length. No energy matching or extra
calibration trial is performed.

`AutoTuneSession.generated.cs` is a research copy of production with only:
record duration/fade endpoint/centering-grid length parameterized, capture of
the TuneRecord, and removal of asynchronous fitting (the harness fits synchronously).
`NativeAutoTune.generated.cs` extends the duration whitelist to 60/120 seconds.
All model fitting, 1–3 order selection, no-ridge numerical treatment, empirical
frequency gate, recent-state initialization and PID search remain production code.
This preserves the effect of additional 5-second frequency windows on the
maximum-discrepancy gate, rather than assuming uncertainty must shrink with N.

The first 30-second generated record was checked against the previous production
harness: U, Y, T, PID, injected SP and instrument arrays match exactly in p000.

All five recommendations are truth-scored from the same first-30-second endpoint
for each case, with the same plant/controller state and disturbance phase. The
scorer uses the same nine smooth-SP tasks and 27-point neighborhoods as the prior
study. The shared finite reference pool includes all five recommendations, the
working/original PID and 121 base centers. This is a finite-pool selection loss,
not global optimality. Nominal PID performance is also reported. Fitting still
uses each arm's actual latest observed state, as production does.

Run from the repository root (prepared cases copied from the previous study):

```powershell
dotnet build tests/CollectionDuration/Study.csproj -c Release
dotnet tests/CollectionDuration/bin/Release/net10.0/Study.dll tests/output/collection-duration-20261027-verified 0 1
python tests/CollectionDuration/grade.py
python tests/CollectionDuration/report.py
```

The final two arguments are shard index and count. The original run used three
processes on disjoint cases; no agent delegation was used. Selection has the
unchanged production wall-clock budget, and budget hits are reported. Existing
output files are reused; use a fresh directory for a fresh run. Python requires
NumPy. Outputs include complete records, model/search diagnostics and frozen
source/result hashes. Old `collection-duration-20261027` output was an abandoned
serialization smoke test and is not used in scoring.
