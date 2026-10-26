# Recording timing regression (1.1.5)

Run `dotnet run --project tests/TimingChecks/TimingChecks.csproj -c Release`.
The project links production source and substitutes only the game host.

316 assertions cover:

- Normal 24/30-second records at 10/20/25/50/100 ms intervals where the existing
  minimum sample count permits them.
- Float absolute clock offsets of 0, 2048, 4096 and 8192 seconds, converted to
  relative collection times as in production.
- Reproduction of the old median-based false failure at 2048 seconds, 25 ms.
- One missing interior sample, missing blocks, duplicate/backwards/NaN timestamps,
  truncated starts and ends, zero span and unsupported duration.
- Historical zero-based sample labels.
- Twelve actual AutoTuneSession collection runs with float absolute timestamps.

Expected result: `PASS 316 timing assertions`.

The corrected dt comes from the measured span, not declared duration or assumed
sample count. Per-interval 10% consistency remains required. Start/end coverage
allows one sample of collector boundary rounding. This cannot distinguish an
otherwise regular lower-rate signal from systematic decimation, or one permitted
boundary rounding sample from a dropped endpoint. Severely quantized clocks are
still rejected when their intervals exceed the tolerance. No missing samples are
silently filled or interpolated.

SessionChecks also covers preparation, fixed collection, setting changes,
window cancellation and accumulated-record isolation. Its legacy NativeChecks
fixture now provides the exogenous SP arrays required by the production IV path;
otherwise its worker-cancellation test returned before doing identification.
