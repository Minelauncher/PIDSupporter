using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;

namespace PIDSupporter
{
    internal sealed class TuneRecord
    {
        internal double[] R = null!, Integrals = null!;
        internal double[] InjectedSP = null!, BaseSP = null!;
        internal double[]? InstrumentSP;
        internal double Duration = 30;
        internal int ExcitationSeed;
        internal double[] U = null!, Y = null!, T = null!, Pid = null!;
        internal double Speed, Amplitude, Lower, Upper, InitialIntegral, InitialError;
        internal AutoTuneSpectrum? Spectrum;
    }

    internal sealed class WorkBudget
    {
        private readonly Stopwatch slice = Stopwatch.StartNew(), total = Stopwatch.StartNew();
        private readonly CancellationToken token;
        internal WorkBudget(CancellationToken token) { this.token = token; }
        internal void Check()
        {
            token.ThrowIfCancellationRequested();
            if (total.Elapsed.TotalMinutes > 10) throw new TimeoutException("Identification time limit; current PID retained.");
            // Dedicated low-priority worker, at most ~4ms bursts between update boundaries.
            // Sleep only the worker, never Unity's main thread. OS scheduling is not a hard real-time bound.
            if (slice.Elapsed.TotalMilliseconds >= 4) { Thread.Sleep(2); slice.Restart(); }
        }
    }

    internal static class NativeAutoTune
    {
        internal static double Validate(TuneRecord r)
        {
            int n = r.U.Length;
            if (n < 300 || n > 6000 || r.Y.Length != n || r.T.Length != n) throw new ArgumentException("Invalid sample count.");
            // Float game timestamps quantize individual intervals asymmetrically.
            // Their median can be biased enough that n * median rejects a valid 30 s
            // record. Endpoint differencing averages that quantization instead.
            double dt = (r.T[n - 1] - r.T[0]) / (n - 1);
            if (!NativeLinear.Finite(dt) || dt <= 0)
                throw new ArgumentException("Invalid controller timestamps; collect a new record.");
            if (r.Duration != 24 && r.Duration != 30)
                throw new ArgumentException("Unsupported recording duration.");
            // Collection timestamps are relative to its start. Permit the collector's
            // one-step endpoint rounding, but not truncated starts/ends. Historical
            // records may label their first sample at zero rather than at dt.
            double edgeTolerance = dt * 1.1 + 1e-6;
            if (r.T[0] < 0 || r.T[0] > edgeTolerance
                || Math.Abs(r.T[n - 1] - r.Duration) > edgeTolerance)
                throw new ArgumentException("Incomplete recording: start or end samples are missing. Collect a full uninterrupted record.");
            for (int i = 0; i < n; i++)
            {
                if (!NativeLinear.Finite(r.U[i]) || !NativeLinear.Finite(r.Y[i]) || !NativeLinear.Finite(r.T[i])) throw new ArgumentException("Non-finite measurements");
                // A full duration alone is insufficient: missing/duplicate samples
                // must still fail even when the average interval looks plausible.
                if (i > 0 && Math.Abs(r.T[i] - r.T[i - 1] - dt) > dt * .1)
                    throw new ArgumentException($"Irregular controller sampling at sample {i}: missing samples or insufficient clock precision. Collect again; restart the game if it persists.");
                if (r.U[i] < r.Lower - 1e-5 || r.U[i] > r.Upper + 1e-5) throw new ArgumentException("Input outside recorded limits");
            }
            if (!NativeLinear.Finite(r.Lower) || !NativeLinear.Finite(r.Upper) || r.Lower >= r.Upper
                || !NativeLinear.Finite(r.Speed) || r.Speed < 0 || r.Speed > 1
                || !NativeLinear.Finite(r.Amplitude) || r.Amplitude <= 0 || r.Pid.Length != 3)
                throw new ArgumentException("Invalid controller settings");
            foreach (double value in r.Pid) if (!NativeLinear.Finite(value)) throw new ArgumentException("Invalid PID");
            if (r.Pid[0] <= 0 || r.Pid[1] < 0 || r.Pid[2] < 0) throw new ArgumentException("Invalid PID");
            return dt;
        }

        private static double Mean(double[] a, int start, int end)
        { double s = 0; for (int i = start; i < end; i++) s += a[i]; return s / (end - start); }
        private static double Std(double[] a, int n, double mean)
        { double s = 0; for (int i = 0; i < n; i++) s += (a[i] - mean) * (a[i] - mean); return Math.Sqrt(s / n); }
        internal static double Clamp(double x, double low, double high) => Math.Max(low, Math.Min(high, x));

        internal static TuneResult Run(TuneRecord r, CancellationToken token, Action<string> progress, bool jointNeighborhood = true, IReadOnlyList<TuneRecord>? records = null)
        {
            token.ThrowIfCancellationRequested();
            var wall = Stopwatch.StartNew(); var budget = new WorkBudget(token);
            double dt = Validate(r); int n = r.U.Length, fit = n / 2;
            double um = Mean(r.U, 0, fit), ym = Mean(r.Y, 0, fit);
            double us = Std(r.U, fit, um), ys = Std(r.Y, fit, ym);
            AutoTuneDiagnostics.Event("record", new { r.Duration, r.ExcitationSeed, r.InstrumentSP, r.U, r.Y, r.R, r.InjectedSP, r.BaseSP, r.Integrals, r.T, r.Pid, r.Lower, r.Upper, r.Amplitude, r.InitialIntegral, r.InitialError, fit, dt, um, ym, us, ys });
            TuneResult More(string detail) => new TuneResult { reason="more_data_required", needs_more_data=true, seconds=wall.Elapsed.TotalSeconds,
                message="More data needed: " + detail + " Run another 30 s test in this window to accumulate data." };
            if (us < 1e-4 || ys < 1e-5) return More("weak input/output response.");
            NativeLinear.Selection selected;
            try { selected = records != null && records.Count > 1 ? NativeLinear.SelectAccumulated(records,budget,progress) : NativeLinear.Select(r, dt, budget, progress); }
            catch (ArgumentException ex) { return More(ex.Message); }
            catch (InvalidOperationException ex) { return More(ex.Message); }
            var net = selected.Model;
            double best = selected.Score;
            // A sign-inconsistent fit cannot justify a positive-PID recommendation.
            if (!NativeLinear.Finite(net.DcGain()) || net.DcGain() <= 0) return More("model gain is inconsistent with the current positive PID.");
            // Reproduction errors rank models and remain diagnostics, not an acceptance threshold.
            if (!NativeLinear.Finite(best) || !NativeLinear.Finite(selected.FinalWorst))
                return More("model response could not be predicted.");
            EmpiricalFrequencyGate gate;
            try { gate=new EmpiricalFrequencyGate(net,records ?? new[]{r},budget); }
            catch(ArgumentException ex){return More(ex.Message);}
            AutoTuneDiagnostics.Event("empiricalFrequencyBounds",new{frequencies=gate.Frequencies,radius=gate.Radii,policy="observed-harmonic-disks",fullBandGuarantee=false});
            var warm = net.RecentState(r);
            progress("Evaluating PID candidates...");
            var x = new double[net.Order * 2];
            double bias = Mean(r.U, Math.Max(0, n - (int)Math.Round(1 / dt)), n);
            double delta = Math.Max(.5 * r.Amplitude, .1 * ys);
            var metrics = new Dictionary<(double,double,double),PidSearch.Motion>();
            PidSearch.Motion Measure(double[] candidate)
            {
                budget.Check(); var key=(candidate[0],candidate[1],candidate[2]);
                if (metrics.TryGetValue(key,out var cached)) return cached;
                double cost = 0, worstSlew = 0, worstEffort = 0;
                for (int sign = -1; sign <= 1; sign += 2)
                {
                    Array.Copy(warm, x, x.Length);
                    double lastU = r.U[n - 1], integral = r.InitialIntegral, prev = r.InitialError;
                    double total = 0, totalSlew = 0, totalEffort = 0; int steps = (int)Math.Round(15 / dt);
                    for (int k = 0; k < steps; k++)
                    {
                        if ((k & 31) == 0) budget.Check();
                        double yp = net.Next(x);
                        double target = r.Y[n - 1] + sign * delta * (1 - Math.Exp(-k * dt / (2 - 1.5 * r.Speed)));
                        double error = target - yp;
                        double next = PidStep(candidate, error, prev, ref integral, dt, r.Lower, r.Upper, false);
                        prev = error;
                        double effort = (next - bias) / (r.Upper - r.Lower);
                        double slew = (next - lastU) / ((r.Upper - r.Lower) * dt);
                        double c = Math.Pow(error / delta, 2);
                        totalSlew += slew * slew; totalEffort += effort * effort;
                        if (!NativeLinear.Finite(c) || Math.Abs(yp - r.Y[n - 1]) > 20 * delta) { total = 1e12 * steps; break; }
                        total += c; net.Push(x, yp, next); lastU = next;
                    }
                    cost = Math.Max(cost, total / steps);
                    worstSlew = Math.Max(worstSlew,totalSlew/steps); worstEffort = Math.Max(worstEffort,totalEffort/steps);
                }
                var value = new PidSearch.Motion { Tracking=cost, Slew=worstSlew, Effort=worstEffort };
                metrics.Add(key,value);return value;
            }
            var search = PidSearch.Run(r.Pid, p => Measure(p).Tracking, budget, progress, jointNeighborhood, Measure, gate.Accept);
            double bestCost = search.RobustCost, baseline = search.RawBaselineRobust;
            AutoTuneDiagnostics.Event("pidSearch", search);
            AutoTuneDiagnostics.Event("empiricalFrequencyChecks",new{checks=gate.Checks,rejected=gate.Rejected,search.BaselinePassesConstraint,search.CandidatePassesConstraint});
            if(!search.CandidatePassesConstraint)return new TuneResult{reason="no_robust_candidate",accepted=false,pid=(double[])r.Pid.Clone(),seconds=wall.Elapsed.TotalSeconds,message="No searched PID passed the model and frequency checks. Current PID retained; another test can add data."};
            string reason = DecisionReason(bestCost, baseline);
            bool accepted = reason == "recommended";
            if(reason=="baseline_prediction_failed"||reason=="candidate_prediction_failed")return More("PID response prediction failed.");
            return new TuneResult { reason = reason, accepted = accepted, pid = accepted ? search.Pid : (double[])r.Pid.Clone(), validation = best,
                predicted_ratio = bestCost / Math.Max(baseline, 1e-12), seconds = wall.Elapsed.TotalSeconds,
                message = DecisionMessage(reason) };
        }

        internal static string DecisionReason(double bestCost, double baseline)
        {
            if (!NativeLinear.Finite(baseline) || baseline >= 1e10) return "baseline_prediction_failed";
            if (!NativeLinear.Finite(bestCost) || bestCost >= 1e10) return "candidate_prediction_failed";
            return bestCost < .95 * baseline ? "recommended" : "insufficient_improvement";
        }
        private static string DecisionMessage(string reason)
        {
            if (reason == "recommended") return "Recommendation ready. Review it, then select Apply.";
            if (reason == "baseline_prediction_failed") return "Model prediction failed for the current PID (response outside evaluation limits); comparison unavailable. Current PID retained.";
            if (reason == "candidate_prediction_failed") return "Model prediction failed for PID candidates (response outside evaluation limits). Current PID retained.";
            return "Predicted improvement is below 5%. Current PID retained.";
        }

        internal static double PidStep(double[] pid, double error, double previous, ref double integral, double dt, double lower, double upper, bool first)
        {
            // FTD 4.3.4.1 PidStandardForm.NewMeasurement: preserve float arithmetic,
            // integral-contribution clamping (not conditional integration), and D on error.
            if (first || dt == 0) return 0;
            float kp = (float)pid[0], ti = Math.Max((float)pid[1], 1e-9f), td = (float)pid[2];
            float e = (float)error, h = (float)dt, lo = (float)lower, hi = (float)upper;
            float derivative = (e - (float)previous) / h;
            float acc = (float)integral + e * h;
            if (acc / ti * kp > hi) acc = hi * ti / kp;
            else if (acc / ti * kp < lo) acc = lo * ti / kp;
            if (ti >= 250 || h > .5f) acc = 0;
            integral = acc;
            return Clamp(kp * (e + acc / ti + derivative * td), lo, hi);
        }

        internal static List<double[]> Candidates(double[] pid)
        {
            var ps = new SortedSet<double>(); var ts = new SortedSet<double>(); var ds = new SortedSet<double> { 0, .1 };
            foreach (double f in new[] { .5, .7, 1, 1.4, 2 }) { ps.Add(Math.Round(Clamp(pid[0] * f, .001, 1), 3)); ts.Add(Math.Round(Clamp(pid[1] * f, .1, 250), 1)); }
            foreach (double f in new[] { .5, 1, 1.5, 2 }) ds.Add(Math.Round(Clamp(pid[2] * f, 0, 100), 1));
            ts.Add(250); var result = new List<double[]> { (double[])pid.Clone() };
            foreach (double p in ps) foreach (double t in ts) foreach (double d in ds) result.Add(new[] { p, t, d });
            return result;
        }
    }
}
