// Research-only generated copy. Only model injection differs from production Run.
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;

namespace PIDSupporter
{
    internal static class SpResearchAutoTune
    {

        internal static PidSearch.Motion Evaluate(TuneRecord r, NativeLinear net, double[] candidate)
        {
            var budget = new WorkBudget(CancellationToken.None);
            double dt = Validate(r); int n = r.U.Length;
            double ys = Std(r.Y, n/2, Mean(r.Y,0,n/2));
            var warm = net.RecordState(r, budget);

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
            return Measure(candidate);
        }
        internal static double Validate(TuneRecord r)
        {
            int n = r.U.Length;
            if (n < 300 || n > 6000 || r.Y.Length != n || r.T.Length != n) throw new ArgumentException("Invalid sample count / 표본 수 오류");
            double[] intervals = new double[n - 1];
            for (int i = 1; i < n; i++) intervals[i - 1] = r.T[i] - r.T[i - 1];
            Array.Sort(intervals); double dt = intervals[intervals.Length / 2];
            if (!NativeLinear.Finite(dt) || dt <= 0 || Math.Abs(n * dt - 30) > .1) throw new ArgumentException("A continuous 30s record is required / 연속 30초 기록 필요");
            for (int i = 0; i < n; i++)
            {
                if (!NativeLinear.Finite(r.U[i]) || !NativeLinear.Finite(r.Y[i]) || !NativeLinear.Finite(r.T[i])) throw new ArgumentException("Non-finite measurements");
                if (i > 0 && Math.Abs(r.T[i] - r.T[i - 1] - dt) > dt * .1) throw new ArgumentException("Irregular controller sampling / 수집 간격 불규칙");
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

        internal static TuneResult Run(TuneRecord r, CancellationToken token, Action<string> progress, bool jointNeighborhood = false)
        {
            token.ThrowIfCancellationRequested();
            var wall = Stopwatch.StartNew(); var budget = new WorkBudget(token);
            double dt = Validate(r); int n = r.U.Length, fit = n / 2;
            double um = Mean(r.U, 0, fit), ym = Mean(r.Y, 0, fit);
            double us = Std(r.U, fit, um), ys = Std(r.Y, fit, ym);
            AutoTuneDiagnostics.Event("record", new { r.U, r.Y, r.R, r.Integrals, r.T, r.Pid, r.Lower, r.Upper, r.Amplitude, r.InitialIntegral, r.InitialError, fit, dt, um, ym, us, ys });
            if (us < 1e-4 || ys < 1e-5) throw new ArgumentException("Insufficient excitation / 가진 정보 부족, 기존 PID 유지");
            var selected = SpPidComparison.Selection ?? NativeLinear.Select(r, dt, budget, progress);
            var net = selected.Model;
            double best = selected.Score;
            // Reproduction errors rank models and remain diagnostics, not an acceptance threshold.
            if (!NativeLinear.Finite(best) || !NativeLinear.Finite(selected.FinalWorst))
                return new TuneResult { message = "선형 예측 수치 오류·발산 · 기존 PID 유지", seconds = wall.Elapsed.TotalSeconds, validation = best };
            var warm = net.RecordState(r, budget);
            progress("PID 후보 평가 중");
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
            var search = PidSearch.Run(r.Pid, p => Measure(p).Tracking, budget, progress, jointNeighborhood, Measure);
            double bestCost = search.RobustCost, baseline = search.BaselineRobust;
            AutoTuneDiagnostics.Event("pidSearch", search);
            bool accepted = NativeLinear.Finite(bestCost) && bestCost < .95 * baseline && baseline < 1e10;
            return new TuneResult { accepted = accepted, pid = accepted ? search.Pid : (double[])r.Pid.Clone(), validation = best,
                predicted_ratio = bestCost / Math.Max(baseline, 1e-12), seconds = wall.Elapsed.TotalSeconds,
                message = accepted ? "추천 PID 준비 완료. 적용 버튼으로 사용하세요." : "예측 개선이 부족해 기존 PID를 유지합니다." };
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
