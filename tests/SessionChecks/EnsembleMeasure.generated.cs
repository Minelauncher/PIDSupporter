// Generated from production NativeAutoTune.Measure; do not edit.
using System;
using System.Linq;
using System.Collections.Generic;
namespace PIDSupporter {
internal static class EnsembleMeasure {
internal static Func<double[],PidSearch.Motion> Create(TuneRecord r, NativeLinear net, WorkBudget budget) {
double dt=NativeAutoTune.Validate(r); int n=r.U.Length, fit=n/2;
double ym=r.Y.Take(fit).Average(), ys=Math.Sqrt(r.Y.Take(fit).Average(v=>(v-ym)*(v-ym)));
            var warm = net.RecentState(r);

            var x = new double[net.Order * 2];
            double bias = r.U.Skip(Math.Max(0, n - (int)Math.Round(1 / dt))).Average();
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
                        double next = NativeAutoTune.PidStep(candidate, error, prev, ref integral, dt, r.Lower, r.Upper, false);
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

return Measure;
}
}}
