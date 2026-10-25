using System;
using System.Diagnostics;
using System.IO;
using System.Text.Json;
using System.Threading;
using PIDSupporter;

internal static class NativeChecks
{
    internal static void Run(string[] args)
    {
        var net = new NativeRen(40); net.Initialize(9701);
        net.Parameters[NativeRen.IBx] = .03; net.Parameters[NativeRen.IBv + 1] = -.02;
        net.Parameters[NativeRen.Y + 1] = .07; net.Parameters[NativeRen.IX0] = .1;
        var u = new double[40]; var y = new double[40];
        for (int i = 0; i < 40; i++) { u[i] = Math.Sin(i * .43); y[i] = Math.Cos(i * .1) * .2; }
        double loss = net.Loss(u, y, 40, true, CancellationToken.None);
        var gradient = (double[])net.Gradient.Clone(); double worst = 0;
        for (int j = 0; j < gradient.Length; j++)
        {
            double initial = net.Parameters[j], eps = 1e-5;
            net.Parameters[j] = initial + eps; double plus = net.Loss(u, y, 40, false, CancellationToken.None);
            net.Parameters[j] = initial - eps; double minus = net.Loss(u, y, 40, false, CancellationToken.None);
            net.Parameters[j] = initial;
            double numerical = (plus - minus) / (2 * eps);
            worst = Math.Max(worst, Math.Abs(numerical - gradient[j]) / Math.Max(1, Math.Abs(numerical)));
        }
        if (worst > 1e-6) throw new Exception("Finite difference gradient mismatch: " + worst);
        net.Refresh();
        Directory.CreateDirectory("tests/output");
        File.WriteAllText("tests/output/native-parity.json", JsonSerializer.Serialize(new { parameters = net.Parameters, explicitParameters = net.Explicit, gradient, u, y, loss }));
        Console.WriteLine($"PASS all 698 gradients vs finite differences; max scaled error {worst:E3}");
        var cancelled = new CancellationTokenSource(); cancelled.Cancel();
        bool stopped = false;
        try { net.Loss(u, y, 40, true, cancelled.Token); } catch (OperationCanceledException) { stopped = true; }
        if (!stopped) throw new Exception("Cancellation ignored");
        double integral = 999;
        NativeAutoTune.PidStep(new[] { .1, 250, .1 }, 1, 0, ref integral, .05, -1, 1, false);
        if (integral != 0) throw new Exception("FTD Ti=250 must disable I");
        NativeAutoTune.PidStep(new[] { .1, 1.0, .1 }, 100, 0, ref integral, .05, -1, 1, false);
        if (integral > 10) throw new Exception("FTD integral limit violated");
        var record = Fixture(600);
        NativeAutoTune.Validate(record);
        var saturatedRecord = Fixture(600);
        for (int i = 0; i < 600; i++) saturatedRecord.U[i] = i % 2 == 0 ? -1 : 1;
        NativeAutoTune.Validate(saturatedRecord); // Valid actual input, even at 100% saturation.
        integral = 0;
        if (NativeAutoTune.PidStep(new[] { .1, 1.0, 0.0 }, 100, 0, ref integral, .05, -1, 1, false) != 1)
            throw new Exception("Positive output saturation missing");
        for (int i = 0; i < 100; i++) NativeAutoTune.PidStep(new[] { .1, 1.0, 0.0 }, 100, 0, ref integral, .05, -1, 1, false);
        if (Math.Abs(integral - 10) > 1e-5) throw new Exception("Sustained saturation integral clamp");
        double recovery = NativeAutoTune.PidStep(new[] { .1, 1.0, 0.0 }, -1, 100, ref integral, .05, -1, 1, false);
        if (recovery <= 0 || recovery >= 1) throw new Exception("FTD integral retention on reversal");
        NativeAutoTune.PidStep(new[] { .1, 1.0, 0.0 }, 0, 0, ref integral, .6, -1, 1, false);
        if (integral != 0) throw new Exception("Long timestep must clear integral");
        record.T[30] += .02;
        bool rejected = false;
        try { NativeAutoTune.Validate(record); } catch (ArgumentException) { rejected = true; }
        if (!rejected) throw new Exception("Irregular record accepted");
        record.T[30] -= .02;
        Console.WriteLine("PASS cancellation, timing gate, FTD integral limiting/off");
        using (var stop = new CancellationTokenSource())
        {
            Exception? failure = null; bool observedCancel = false;
            var thread = new Thread(() =>
            {
                try { NativeAutoTune.Run(record, stop.Token, _ => { }); }
                catch (OperationCanceledException) { observedCancel = true; }
                catch (Exception ex) { failure = ex; }
            }) { IsBackground = true };
            thread.Start(); Thread.Sleep(30); var latency = Stopwatch.StartNew(); stop.Cancel();
            if (!thread.Join(2000) || !observedCancel || failure != null) throw new Exception("Active worker cancellation failed", failure);
            Console.WriteLine($"PASS active worker cancellation in {latency.Elapsed.TotalMilliseconds:0.00}ms");
        }
        if (Array.IndexOf(args, "--train") >= 0)
        {
            if (Array.IndexOf(args, "--1500") >= 0) record = Fixture(1500);
            var timer = Stopwatch.StartNew(); long before = GC.GetTotalAllocatedBytes(true);
            var result = NativeAutoTune.Run(record, CancellationToken.None, Console.WriteLine);
            var stats = new { result, wallSeconds = timer.Elapsed.TotalSeconds, allocatedBytes = GC.GetTotalAllocatedBytes(true) - before, samples = record.U.Length };
            File.WriteAllText("tests/output/native-training-" + record.U.Length + ".json", JsonSerializer.Serialize(stats));
            if (!NativeRen.Finite(result.validation) || result.validation > .5) throw new Exception("Synthetic holdout prediction failed");
            Console.WriteLine(JsonSerializer.Serialize(stats));
        }
    }

    internal static TuneRecord Fixture(int count)
    {
        var r = new TuneRecord { R = new double[count], Integrals = new double[count], U = new double[count], Y = new double[count], T = new double[count],
            Pid = new[] { .05, 2, .1 }, Lower = -1, Upper = 1, Amplitude = 1, Speed = .5 };
        double x = 0, integral = 0, previous = 0, dt = 30.0 / count;
        var rng = new Random(826); var phases = new double[60];
        for (int i = 0; i < 60; i++) phases[i] = rng.NextDouble() * 2 * Math.PI;
        for (int k = 0; k < count; k++)
        {
            double sp = 0;
            for (int j = 1; j <= 60; j++) sp += Math.Sin(2 * Math.PI * j * k * dt / 30 + phases[j - 1]) / 20;
            double error = sp - x;
            double action = NativeAutoTune.PidStep(r.Pid, error, previous, ref integral, dt, -1, 1, k == 0);
            r.R[k] = sp; r.Integrals[k] = integral; r.U[k] = action; r.Y[k] = x; r.T[k] = (k + 1) * dt;
            // Test-only synthetic plant, absent from the production learner.
            double a = Math.Exp(-dt); x = a * x + (1 - a) * 10 * action; previous = error;
        }
        return r;
    }
}
