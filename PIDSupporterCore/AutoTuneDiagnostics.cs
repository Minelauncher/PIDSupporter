using System;
using System.IO;
using System.Collections.Concurrent;
using System.Threading;
using BrilliantSkies.Core.Logger;
using Newtonsoft.Json;
using UnityEngine;

namespace PIDSupporter
{
    // Frame timing telemetry only: no optimization or gameplay decisions depend on it.
    internal static class AutoTuneDiagnostics
    {
        private static readonly double[] idle = new double[600], phaseFrames = new double[60000];
        private static int idleCount, idleCursor, phaseCount;
        private static string phase = "idle";
        private static AutoTuneSession? owner;
        private static readonly ConcurrentQueue<string> pending = new ConcurrentQueue<string>();
        private static int writing;
        private static readonly string file = Path.Combine(Path.GetDirectoryName(typeof(AutoTuneDiagnostics).Assembly.Location)!,
            "Diagnostics", "autotune-" + DateTime.UtcNow.ToString("yyyyMMdd-HHmmss") + ".jsonl");
        internal static void Event(string kind, object value)
        {
            pending.Enqueue(JsonConvert.SerializeObject(new { utc = DateTime.UtcNow, kind, value }));
            if (Interlocked.CompareExchange(ref writing, 1, 0) == 0) ThreadPool.QueueUserWorkItem(_ => Flush());
        }
        private static void Flush()
        {
            try
            {
                Directory.CreateDirectory(Path.GetDirectoryName(file)!);
                using (var stream = new StreamWriter(file, true))
                    while (pending.TryDequeue(out var line)) stream.WriteLine(line);
            }
            catch (Exception ex) { AdvLogger.LogError("PIDSupporter diagnostic write failed: " + ex.Message, LogOptions.None); }
            finally
            {
                Interlocked.Exchange(ref writing, 0);
                if (!pending.IsEmpty && Interlocked.CompareExchange(ref writing, 1, 0) == 0)
                    ThreadPool.QueueUserWorkItem(_ => Flush());
            }
        }
        internal static void Tick()
        {
            var current = AutoTuneSession.Active;
            string next = current == null ? "idle" : current.Collecting ? "collect" : "learn";
            if (current != owner || next != phase)
            {
                if (phase != "idle") Emit(phase, phaseFrames, phaseCount);
                if (owner == null && current != null) Emit("baseline", idle, idleCount);
                phaseCount = 0; phase = next; owner = current;
            }
            double ms = Time.unscaledDeltaTime * 1000.0;
            if (ms <= 0 || double.IsNaN(ms)) return;
            if (phase == "idle") { idle[idleCursor] = ms; idleCursor = (idleCursor + 1) % idle.Length; idleCount = Math.Min(idleCount + 1, idle.Length); }
            else if (phaseCount < phaseFrames.Length) phaseFrames[phaseCount++] = ms;
        }
        private static void Emit(string label, double[] data, int count)
        {
            if (count == 0) return;
            var sorted = new double[count]; Array.Copy(data, sorted, count); Array.Sort(sorted);
            double sum = 0; int over33 = 0, over50 = 0, over100 = 0;
            foreach (double ms in sorted) { sum += ms; if (ms > 33.3) over33++; if (ms > 50) over50++; if (ms > 100) over100++; }
            Event("frames", new { phase = label, count, seconds = sum / 1000, fps = count * 1000 / sum,
                medianMs = sorted[count / 2], p95Ms = sorted[(int)((count - 1) * .95)], p99Ms = sorted[(int)((count - 1) * .99)],
                maxMs = sorted[count - 1], over33, over50, over100 });
        }
    }
}
