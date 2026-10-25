using System;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using System.Runtime.CompilerServices;
using BrilliantSkies.Ai.Control.Pids;

namespace PIDSupporter
{
    // Read-only diagnostics. Resolve ownership once; never scan vehicles per sample.
    // Reflection is restricted to known game members so missing/versioned fields become null.
    internal sealed class AutoTuneContext
    {
        private sealed class Binding { internal object Source = null!; internal string Kind = ""; }
        private static readonly ConditionalWeakTable<VariableControllerMaster, Binding> bindings = new ConditionalWeakTable<VariableControllerMaster, Binding>();
        private static readonly Dictionary<(Type, string), MemberInfo?> members = new Dictionary<(Type, string), MemberInfo?>();
        private readonly string runId;
        private readonly VariableControllerMaster controller;
        private object? construct, common;
        private string axis = "unknown", kind = "unknown";
        private int? nodeIndex;
        private string? failure;
        private readonly List<object> physics = new List<object>();
        private readonly List<double> rawTimes = new List<double>();
        private readonly List<double?> rawCommands = new List<double?>();
        private double nextPhysics;
        private bool emitted;
        internal static Action<object, string>? BeginPhysics;
        internal static Action<string>? EndPhysics;
        private bool physicsStarted;

        internal static void Bind(VariableControllerMaster controller, object source, string kind)
        { bindings.Remove(controller); bindings.Add(controller, new Binding { Source = source, Kind = kind }); }

        internal AutoTuneContext(VariableControllerMaster controller, string runId)
        {
            this.controller = controller; this.runId = runId;
            try
            {
                if (bindings.TryGetValue(controller, out var bound))
                {
                    kind = bound.Kind;
                    axis = Read(bound.Source, "PidType")?.ToString() ?? "unknown";
                    if (kind == "GP-PID") construct = Read(bound.Source, "MainConstruct");
                }
                if (construct == null)
                {
                    foreach (var assembly in AppDomain.CurrentDomain.GetAssemblies())
                    {
                        var manager = assembly.GetType("StaticConstructablesManager", false);
                        if (manager == null) continue;
                        var vehicles = manager.GetField("Constructables", BindingFlags.Public | BindingFlags.Static)?.GetValue(null) as IEnumerable;
                        if (vehicles != null) Resolve(vehicles);
                        break;
                    }
                }
                if (construct == null) failure = "Controller owner unresolved; physical fields unavailable.";
            }
            catch (Exception ex) { failure = "Owner lookup: " + ex.GetType().Name; }
        }

        // Also used by the host-independent regression test.
        internal void Resolve(IEnumerable vehicles)
        {
            foreach (var vehicle in vehicles)
            {
                var nodes = Read(Read(Read(vehicle, "NodeSets"), "AiNodes"), "Nodes") as IEnumerable;
                if (nodes == null) continue;
                int index = 0;
                foreach (var node in nodes)
                {
                    var candidate = Read(Read(node, "Master"), "Common");
                    foreach (string name in Axes)
                        if (ReferenceEquals(Read(candidate, name + "Control"), controller))
                        { construct = vehicle; common = candidate; axis = name; nodeIndex = index; kind = "AI-PID"; failure = null; return; }
                    index++;
                }
            }
        }
        private static readonly string[] Axes = { "Roll", "Pitch", "Yaw", "Hover", "Strafe", "ForwardBackward" };
        private static object? Read(object? owner, string name)
        {
            if (owner == null) return null;
            var key = (owner.GetType(), name);
            MemberInfo? member;
            lock (members)
            {
                if (!members.TryGetValue(key, out member))
                {
                    for (var type = key.Item1; type != null && member == null; type = type.BaseType)
                        member = (MemberInfo?)type.GetProperty(name, BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.DeclaredOnly)
                            ?? type.GetField(name, BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.DeclaredOnly);
                    members[key] = member;
                }
            }
            try { return member is PropertyInfo p ? p.GetValue(owner, null) : (member as FieldInfo)?.GetValue(owner); }
            catch { return null; }
        }
        private static double? Number(object? value)
        {
            if (value == null) return null;
            try { double v = Convert.ToDouble(value); return double.IsNaN(v) || double.IsInfinity(v) ? (double?)null : v; }
            catch { return null; }
        }
        private static double? N(object? owner, string name) => Number(Read(owner, name));
        private static double?[]? Vector(object? v) => v == null ? null : new[] { N(v, "x"), N(v, "y"), N(v, "z") };

        internal void Capture(double t)
        {
            if (emitted) return;
            try
            {
                if (!physicsStarted && construct != null) { physicsStarted = true; BeginPhysics?.Invoke(construct, runId); }
                rawTimes.Add(t); rawCommands.Add(N(controller, "LastControlVariable"));
                if (t + 1e-6 < nextPhysics) return;
                nextPhysics = t + .2;
                var platform = Read(construct, "PlatformPhysics");
                var velocities = Read(Read(construct, "PartPhysics"), "iVelocities");
                var other = new List<object>();
                if (common != null) foreach (string name in Axes)
                {
                    var c = Read(common, name + "Control");
                    other.Add(new { axis = name, sp = N(c, "LastSetPoint"), y = N(c, "LastProcessVariable"),
                        rawU = N(c, "LastControlVariable"), type = Read(Read(c, "Type"), "Us")?.ToString() });
                }
                physics.Add(new {
                    t, altitudeM = N(construct, "AltitudeOfComAboveMeanSeaLevel"),
                    airDensityModifier = N(platform, "AltitudeThrustModifier"),
                    speedMps = N(velocities, "VelocityMagnitude"),
                    localVelocityMps = Vector(Read(velocities, "LocalVelocityVector")),
                    localAngularVelocityRadPerSec = Vector(Read(velocities, "LocalAngularVelocityVector")),
                    attitudeDeg = new[] { N(construct, "Pitch"), N(construct, "Yaw"), N(construct, "Roll") },
                    localDragForce = Vector(Read(platform, "LocalDragForce")),
                    otherAxes = other
                });
            }
            catch (Exception ex) { failure = "Context capture: " + ex.GetType().Name; }
        }
        internal void Finish(string reason)
        {
            if (emitted) return;
            emitted = true;
            try { if (physicsStarted) EndPhysics?.Invoke(runId); } catch { }
            try
            {
                AutoTuneDiagnostics.Event("operatingContext", new {
                    schemaVersion = 1, runId, reason, kind, axis, nodeIndex,
                    controllerSessionIdentity = RuntimeHelpers.GetHashCode(controller),
                    constructId = N(construct, "UniqueId"), blueprintName = Read(construct, "blueprintName")?.ToString(),
                    ownerResolved = construct != null, diagnosticWarning = failure,
                    physicsPeriodSeconds = .2, vectorOrder = "x,y,z; attitude=pitch,yaw,roll",
                    rawTimes, rawCommands, physics,
                    notes = "rawCommands are pre-output-clamp PID requests, not delivered thrust. airDensityModifier is the game's altitude thrust factor, not kg/m^3. Physics samples may use the last completed physics update. Missing fields are null."
                });
            }
            catch { /* Telemetry must never stop tuning or cancellation. */ }
        }
    }
}
