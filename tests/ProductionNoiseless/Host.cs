using System;
using System.IO;
using System.Reflection;
using System.Collections.Generic;
using PIDSupporter;
using BrilliantSkies.Ai.Control.Pids;

// Session lifecycle tests use substitutes only for the game/Unity host. The linked
// AutoTuneSession and measurement hooks are the production source, not copies.
namespace UnityEngine { public static class Time { public static float realtimeSinceStartup; } }
namespace PIDSupporter { internal static class AutoTuneDiagnostics { internal static Action<string, object>? Sink; internal static void Event(string kind, object value) { Sink?.Invoke(kind, value); } } }
namespace HarmonyLib
{
    [AttributeUsage(AttributeTargets.Class)] public sealed class HarmonyPatch : Attribute
    { public HarmonyPatch(Type type, string name) { } }
}
namespace BrilliantSkies.Ai.Control.Pids
{
    public sealed class Var { public float Us; public Var(float x) { Us = x; } }
    public sealed class BoolVar { public bool Us; }
    public sealed class Pid
    {
        public Var kP = new Var(.05f), kI = new Var(2f), kD = new Var(.1f);
        public float LowerOutputLimit = -1, UpperOutputLimit = 1;
        public float TestError, TestIntegral;
        public void GetState(out float integral, out float error, out float time, out float accumulated)
        { time = accumulated = 0; integral=TestIntegral; error=TestError; }
    }
    public sealed class VariableControllerMaster
    {
        public float LastControlVariable { get; set; }
        public Pid Pid = new Pid(); public object? DataCollector;
        public Var SetPointMultiple = new Var(2f), SetPointAdjust = new Var(3f);
        public BoolVar FakeSetPointInUse=new BoolVar(); public Var FakeSetPoint=new Var(0);
        public object GetCurrentController() => Pid;
    }
}
