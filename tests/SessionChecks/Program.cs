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
        public float TestError;
        public void GetState(out float integral, out float error, out float time, out float accumulated)
        { integral = time = accumulated = 0; error=TestError; }
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

internal static class Program
{
    static void Assert(bool condition, string name) { if (!condition) throw new Exception(name); }
    static void Main(string[] args)
    {
        if(args.Length==3 && args[0]=="--robust-release") { RobustReleaseChecks.Run(args[1],args[2]); return; }
        if(args.Length==4 && args[0]=="--stabilized-square") { StabilizedSquareChecks.Run(args[1],args[2],int.Parse(args[3])); return; }
        if(args.Length==4 && args[0]=="--square-design") { SquareDesignChecks.Run(args[1],args[2],int.Parse(args[3])); return; }
        if(args.Length==4 && args[0]=="--smooth-holdout") { SmoothHoldoutChecks.Run(args[1],args[2],int.Parse(args[3])); return; }
        if(args.Length==3 && args[0]=="--online-energy") { OnlineEnergyChecks.Run(args[1],int.Parse(args[2])); return; }
        if(args.Length==3 && args[0]=="--waveform-energy") { EnergyMatchedWaveformChecks.Run(args[1],int.Parse(args[2])); return; }
        if(args.Length==3 && args[0]=="--waveform-comparison") { WaveformChecks.Run(args[1],int.Parse(args[2])); return; }
        if(args.Length==3 && args[0]=="--pilot-amplitude") { PilotAmplitudeChecks.Run(args[1],int.Parse(args[2])); return; }
        if(args.Length==3 && args[0]=="--nonlinear-causes") { NonlinearCauseChecks.Run(args[1],int.Parse(args[2])); return; }
        if(args.Length==3 && args[0]=="--coupled-saturation") { CoupledAmplitudeSaturationChecks.Run(args[1],int.Parse(args[2])); return; }
        if(args.Length==2 && args[0]=="--roll-geometry") { RollGeometryReview.Run(args[1]); return; }
        if(args.Length==3 && args[0]=="--quiet-other-axes") { QuietOtherAxesChecks.Run(args[1],int.Parse(args[2])); return; }
        if(args.Length==3 && args[0]=="--coupled-amplitude") { CoupledAmplitudeChecks.Run(args[1],int.Parse(args[2])); return; }
        if(args.Length==2 && args[0]=="--roll-attribution") { RollAttributionChecks.Run(args[1]); return; }
        if(args.Length==1 && args[0]=="--eoptimal-production") { EOptimalProductionChecks.Run(); return; }
        if(args.Length==1 && args[0]=="--eoptimal-audit") { EOptimalChecks.Audit(); return; }
        if(args.Length==1 && args[0]=="--eoptimal") { EOptimalChecks.Run(); return; }
        if(args.Length==1 && args[0]=="--adaptive-numerics") { AdaptiveChecks.Numerics(); return; }
        if(args.Length==1 && args[0]=="--adaptive") { AdaptiveChecks.Run(); return; }
        if(args.Length==3 && args[0]=="--post-ridge-review") { PostRidgeReview.Run(args[1],args[2]); return; }
        if(args.Length==3 && args[0]=="--constrained-full") { BandwidthLogReview.Full(args[1],args[2]); return; }
        if(args.Length==1 && args[0]=="--constraints") { ConstraintChecks.Run(); return; }
        if(args.Length==3 && args[0]=="--negative-gain-review") { NegativeGainReview.Run(args[1],args[2]); return; }
        if(args.Length==3 && args[0]=="--bandwidth-log-review") { BandwidthLogReview.Run(args[1],args[2]); return; }
        if (args.Length==1 && args[0]=="--accumulation") { AccumulationChecks.Run(); return; }
        if (args.Length==1 && args[0]=="--spectrum") { SpectrumChecks.Run(); return; }
        if (args.Length == 1 && args[0] == "--quaternion-model-review") { QuaternionModelReview.Run(); return; }
        if (args.Length == 1 && args[0] == "--pitch-confound-review") { PitchConfoundReview.Run(); return; }
        if (args.Length == 1 && args[0] == "--pitch-rate-review") { PitchRateReview.Run(); return; }
        if (args.Length == 1 && args[0] == "--pitch-diagnosis") { PitchDiagnosis.Run(); return; }
        if (args.Length == 1 && args[0] == "--context") { ContextChecks.Run(); return; }
        if (args.Length == 1 && args[0] == "--refined-iv-review") { RefinedIvReview.Run(); return; }
        if (args.Length == 1 && args[0] == "--ensemble-review") { EnsembleReview.Run(); return; }
        if (args.Length == 1 && args[0] == "--low-order-regression") {
            Assert(NativeAutoTune.DecisionReason(1,1)=="insufficient_improvement", "no improvement reason");
            Assert(NativeAutoTune.DecisionReason(1,1e12)=="baseline_prediction_failed", "invalid baseline is not improvement");
            Assert(NativeAutoTune.DecisionReason(1e12,1)=="candidate_prediction_failed", "candidate failure reason");
            Assert(NativeAutoTune.DecisionReason(double.NaN,1)=="candidate_prediction_failed", "nonfinite failure reason");
            Assert(NativeAutoTune.DecisionReason(.9,1)=="recommended", "improvement reason");
            LowOrderRegression.Run(); return;
        }
        if (args.Length == 2 && args[0] == "--information-dose") { InfoStudy.Run(args[1]); return; }
        if (args.Length == 1 && args[0] == "--grade-common") { GradeCommon.Run(); return; }
        if (args.Length == 2 && args[0] == "--common-plant") { BatchCommon.Run(args[1]); return; }
        if (args.Length == 2 && args[0] == "--repeat-synthetic") { RepeatStudy.Run(args[1]); return; }
        if (args.Length == 1 && args[0] == "--joint-records") { JointStudy.Run(); return; }
        if (args.Length == 1 && args[0] == "--state-order") { StateStudy.Run(); return; }
        if (args.Length == 1 && args[0] == "--injected-iv") { InjectedIvComparison.Run(); return; }
        if (args.Length == 2 && args[0] == "--legacy-study") { LegacyStudy.Run(args[1]); return; }
        if (args.Length == 2 && args[0] == "--morelli-study") { MorelliStudy.Run(args[1]); return; }
        if (args.Length == 2 && args[0] == "--information-study") { InformationStudy.Run(args[1]); return; }
        if (args.Length == 1 && args[0] == "--cross-bias") { CrossBias.Run(); return; }
        if (args.Length == 2 && args[0] == "--cross-study") { CrossStudy.Run(args[1]); return; }
        if (args.Length == 1 && args[0] == "--adaptive-bias") { AdaptiveBias.Run(); return; }
        if (args.Length == 2 && args[0] == "--adaptive-study") { AdaptiveStudy.Run(args[1]); return; }
        if (args.Length == 2 && args[0] == "--band-phase-study") { BandPhaseStudy.Run(args[1]); return; }
        if (args.Length == 2 && args[0] == "--phase-study") { PhaseStudy.Run(args[1]); return; }
        if (Array.IndexOf(args, "--iv") >= 0) { IvChecks.Run(); return; }
        if (Array.IndexOf(args, "--regularized-indirect") >= 0) { SpPidComparison.Run(true,true); return; }
        if (Array.IndexOf(args, "--indirect-pid-comparison") >= 0) { SpPidComparison.Run(true); return; }
        if (Array.IndexOf(args, "--sp-pid-comparison") >= 0) { SpPidComparison.Run(); return; }
        if (Array.IndexOf(args, "--excitation-wave") >= 0) { ExcitationChecks.Run(); return; }
        if (args.Length==2 && args[0]=="--excitation-study") { ExcitationStudy.Run(args[1]); return; }
        if (Array.IndexOf(args, "--duster-reference") >= 0) {
            var r=System.Text.Json.JsonDocument.Parse(System.Linq.Enumerable.First(System.Linq.Enumerable.Select(File.ReadLines("tests/output/duster-diagnostic/source.jsonl"),l=>System.Text.Json.JsonDocument.Parse(l).RootElement.Clone()),e=>e.GetProperty("kind").GetString()=="record").GetProperty("value").GetRawText()).RootElement;
            var c0=new VariableControllerMaster();var pp=r.GetProperty("Pid");c0.Pid.kP.Us=pp[0].GetSingle();c0.Pid.kI.Us=pp[1].GetSingle();c0.Pid.kD.Us=pp[2].GetSingle();
            var ss=new AutoTuneSession(c0,.5f);var values=new List<double>();double prev=0;
            foreach(var t in r.GetProperty("T").EnumerateArray()){values.Add(ss.Wave(prev));prev=t.GetDouble();}
            var rng=new Random(826);var phases=new double[60];for(int j=0;j<60;j++)phases[j]=rng.NextDouble()*2*Math.PI;
            File.WriteAllText("tests/output/duster-diagnostic/reference.json",System.Text.Json.JsonSerializer.Serialize(new {wave=values,phases}));return;
        }
        if (Array.IndexOf(args, "--duster-closedren-landscape") >= 0) { DusterLandscape.Run(false,false,false,true); return; }
        if (Array.IndexOf(args, "--duster-closedloop-landscape") >= 0) { DusterLandscape.Run(false,true,true); return; }
        if (Array.IndexOf(args, "--duster-indirect-landscape") >= 0) { DusterLandscape.Run(false,true); return; }
        if (Array.IndexOf(args, "--duster-duration") >= 0) { DusterDuration.Run(); return; }
        if (Array.IndexOf(args, "--duster-landscape") >= 0) { DusterLandscape.Run(); return; }
        if (Array.IndexOf(args, "--duster-encoder-landscape") >= 0) { DusterLandscape.Run(true); return; }
        if (Array.IndexOf(args, "--duster-joint") >= 0) { DusterLinear.Run(true); return; }
        if (Array.IndexOf(args, "--duster-linear") >= 0) { DusterLinear.Run(); return; }
        if (Array.IndexOf(args, "--linear") >= 0) { LinearChecks.Run(); return; }
        if (Array.IndexOf(args, "--native") >= 0) { NativeChecks.Run(args); return; }
        ProtocolChecks.Run();
    }
}
