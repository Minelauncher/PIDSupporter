using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using BrilliantSkies.Ai.Control.Pids;
using HarmonyLib;
using UnityEngine;

namespace PIDSupporter
{
    internal sealed class TuneResult
    {
        public bool accepted { get; set; }
        public string reason { get; set; } = "";
        public double[]? pid { get; set; }
        public double validation { get; set; }
        public double predicted_ratio { get; set; }
        public double seconds { get; set; }
        public string message { get; set; } = "";
        public int accumulated_records { get; set; }
        public bool needs_more_data { get; set; }
    }

    internal sealed class AutoTuneSession
    {
        // One active experiment/learner globally: axes must not excite simultaneously.
        internal static AutoTuneSession? Active;
        internal readonly VariableControllerMaster Controller;
        internal readonly double[] Original;
        internal readonly double[] WorkingPid;
        internal readonly int ExcitationSeed;
        internal readonly int PreparationSeed;
        internal bool CollectionStarted { get; private set; }
        internal double CollectionAmplitude { get; private set; }
        internal string Phase => CollectionStarted ? "Collecting (fixed PID / excitation)" : "Preparing gain / excitation";
        internal readonly float Speed;
        internal double Amplitude { get; private set; }
        internal readonly double Lower, Upper;
        internal readonly float Multiple;
        internal float Heartbeat;
        internal bool Collecting = true;
        internal bool Cancelled;
        internal volatile string Message = "Observing gain: 0 / 4 s";
        internal TuneResult? Result;
        internal double Elapsed;
        internal const double ObservationSeconds=4;
        internal double CollectionElapsed => CollectionStarted?Math.Max(0,Elapsed-collectionStart):0;
        internal int PreparationAttempts { get; private set; }
        private double collectionStart,observationStart;
        private bool gainPassed;
        private string gainReason="initial observation";
        internal readonly string DiagnosticId = Guid.NewGuid().ToString("N");
        private readonly AutoTuneContext context;
        private readonly AutoTuneHistory history;
        private readonly List<double> u = new List<double>(), y = new List<double>(), t = new List<double>();
        private readonly List<double> references = new List<double>(), integrals = new List<double>();
        private readonly List<double> injected = new List<double>(), baseReferences = new List<double>();
        private readonly List<double> instruments = new List<double>();
        private int passedGainWindows;
        private double previousPrepU, prepHighFrequency;
        private bool havePrepU;
        private readonly List<double> prepU=new List<double>(),prepY=new List<double>(),prepR=new List<double>(),prepT=new List<double>();
        private double prepPeak;

        private int prepCount, prepSaturated, prepReversals;
        private double prepEffort, prepError, previousPrepError=double.PositiveInfinity, previousDelta, previousY;
        private bool havePreviousY;
        private readonly float adjust;
        private readonly bool fakeSetPointInUse;
        private readonly float fakeSetPoint;
        private float displayedAdjust;
        internal double LiveOffset { get; private set; }
        internal double LastReference { get; private set; }
        internal double LastOutput { get; private set; }
        internal double LastInput { get; private set; }
        private double? lastTime;
        private double waveTime;
        private Task<TuneResult>? work;
        private readonly CancellationTokenSource cancellation = new CancellationTokenSource();
        internal const double SquareFrequency = .2;
        private double squarePhase, squareSign, squareCenter;
        private const double FadeSeconds = 1.0;

        internal AutoTuneSession(VariableControllerMaster controller, float speed, AutoTuneHistory? history = null)
        {
            if (Active != null) throw new InvalidOperationException("Another auto-tune is active.");
            Controller = controller;
            this.history = history ?? new AutoTuneHistory();
            if (!ReferenceEquals(controller.GetCurrentController(), controller.Pid) || controller.DataCollector != null)
                throw new InvalidOperationException("Select PID control before auto-tuning.");
            Original = ReadPid(controller); WorkingPid=(double[])Original.Clone();
            ExcitationSeed=Guid.NewGuid().GetHashCode(); PreparationSeed=Guid.NewGuid().GetHashCode();
            if (Original[0] <= 0) throw new InvalidOperationException("A working positive PID gain is required.");
            Speed = speed; Multiple = controller.SetPointMultiple.Us; adjust = controller.SetPointAdjust.Us;
            displayedAdjust = adjust;
            fakeSetPointInUse=controller.FakeSetPointInUse.Us;fakeSetPoint=controller.FakeSetPoint.Us;
            Lower = controller.Pid.LowerOutputLimit; Upper = controller.Pid.UpperOutputLimit;
            if (Math.Abs(Multiple) < 1e-6 || !(Lower < Upper)) throw new InvalidOperationException("Invalid controller limits");
            RefreshExcitationBasis();
            Heartbeat = Time.realtimeSinceStartup;
            Active = this;
            context = new AutoTuneContext(controller, DiagnosticId);
            AutoTuneDiagnostics.Event("start", new { runId = DiagnosticId, pid = Original, speed = Speed, amplitude = Amplitude,
                excitation = "sp-square-independent-carrier-v6", fakeSetPointInUse, fakeSetPoint, seed=ExcitationSeed, preparationSeed=PreparationSeed, observationSeconds=ObservationSeconds, identificationSeconds=30, fadeSeconds = FadeSeconds,
                frequencyHz=SquareFrequency, amplitudePolicy="existing-P-cap-and-observed-peak" });
        }

        // Called only before design, never while collecting. Rebase amplitude as well as
        // its cap: retaining the old amplitude would keep the original-P attenuation.
        private void RefreshExcitationBasis()
        {
            Amplitude=Math.Min(5.0,.1*(Upper-Lower)/WorkingPid[0]);
            CollectionAmplitude=.5*Amplitude;

        }

        internal static double[] ReadPid(VariableControllerMaster c) => new double[] { c.Pid.kP.Us, c.Pid.kI.Us, c.Pid.kD.Us };
        internal bool MatchesPid()
        {
            var current = ReadPid(Controller);
            for (int i = 0; i < 3; i++) if (Math.Abs(current[i] - WorkingPid[i]) > 1e-6) return false;
            return true;
        }

        internal bool MatchesSettings() => SettingsChange() == null;
        internal string? SettingsChange()
        {
            var current = ReadPid(Controller);
            for (int i = 0; i < 3; i++)
                if (Math.Abs(current[i] - WorkingPid[i]) > 1e-6)
                    return $"{(i == 0 ? "P" : i == 1 ? "Ti" : "Td")} changed ({WorkingPid[i]:G6} → {current[i]:G6})";
            if(Controller.FakeSetPointInUse.Us!=fakeSetPointInUse)return "Fake SP mode changed";
            if(fakeSetPointInUse && Controller.FakeSetPoint.Us!=fakeSetPoint)return "Fake SP value changed";
            if (Controller.SetPointMultiple.Us != Multiple) return $"SP multiplier changed ({Multiple:G6} → {Controller.SetPointMultiple.Us:G6})";
            if (Controller.SetPointAdjust.Us != displayedAdjust) return $"SP offset changed ({displayedAdjust:G6} -> {Controller.SetPointAdjust.Us:G6})";
            if (!ReferenceEquals(Controller.GetCurrentController(), Controller.Pid)) return "Active controller changed";
            if (Controller.DataCollector != null) return "Another tuning function is active";
            if (Controller.Pid.LowerOutputLimit != Lower) return $"Output lower limit changed ({Lower:G6} → {Controller.Pid.LowerOutputLimit:G6})";
            if (Controller.Pid.UpperOutputLimit != Upper) return $"Output upper limit changed ({Upper:G6} → {Controller.Pid.UpperOutputLimit:G6})";
            return null;
        }

        internal void SetExcitation(double wave)
        {
            // Hold the sampled SP offset between measurements so the native SP
            // slider renders the real value. Always reference the original offset.
            displayedAdjust = adjust + (float)(wave / Multiple);
            Controller.SetPointAdjust.Us = displayedAdjust;
            LiveOffset = (displayedAdjust - adjust) * Multiple;
        }

        private void RestoreExcitation()
        {
            // Do not overwrite an external/manual edit that caused cancellation.
            if (Controller.SetPointAdjust.Us == displayedAdjust)
                Controller.SetPointAdjust.Us = adjust;
            displayedAdjust = adjust;
            LiveOffset = 0;
        }

        internal double Wave(double seconds) => CollectionAmplitude * Carrier(seconds);
        // Unscaled carrier, drawn after preparation. Do not divide rounded SP by amplitude.
        internal double Carrier(double seconds)
        {
            if(!CollectionStarted)return 0;
            double local=seconds-collectionStart;
            if(local<=0 || local>=30)return 0;
            return squareSign*Envelope(local)*(Square(local)-squareCenter)/(1+Math.Abs(squareCenter));
        }
        private static double Envelope(double local)
        {
            double edge=Math.Min(local,30-local);
            return edge>=FadeSeconds?1:.5*(1-Math.Cos(Math.PI*edge/FadeSeconds));
        }
        private double Square(double local)=>Math.Sin(2*Math.PI*SquareFrequency*local+squarePhase)>=0?1:-1;

        internal bool CheckAlive()
        {
            if (Cancelled) return false;
            if (Time.realtimeSinceStartup - Heartbeat > .5f)
            { Cancel("Window inactive; test cancelled."); return false; }
            var change = SettingsChange();
            if (change != null)
            { AutoTuneDiagnostics.Event("settingsChanged", new { reason = change }); Cancel("Test cancelled: " + change); return false; }
            return true;
        }

        internal double? BeginSample(double timestamp)
        {
            if (!Collecting || !CheckAlive()) return null;
            if (double.IsNaN(timestamp) || double.IsInfinity(timestamp)) { Cancel("Invalid controller time"); return null; }
            if (!lastTime.HasValue) { lastTime = timestamp; return null; }
            double dt = timestamp - lastTime.Value;
            if (dt == 0) return null;
            if (dt < 0 || dt > .1) { Cancel("Controller timing interrupted."); return null; }
            lastTime = timestamp;
            if(!CollectionStarted && gainPassed)FreezeCollectionPlan();
            if(Cancelled)return null;
            if (CollectionStarted && CollectionElapsed + dt > 30.0001)
            { FinishCollection(); return null; }
            double before=Elapsed;
            Elapsed += dt;
            waveTime=before;
            return Wave(waveTime);
        }

        internal void EndSample(double actualU, double measuredY)
        {
            if (Cancelled) return;
            if (double.IsNaN(actualU) || double.IsInfinity(actualU) || double.IsNaN(measuredY) || double.IsInfinity(measuredY))
            { Cancel("Invalid measurements."); return; }
            // Record NewMeasurement's returned post-limit command, not LastControlVariable (pre-limit).
            Controller.Pid.GetState(out float integral, out float error, out _, out _);
            LastReference = measuredY + error; LastOutput = measuredY; LastInput = actualU;
            // Preparation may repeat indefinitely; retain full-rate context only for the fixed record.
            if(CollectionStarted)context.Capture(CollectionElapsed);
            if(!CollectionStarted) {
                if(!gainPassed){
                    prepU.Add(actualU);prepY.Add(measuredY);prepR.Add(LiveOffset);prepT.Add(Elapsed);prepPeak=Math.Max(prepPeak,Math.Abs(actualU));
                    PrepareGain(actualU,measuredY,error);
                    if(!gainPassed)Message=$"Observing gain: {Elapsed-observationStart:0.0} / {ObservationSeconds:0} s | adjustments {PreparationAttempts} | P {WorkingPid[0]:0.000} | {gainReason}";
                }
                return;
            }
            references.Add(measuredY + error); integrals.Add(integral);
            injected.Add(LiveOffset); baseReferences.Add(measuredY + error - LiveOffset);
            LastReference = measuredY + error; LastOutput = measuredY; LastInput = actualU;
            u.Add(actualU); y.Add(measuredY); t.Add(CollectionElapsed);
            instruments.Add(Carrier(waveTime));
            Message = $"Collecting fixed SP response: {CollectionElapsed:0.0} / 30 s | preparation {collectionStart:0.0} s";
            if (CollectionElapsed >= 29.9999) FinishCollection();
        }

        private void PrepareGain(double actualU,double measuredY,double error)
        {
            double limit=Math.Max(Math.Abs(Lower),Math.Abs(Upper));prepCount++;
            if(actualU<=Lower+1e-5||actualU>=Upper-1e-5)prepSaturated++;
            prepEffort+=Math.Abs(actualU)/limit;prepError+=error*error;
            if(havePrepU)prepHighFrequency+=Math.Pow((actualU-previousPrepU)/(2*limit),2);
            previousPrepU=actualU;havePrepU=true;
            if(havePreviousY){double delta=measuredY-previousY;if(Math.Abs(delta)>1e-5&&delta*previousDelta<0)prepReversals++;previousDelta=delta;}
            havePreviousY=true;previousY=measuredY;
            if(Elapsed-observationStart+1e-6<ObservationSeconds)return;
            double rms=Math.Sqrt(prepError/prepCount),sat=(double)prepSaturated/prepCount;
            double highFrequencyRms=Math.Sqrt(prepHighFrequency/Math.Max(1,prepCount-1));
            double factor=highFrequencyRms>.2?.7:AdaptiveExcitation.GainFactor(sat,prepEffort/prepCount,rms,previousPrepError,prepReversals);
            if(factor==1){
                passedGainWindows++;
                if(passedGainWindows<2){gainReason="first observation passed; confirming";ResetGainWindow(rms);return;}
                gainPassed=true;
                AutoTuneDiagnostics.Event("gainObservationPassed",new{seconds=Elapsed,pid=(double[])WorkingPid.Clone(),saturation=sat,errorRms=rms,adjustments=PreparationAttempts,highFrequencyRms,passedWindows=passedGainWindows,stableNotGuaranteed=true});
                RequestPreparationPlan();Message="Gain observations passed; freezing square-wave collection...";
                return;
            }
            passedGainWindows=0;
            double before=WorkingPid[0];Controller.Pid.kP.Us=(float)Math.Max(.001,Math.Min(1,before*factor));WorkingPid[0]=Controller.Pid.kP.Us;
            gainReason=factor<1?"saturation / growing oscillation":"low effort with persistent error";
            if(before==WorkingPid[0])gainReason+=" (gain limit; retrying)";
            PreparationAttempts++;
            AutoTuneDiagnostics.Event("preparationGain",new{seconds=Elapsed,before,after=WorkingPid[0],saturation=sat,errorRms=rms,attempt=PreparationAttempts,
                reason=factor<1?"saturation or growing oscillation":"low effort with persistent error",atGainLimit=before==WorkingPid[0]});
            RefreshExcitationBasis();
            ResetGainWindow(rms);
        }

        private void ResetGainWindow(double rms)
        {
            previousPrepError=rms;prepCount=prepSaturated=prepReversals=0;prepEffort=prepError=prepPeak=0;
            havePreviousY=false;previousDelta=0;havePrepU=false;prepHighFrequency=0;observationStart=Elapsed;
            prepU.Clear();prepY.Clear();prepR.Clear();prepT.Clear();
        }

        private void RequestPreparationPlan()
        {
            double oldCap=Amplitude,oldAmplitude=CollectionAmplitude;
            RefreshExcitationBasis();
            // Existing amplitude policy, no new RMS/response-bound formula.
            double limit=Math.Min(Math.Abs(Lower),Math.Abs(Upper));
            double amplitudeFactor=prepPeak>1e-8?Math.Max(.5,Math.Min(1.5,.6*limit/prepPeak)):1;
            CollectionAmplitude=Math.Min(Amplitude,CollectionAmplitude*amplitudeFactor);
            AutoTuneDiagnostics.Event("excitationBasisRefreshed",new{pid=(double[])WorkingPid.Clone(),oldCap,oldAmplitude,cap=Amplitude,amplitude=CollectionAmplitude,amplitudeFactor,prepPeak,policy="existing-P-cap-and-observed-peak"});
        }

        private void FreezeCollectionPlan()
        {
            var random=new System.Random(ExcitationSeed);
            squarePhase=random.NextDouble()*2*Math.PI;squareSign=random.Next(2)==0?-1:1;
            double sum=0,total=0;
            // Bounded one-time setup: approximate tapered mean on a 20 Hz midpoint grid.
            for(int i=0;i<600;i++){double t=(i+.5)*.05,e=Envelope(t);sum+=e*Square(t);total+=e;}
            squareCenter=sum/total;
            collectionStart=Elapsed;CollectionStarted=true;
            AutoTuneDiagnostics.Event("collectionPlanFrozen",new{seconds=Elapsed,pid=(double[])WorkingPid.Clone(),
                amplitude=CollectionAmplitude,cap=Amplitude,seed=ExcitationSeed,frequencyHz=SquareFrequency,
                phase=squarePhase,sign=squareSign,center=squareCenter,policy="square-fixed-30s",
                duration=30,instrument="independent-unit-carrier-before-amplitude-and-quantization"});
        }

        private void FinishCollection()
        {
            Collecting = false;
            context.Finish("collection_complete");
            RestoreExcitation();
            var request = new TuneRecord { R = references.ToArray(), InjectedSP = injected.ToArray(), BaseSP = baseReferences.ToArray(), Integrals = integrals.ToArray(), U = u.ToArray(), Y = y.ToArray(), T = t.ToArray(), Pid = (double[])WorkingPid.Clone(), Duration=30, InstrumentSP=instruments.ToArray(), ExcitationSeed=ExcitationSeed,
                Speed = Speed, Amplitude = CollectionAmplitude, Lower = Lower, Upper = Upper };
            Controller.Pid.GetState(out float integral, out float previousError, out _, out _);
            request.InitialIntegral = integral; request.InitialError = previousError;
            Message = "Estimating a linear model from the SP response...";
            AutoTuneDiagnostics.Event("collected", new { samples = u.Count, seconds = CollectionElapsed, preparationSeconds=collectionStart });
            var completion = new TaskCompletionSource<TuneResult>(TaskCreationOptions.RunContinuationsAsynchronously);
            work = completion.Task;
            var token = cancellation.Token;
            // One dedicated managed thread; no external process, thread-pool starvation,
            // or Unity/game access. Only copied IO and scalar settings cross this boundary.
            var worker = new Thread(() =>
            {
                try
                {
                    token.ThrowIfCancellationRequested();
                    var records = history.Add(request, Multiple, token);
                    AutoTuneDiagnostics.Event("accumulation", new { runId=DiagnosticId, records=records.Length, capacity=AutoTuneHistory.Capacity });
                    var result = NativeAutoTune.Run(request, token, text => { if (!token.IsCancellationRequested) Message = text; }, records: records);
                    result.accumulated_records=records.Length;
                    token.ThrowIfCancellationRequested(); completion.TrySetResult(result);
                }
                catch (OperationCanceledException) { completion.TrySetCanceled(); }
                catch (Exception ex) { completion.TrySetResult(new TuneResult { message = "Calculation failed; current PID retained: " + ex.Message }); }
            }) { IsBackground = true, Name = "PIDSupporter Linear", Priority = System.Threading.ThreadPriority.BelowNormal };
            worker.Start();
        }

        internal void Poll()
        {
            if (Cancelled) return;
            if (work != null && work.IsCompleted)
            {
                try { Result = work.GetAwaiter().GetResult(); Message = Result.message; AutoTuneDiagnostics.Event("result", Result); }
                catch (Exception ex) { Message = ex.Message; }
                if (ReferenceEquals(Active, this)) Active = null;
                work = null;
            }
        }

        internal void Cancel(string reason = "Cancelled; current PID retained.")
        {
            context.Finish(reason);
            Cancelled = true; Collecting = false; Result = null;
            RestoreExcitation();
            cancellation.Cancel(); Message = reason;
            AutoTuneDiagnostics.Event("end", new { reason });
            if (ReferenceEquals(Active, this)) Active = null;
        }
    }

    [HarmonyPatch(typeof(VariableControllerMaster), "NewMeasurement")]
    internal static class AutoTuneMeasurementPatch
    {
        internal sealed class SampleState
        {
            internal AutoTuneSession Session = null!;
        }

        internal static void Prefix(VariableControllerMaster __instance, float time, out SampleState? __state)
        {
            __state = null;
            var s = AutoTuneSession.Active;
            if (s == null || !ReferenceEquals(s.Controller, __instance)) return;
            double? wave = s.BeginSample(time);
            if (!wave.HasValue) return;
            __state = new SampleState { Session = s };
            s.SetExcitation(wave.Value);
        }

        internal static void Postfix(VariableControllerMaster __instance, float processVariables, float __result, SampleState? __state)
        {
            if (__state == null) return;
            __state.Session.EndSample(__result, processVariables);
        }

        internal static Exception? Finalizer(VariableControllerMaster __instance, Exception? __exception, SampleState? __state)
        {
            if (__state != null)
            {
                if (__exception != null) __state.Session.Cancel("Measurement failed.");
            }
            return __exception;
        }
    }
}
