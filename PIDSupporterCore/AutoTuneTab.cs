using System;
using System.Runtime.CompilerServices;
using BrilliantSkies.Ai.Control.Pids;
using BrilliantSkies.Ui.Consoles;
using BrilliantSkies.Ui.Consoles.Getters;
using BrilliantSkies.Ui.Consoles.Interpretters.Subjective;
using BrilliantSkies.Ui.Consoles.Interpretters.Subjective.Buttons;
using BrilliantSkies.Ui.Consoles.Interpretters.Subjective.Choices;
using BrilliantSkies.Ui.Consoles.Segments;
using BrilliantSkies.Ui.Consoles.Styles;
using BrilliantSkies.Ui.Tips;
using UnityEngine;

namespace PIDSupporter
{
    public sealed class AutoTuneTab : SuperScreen<VariableControllerMaster>
    {
        private float speed = .5f;
        private AutoTuneSession? session;
        private readonly AutoTuneHistory history = new AutoTuneHistory();
        private int spectrumIndex=-1;
        internal AutoTuneSpectrum? Spectrum => history.SpectrumAt(spectrumIndex);
        internal string SpectrumSource => spectrumIndex<0?"Latest completed record":"Record "+(spectrumIndex+1);
        internal void SelectSpectrum(int index){spectrumIndex=index;}
        internal int RecordCount => history.Count;
        internal string RecordCell(int row,int column)=>history.Cell(row,column);
        internal string ProgressCaption => session==null?"Ready: prepare, then collect 30 s":session.CollectionStarted?$"Collection {session.CollectionElapsed:0.0} / 30 s":$"Gain preparation | {session.PreparationAttempts} adjustments";
        internal double CollectionProgress => session==null?0:Math.Max(0,Math.Min(1,session.CollectionElapsed/30));
        internal void KeepWindowAlive()
        { if(!IsClosed&&session!=null&&!session.Cancelled)session.Heartbeat=Time.realtimeSinceStartup; }
        internal string DataSummary => $"{history.Count} / {AutoTuneHistory.Capacity} completed records\n{history.Count*30} s collected | {history.Count*15} s used for fitting\nScope: this PID editor and this controller only";
        internal string DataRecords => history.Describe();
        internal string ExcitationPlan
        {
            get
            {
                double p=AutoTuneSession.ReadPid(_focus)[0];double cap=session?.Amplitude??(p>0?Math.Min(5,.1*(_focus.Pid.UpperOutputLimit-_focus.Pid.LowerOutputLimit)/p):0);
                return $"Injection: setpoint offset (SP)\nDuration: repeat 4 s gain checks, then 30 s collection | square wave\nFundamental: 0.200 Hz (5 s period; odd harmonics)\nOffset cap: +/-{cap:0.000} in PID reference units\nFixed collection amplitude: {session?.CollectionAmplitude??0:0.000}\nPrepare first; fresh independent carrier, fixed amplitude";
            }
        }
        internal string ExcitationLive => session==null?"Ready - no active test.":
            $"{(session.Cancelled?"Stopped":session.Collecting?session.Phase:"Collection complete")} | collection {session.CollectionElapsed:0.0} / 30 s\n"+
            $"Injected offset: {session.LiveOffset:+0.000;-0.000;0.000}\n"+
            $"Latest sampled reference: {session.LastReference:0.000}\nMeasured output: {session.LastOutput:0.000}\nPost-limit control: {session.LastInput:0.000}";
        internal bool IsClosed { get; private set; }
        internal void CloseHistory()
        {
            IsClosed=true;session?.Cancel("PID window closed; accumulated records cleared.");history.Close();session=null;
            message="PID window closed. Reopen the PID editor to start a new history.";
        }
        private sealed class SavedPid { internal double[] Value = null!; }
        private static readonly ConditionalWeakTable<VariableControllerMaster, SavedPid> Rollbacks = new ConditionalWeakTable<VariableControllerMaster, SavedPid>();
        private string message = "Ready. Keep this window open during collection and calculation.";
        public AutoTuneTab(ConsoleWindow window, VariableControllerMaster focus) : base(window, focus)
        { Name = new Content("Auto Tune", new ToolTip("Collect 30 seconds, estimate a model, review and apply a PID.", 280f), "autotune"); }

        public override void Build()
        {
            AddSegment(new AutoTuneVisual(this,this,AutoTuneVisual.View.Progress));
            var panel = Section("1. Tracking preference");
            Display(panel, () => $"Test reference: 63% of a change in {2 - 1.5 * speed:0.00} s",
                "Sets the reference used to compare PID candidates. This is not a promised vehicle response time. All three choices use the same collection waveform.");
            var choices = CreateStandardHorizontalSegment();
            choices.AddInterpretter(SpeedChoice("Gentle", 0f));
            choices.AddInterpretter(SpeedChoice("Balanced", .5f));
            choices.AddInterpretter(SpeedChoice("Fast", 1f));
            var collect = Section("2. Collect & calculate");
            Display(collect, Status, "Start from a working PID; preparation may adjust P. Require two consecutive 4-second gain observations to pass, then fix excitation and collect 30 seconds. Calculation runs in the background.");
            Display(collect, () => "Data tab: saved runs and PID settings\nExcitation tab: waveform plan and live signals\nGuide tab: workflow and result meanings", "All tabs belong to this PID window. Switching tabs does not clear collected data.");
            Display(collect, () => $"Accumulated: {history.Count} / {AutoTuneHistory.Capacity} records (this PID window only)",
                "Every complete test is retained, including successful tests and tests using different PID settings. The latest five records are fitted jointly. Closing this PID window clears all records; other axes never share this history.");
            var recording = CreateStandardHorizontalSegment();
            recording.AddInterpretter(Button("Start 30 s test", Start));
            recording.AddInterpretter(Button("Cancel", () => { session?.Cancel(); }));
            recording.AddInterpretter(Button("Clear accumulated data", () => {
                if(AutoTuneSession.Active!=null)return;history.Clear();spectrumIndex=-1;session=null;message="Accumulated records cleared. Start a new 30 s test.";
            }));
            var live = Section("3. Live signals");
            Display(live, () => session == null ? "No samples yet." :
                $"SP offset: {session.LiveOffset:+0.000;-0.000;0.000}\n" +
                $"Reference: {session.LastReference:0.000}    Output: {session.LastOutput:0.000}\n" +
                $"Control: {session.LastInput:0.000}    Collection: {session.CollectionElapsed:0.0} / 30 s",
                "Measured reference, output and post-limit control. During collection, the native Set point adjustment slider follows the applied excitation.");
            var resultPanel = Section("4. Review & apply");
            Display(resultPanel, Results, "Tracking error is prioritized. Among similarly performing candidates, smoother control is preferred. Predictions include nearby PID settings.");
            var actions = CreateStandardHorizontalSegment();
            actions.AddInterpretter(Button("Apply recommended PID", Apply));
            actions.AddInterpretter(Button("Restore previous PID", Undo));
        }

        private AutoTuneText Section(string title)
        {
            var section=new AutoTuneText(this,this,title);AddSegment(section);return section;
        }
        private void Display(AutoTuneText segment,Func<string> text,string tip)=>segment.Add(text,tip);

        // Mutually exclusive toggles: clicking the selected option cannot leave all options off.
        private SubjectiveToggle<VariableControllerMaster> SpeedChoice(string title, float value) =>
            new SubjectiveToggle<VariableControllerMaster>(_focus,
                M.m<VariableControllerMaster>(_ => title),
                M.m<VariableControllerMaster>(new ToolTip($"Test reference reaches 63% in {2 - 1.5 * value:0.00} s. Locked while collecting or calculating.", 320f)),
                (_, selected) =>
                {
                    if (!selected || speed == value || ReferenceEquals(AutoTuneSession.Active, session) && session != null) return;
                    speed = value;
                    session = null;
                    message = "Tracking preference changed. Start a new test.";
                }, null!, _ => speed == value, Array.Empty<string>());

        private SubjectiveButton<VariableControllerMaster> Button(string title, Action action) =>
            new SubjectiveButton<VariableControllerMaster>(_focus, M.m<VariableControllerMaster>(_ => title),
                M.m<VariableControllerMaster>(new ToolTip(title, 180f)), null!, _ => action());

        private static string PidText(double[] p) => $"P {p[0]:0.000} | Ti {(p[1] >= 250 ? "Off" : p[1].ToString("0.0") + " s")} | Td {(p[2] == 0 ? "Off" : p[2].ToString("0.0") + " s")}";

        private string Status()
        {
            // Only a rendered tuner refreshes this lease; closing its separate window cancels.
            KeepWindowAlive();
            return session?.Message ?? message;
        }

        private string Results()
        {
            var p = AutoTuneSession.ReadPid(_focus);
            string text = $"Current: {PidText(p)}";
            var r = session?.Result;
            if (session?.Cancelled != true && r?.accepted == true && r.pid != null)
                text += $"\nRecommended: {PidText(r.pid)}\nPredicted tracking cost: {100 * r.predicted_ratio:0.0}% of current\nLower is better. Model prediction, including nearby PID settings.";
            else if (session?.Cancelled != true && r != null) text += "\n" + r.message;
            else text += "\nRecommended: pending a successful test";
            if (r != null && r.seconds > 0) text += $"\nCalculation: {r.seconds:0.00} s";
            if (Rollbacks.TryGetValue(_focus, out _)) text += "\nPrevious PID saved: Restore is available.";
            return text;
        }

        public void OnUiFixed()
        {
            if (session == null) return;
            // An interrupted UI session must not resume a partially collected record.
            if (ReferenceEquals(AutoTuneSession.Active, session) && !session.CheckAlive()) return;
            session.Poll();
        }

        private void Start()
        {
            try { if(IsClosed)throw new InvalidOperationException("Reopen the PID window to start a new test.");session = new AutoTuneSession(_focus, speed, history); Rollbacks.Remove(_focus); Rollbacks.Add(_focus,new SavedPid{Value=(double[])session.Original.Clone()});spectrumIndex=-1; }
            catch (Exception ex) { message = ex.Message; if (AutoTuneSession.Active != session) session = null; }
        }

        private void Apply()
        {
            var r = session?.Result;
            if (AutoTuneSession.Active != null || r?.accepted != true || r.pid == null) return;
            var change = session!.SettingsChange();
            if (change != null) { session.Cancel("New test required: " + change); return; }
            var p = r.pid;
            if (p.Length != 3 || !Finite(p[0]) || !Finite(p[1]) || !Finite(p[2]) || p[0] < .001 || p[0] > 1 || p[1] < .01 || p[1] > 250 || p[2] < 0 || p[2] > 100)
            { session.Cancel("Invalid recommendation"); return; }
            var previous = AutoTuneSession.ReadPid(_focus);
            try
            {
                SetPid(p); Rollbacks.Remove(_focus); Rollbacks.Add(_focus, new SavedPid { Value = previous });
                AutoTuneDiagnostics.Event("apply", new { before = previous, after = AutoTuneSession.ReadPid(_focus) });
                session.Cancel("PID applied. Restore returns to the previous PID.");
            }
            catch (Exception ex) { SetPid(previous); session.Cancel("Apply failed; previous PID restored: " + ex.Message); }
        }

        private void Undo()
        {
            if (!Rollbacks.TryGetValue(_focus, out var rollback)) return;
            session?.Cancel(); SetPid(rollback.Value); Rollbacks.Remove(_focus); session = null;
            AutoTuneDiagnostics.Event("undo", new { after = AutoTuneSession.ReadPid(_focus) });
            message = "Previous PID restored.";
        }

        private void SetPid(double[] p)
        { _focus.Pid.kP.Us = (float)p[0]; _focus.Pid.kI.Us = (float)p[1]; _focus.Pid.kD.Us = (float)p[2]; }
        private static bool Finite(double x) => !double.IsNaN(x) && !double.IsInfinity(x);
    }
}
