using System;
using BrilliantSkies.Ai.Control.Pids;
using BrilliantSkies.Ui.Consoles;
using BrilliantSkies.Ui.Consoles.Getters;
using BrilliantSkies.Ui.Consoles.Interpretters.Subjective;
using BrilliantSkies.Ui.Consoles.Interpretters.Subjective.Buttons;
using BrilliantSkies.Ui.Consoles.Segments;
using BrilliantSkies.Ui.Consoles.Styles;
using BrilliantSkies.Ui.Tips;
namespace PIDSupporter
{
    // All panels share their owning tuner's lease; reading the guide must not cancel collection.
    internal sealed class AutoTuneInfoTab : SuperScreen<VariableControllerMaster>
    {
        private readonly AutoTuneTab owner;
        private readonly string kind;
        internal AutoTuneInfoTab(ConsoleWindow window,VariableControllerMaster focus,AutoTuneTab owner,string kind):base(window,focus)
        { this.owner=owner;this.kind=kind;Name=new Content(kind,new ToolTip(kind+" for this auto-tune window",260f),"autotune-"+kind.ToLowerInvariant()); }
        private void Panel(string title,Func<string> content,string tip)
        {
            var panel=new AutoTuneText(this,owner,title);panel.Add(content,tip);AddSegment(panel);
        }
        public override void Build()
        {
            if(kind=="Data")
            {
                AddSegment(new AutoTuneVisual(this,owner,AutoTuneVisual.View.Progress));
                Panel("Collection overview",()=>owner.DataSummary,"Preparation repeats 4-second gain observations without an attempt limit. After two consecutive passes, collect a separate 30 seconds with fixed PID and excitation. Preparation is excluded from identification.");
                AddSegment(new AutoTuneVisual(this,owner,AutoTuneVisual.View.Records));
                Panel("Bandwidth - select a run above",()=>owner.Spectrum==null?"Waiting for a completed record.":$"{owner.SpectrumSource}\n30 s collection spectrum | {owner.Spectrum.Windows} overlapping 10 s windows\nNo averaging across different PID records.","Click a run number to inspect its bandwidth. A new test resets the view to the latest completed record. Analysis is calculated once on the background worker.");
                AddSegment(new AutoTuneVisual(this,owner,AutoTuneVisual.View.Spectrum));
                AddSegment(new AutoTuneVisual(this,owner,AutoTuneVisual.View.Bands));
                AddSegment(new AutoTuneVisual(this,owner,AutoTuneVisual.View.Coherence));
                Panel("Read the plots",()=>"Coverage: each curve has its own 0 dB peak.\nUse the RMS table to compare absolute levels.\n\nCoherence is not a pass/fail quality score.\nBelow 0.1 Hz: insufficient resolution here.\nGaps: missing or very weak excitation.","Hann spectra include transients and other-axis motion. Overlapping Welch windows provide only a rough coherence estimate; high values can still include coupled responses. Bins more than 60 dB below the strongest injected-SP Welch power are hidden. This display threshold is not an identification gate.");
                Panel("How records are combined",()=>"Shared plant coefficients\nSeparate starting states and offsets\nEach record replayed with its own PID\n\nPer record: 15 s fit / 5 s select / 10 s check","The last 10 seconds are diagnostic only. More data does not guarantee a correct model.");
                Panel("History lifetime",()=>"Latest 5 completed runs are kept.\nRun 6 replaces the oldest run.\n\nClosing this PID editor or tuner clears all runs.\nOther PID windows and axes never share them.\nUse Clear accumulated data after changing the setup.","History stays in memory for this window only. Output limits, SP multiplier or sample-time changes reset incompatible history.");
            }
            else if(kind=="Excitation")
            {
                AddSegment(new AutoTuneVisual(this,owner,AutoTuneVisual.View.Progress));
                Panel("30-second SP excitation",()=>owner.ExcitationPlan,"This is an injected reference signal, not direct actuator excitation. The cap is a limit, not the waveform's measured peak.");
                Panel("Live measurements",()=>owner.ExcitationLive,"Values are actual samples. After collection or cancellation they remain as the most recent sample, while the injected offset is restored.");
                Panel("Waveform shape",()=>"Prepare gain before collecting data.\nRecompute amplitude for the final PID.\n0.2 Hz square wave, fixed for 30 seconds.\nFresh independent carrier for IV.\n\nThe native setpoint adjustment slider follows\nthe injected offset during collection.","The existing P-based cap and observed-peak amplitude policy are retained. Preparation does not enter identification. No E-optimal band allocation or new RMS amplitude-limit formula is used.");
                Panel("Input limits",()=>"The controller's output limits remain active.\nIdentification uses the measured post-limit u.\n\nGentle / Balanced / Fast changes the evaluation\nreference speed, not this collection waveform.","Clipping is represented in PID evaluation. Actuator allocation, coordinate coupling and unobserved dynamics can still cause model error.");
            }
            else
            {
                Panel("1. Collect",()=>"Start with a working PID.\nChoose Gentle, Balanced or Fast.\nPress Start 30 s test and keep this editor open.\nYou may read Data or Excitation while it runs.","Cancel restores the SP offset; prepared P is retained. Use Restore previous PID to undo it. Changing controller settings during collection cancels that run.");
                Panel("2. Review and repeat",()=>"Review the recommendation before Apply.\nRestore also works after preparation changes P.\n\nRun another test to add data, even after success.\nDifferent PID settings are retained per record.","Incomplete runs are not saved. Completed runs remain if calculation is cancelled while this window stays open.");
                Panel("3. Understand the result",()=>"Recommendation ready: a candidate improves\nthe model's predicted cost by at least 5%.\n\nNo improvement: no sufficient predicted gain.\nMore data needed: identification or response\nprediction could not support a recommendation.","A recommendation is a model prediction, not a guarantee of real vehicle improvement. Apply remains a manual action.");
                Panel("What the optimizer prefers",()=>"First: lower tracking error around the PID.\nWithin the tracking tolerance: smoother control.\nThen: lower control effort.\n\nTi Off disables integral action.\nTd Off disables derivative action.","Nearby PID settings are evaluated to reduce preference for isolated model optima. This is not a stability certificate.");
                Panel("Plant model constraints",()=>"No coefficient ridge penalty.\nStable real or complex poles.\nBounded normalized frequency response.\n\nSVD removes numerical-zero directions only.\nThese constraints do not certify the real plant.","Orders 1-3; training-only scaling. A 300 s maximum pole time constant and a normalized sampled response cap of 10 are model assumptions. Existing model-selection errors and record accumulation remain unchanged.");
                Panel("When to start a fresh history",()=>"Clear data after changing the vehicle or setup.\nAvoid mixing very different operating conditions.\n\nClosing this PID window also clears the history.\nIf repeated collection does not help, inspect\nexcitation, coupling and the operating condition.","Repeated data can improve identification, but cannot automatically fix an unsuitable model structure or missing signals.");
                Panel("Inspect bandwidth in Data",()=>"Green: injected SP. Blue: actual control u.\nOrange: measured output y.\n\nClick a saved run to inspect its spectrum.\nCheck the RMS table for weak response bands.\nCoherence is not a quality guarantee.","A new test shows the newest completed record by default. Switching tabs keeps the active collection running.");
            }
        }
    }
}
