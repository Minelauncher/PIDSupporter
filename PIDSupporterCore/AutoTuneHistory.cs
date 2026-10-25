using System;
using System.Collections.Generic;
using System.Text;
namespace PIDSupporter
{
    // Owned by one AutoTuneTab/window, never keyed globally by axis/name or vehicle ID.
    internal sealed class AutoTuneHistory
    {
        internal const int Capacity = 5;
        private readonly List<TuneRecord> records = new List<TuneRecord>();
        private readonly object gate = new object();
        private bool closed;
        internal TuneRecord[] DesignRecords(double lower,double upper,double spMultiple,double sampleTime)
        { lock(gate)return !closed && records.Count>0 && records[0].Lower==lower && records[0].Upper==upper && multiple==spMultiple && Math.Abs(dt-sampleTime)<=dt*.01 ? records.ToArray() : Array.Empty<TuneRecord>(); }
        private double multiple, dt;
        internal int Count { get { lock (gate) return records.Count; } }
        internal AutoTuneSpectrum? LatestSpectrum { get { lock(gate)return records.Count==0?null:records[records.Count-1].Spectrum; } }
        internal AutoTuneSpectrum? SpectrumAt(int index) { lock(gate)return index<0?LatestSpectrum:index<records.Count?records[index].Spectrum:null; }
        internal string Cell(int row,int column)
        {
            lock(gate)
            {
                if(row<0||row>=records.Count)return "-";var r=records[row];
                switch(column){case 0:return (row+1).ToString();case 1:return r.Pid[0].ToString("0.000");case 2:return r.Pid[1]>=250?"Off":r.Pid[1].ToString("0.0");case 3:return r.Pid[2]==0?"Off":r.Pid[2].ToString("0.0");default:return r.U.Length.ToString();}
            }
        }
        internal string Describe()
        {
            lock(gate)
            {
                if(records.Count==0)return "No completed records yet.\nStart a 30 s test to add the first record.";
                var text=new StringBuilder();
                for(int i=0;i<records.Count;i++)
                {
                    var r=records[i];
                    if(i>0)text.Append("\n\n");
                    text.Append($"Record {i+1}{(i==records.Count-1?" - newest":"")} | {r.Duration:0} s fixed PID | {r.U.Length} samples\n");
                    text.Append($"P {r.Pid[0]:0.000} | Ti {(r.Pid[1]>=250?"Off":r.Pid[1].ToString("0.0")+" s")} | Td {(r.Pid[2]==0?"Off":r.Pid[2].ToString("0.0")+" s")}\n");
                    text.Append($"SP offset cap +/-{r.Amplitude:0.000} | u limits [{r.Lower:0.00}, {r.Upper:0.00}]");
                }
                return text.ToString();
            }
        }
        internal void Clear() { lock (gate) { records.Clear(); } }
        internal void Close() { lock (gate) { closed = true; records.Clear(); } }
        internal TuneRecord[] Add(TuneRecord record, double setpointMultiple, System.Threading.CancellationToken token=default)
        {
            token.ThrowIfCancellationRequested();
            lock(gate)if(closed)throw new OperationCanceledException("PID window closed");
            double sampleTime = NativeAutoTune.Validate(record);
            // Runs on the acquisition worker, not the GUI/render thread. Cache once per record.
            if(record.Spectrum==null)record.Spectrum=AutoTuneSpectrum.Analyze(record);
            lock (gate)
            {
                token.ThrowIfCancellationRequested();
                if (closed) throw new OperationCanceledException("PID window closed; history discarded.");
                if (records.Count > 0 && (Math.Abs(sampleTime-dt)>dt*.01 || multiple!=setpointMultiple
                    || records[0].Lower!=record.Lower || records[0].Upper!=record.Upper)) records.Clear();
                multiple=setpointMultiple;dt=sampleTime;
                // PID changes are intentionally allowed; each record retains its own controller.
                records.Add(record);if(records.Count>Capacity)records.RemoveAt(0);
                return records.ToArray();
            }
        }
    }
}
