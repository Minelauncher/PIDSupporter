using System;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Threading;
using PIDSupporter;
internal static class DusterLinear
{
    static double[] A(JsonElement e, string key) => e.GetProperty(key).EnumerateArray().Select(x => x.GetDouble()).ToArray();
    internal static void Run(bool joint = false)
    {
        string folder = "tests/output/duster-hierarchical-20261020/" + (joint ? "joint27" : "axis7");
        Directory.CreateDirectory(folder);
        var record = File.ReadLines("tests/output/duster-diagnostic/source.jsonl").Select(l => JsonDocument.Parse(l).RootElement.Clone()).First(e => e.GetProperty("kind").GetString() == "record").GetProperty("value");
        var r = new TuneRecord { U=A(record,"U"), Y=A(record,"Y"), T=A(record,"T"), Pid=A(record,"Pid"), Lower=record.GetProperty("Lower").GetDouble(), Upper=record.GetProperty("Upper").GetDouble(), Amplitude=record.GetProperty("Amplitude").GetDouble(), Speed=.5 };
        r.R=A(JsonDocument.Parse(File.ReadAllText("tests/output/duster-diagnostic/reference.json")).RootElement,"wave");
        r.Integrals=new double[r.U.Length];
        if (r.Pid[1] < 250) throw new Exception("Old-record reconstruction only supports integral disabled");
        double offset=0;
        for(int i=160;i<600;i++) { double e=r.R[i]-r.Y[i], ep=r.R[i-1]-r.Y[i-1]; offset+=(r.U[i]/r.Pid[0]-e-r.Pid[2]*(e-ep)/(r.T[i]-r.T[i-1]))/440; }
        for(int i=0;i<r.R.Length;i++) r.R[i]+=offset;
        r.InitialIntegral=0; r.InitialError=r.R[r.R.Length-1]-r.Y[r.Y.Length-1];
        double replay=0;
        for(int i=160;i<r.U.Length;i++) { double integral=0; double u=NativeAutoTune.PidStep(r.Pid,r.R[i]-r.Y[i],r.R[i-1]-r.Y[i-1],ref integral,r.T[i]-r.T[i-1],r.Lower,r.Upper,false); replay+=Math.Pow(u-r.U[i],2)/(r.U.Length-160); }
        replay=Math.Sqrt(replay); if(replay>1e-4)throw new Exception("SP reconstruction parity failure");
        using var log=new StreamWriter(folder+"/diagnostics.jsonl");
        AutoTuneDiagnostics.Sink=(kind,value)=>{log.WriteLine(JsonSerializer.Serialize(new {kind,value})); if(kind=="linearSelected" || kind=="pidSearch")Console.WriteLine(JsonSerializer.Serialize(new {kind,value}));};
        var result=NativeAutoTune.Run(r,CancellationToken.None,Console.WriteLine,joint);
        var output=new { result, originalPid=r.Pid, spOffset=offset, recordedPidReplayRmse=replay, note="Legacy SP reconstructed with training-only offset; integral disabled. Same production model and cost for grid, nominal DE and neighborhood DE; sequential timings share evaluation cache." };
        File.WriteAllText(folder+"/result.json",JsonSerializer.Serialize(output,new JsonSerializerOptions{WriteIndented=true}));
        Console.WriteLine(JsonSerializer.Serialize(output)); AutoTuneDiagnostics.Sink=null;
    }
}
