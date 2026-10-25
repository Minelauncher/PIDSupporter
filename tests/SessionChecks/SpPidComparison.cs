using System;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Threading;
using System.Collections.Generic;
using PIDSupporter;
internal static class SpPidComparison
{
    internal static NativeLinear.Selection? Selection;
    static double[] A(JsonElement e,string key)=>e.GetProperty(key).EnumerateArray().Select(v=>v.GetDouble()).ToArray();
    internal static void Run(bool indirect = false, bool regularized = false)
    {
        string src=regularized ? "research_lti/results/regularized_indirect_20261020" : indirect ? "research_lti/results/indirect_comparison_20261020" : "research_lti/results/negative_gain_diagnosis_20261020";
        string output=regularized ? "tests/output/regularized-indirect-pid-20261020" : indirect ? "tests/output/indirect-pid-comparison-20261020" : "tests/output/sp-pid-comparison-20261020";
        var record=File.ReadLines(src+"/source.jsonl").Select(l=>JsonDocument.Parse(l).RootElement.Clone()).First(e=>e.GetProperty("kind").GetString()=="record").GetProperty("value");
        var r=new TuneRecord {U=A(record,"U"),Y=A(record,"Y"),R=A(record,"R"),T=A(record,"T"),Integrals=A(record,"Integrals"),Pid=A(record,"Pid"),Speed=.5,Amplitude=record.GetProperty("Amplitude").GetDouble(),Lower=record.GetProperty("Lower").GetDouble(),Upper=record.GetProperty("Upper").GetDouble(),InitialIntegral=record.GetProperty("InitialIntegral").GetDouble(),InitialError=record.GetProperty("InitialError").GetDouble()};
        int fit=r.U.Length/2;double um=r.U.Take(fit).Average(),ym=r.Y.Take(fit).Average(),us=Math.Sqrt(r.U.Take(fit).Average(v=>(v-um)*(v-um))),ys=Math.Sqrt(r.Y.Take(fit).Average(v=>(v-ym)*(v-ym)));
        Directory.CreateDirectory(output);
        // Unmodified production baseline, compared with generated evaluation code.
        var baseline=NativeAutoTune.Run(r,CancellationToken.None,_=>{});
        var models=new Dictionary<string,NativeLinear>();var pids=new Dictionary<string,double[]> { ["incumbent"]=r.Pid };
        var results=new List<object>();
        foreach(var c in JsonDocument.Parse(File.ReadAllText(src+(regularized?"/pid-models.json":"/results.json"))).RootElement.EnumerateArray())
        {
            string group=c.GetProperty("group").GetString()!;
            if(!indirect && group!="direct0" && group!="direct2" && group!="direct8" && !group.StartsWith("iv2_"))continue;
            var model=new NativeLinear {Order=c.GetProperty("order").GetInt32(),Coefficients=A(c,"coeff"),Um=um,Ym=ym,Us=us,Ys=ys};
            var final=A(c,"final");Selection=new NativeLinear.Selection {Model=model,Score=c.GetProperty("score").GetDouble(),FinalFree=final[0],FinalY=final[1],FinalU=final[2]};
            object? search=null;AutoTuneDiagnostics.Sink=(kind,value)=> {if(kind=="pidSearch")search=value;};
            var result=SpResearchAutoTune.Run(r,CancellationToken.None,_=>{});
            if(group=="direct0" && (result.accepted!=baseline.accepted || Math.Abs(result.predicted_ratio-baseline.predicted_ratio)>1e-7 || !result.pid!.SequenceEqual(baseline.pid!)))throw new Exception("Production baseline mismatch");
            results.Add(new{group,result,search});models[group]=model;pids[group]=result.pid!;
            Console.WriteLine(JsonSerializer.Serialize(new{group,result}));
        }
        Selection=null;AutoTuneDiagnostics.Sink=null;
        File.WriteAllText(output+"/results.json",JsonSerializer.Serialize(results,new JsonSerializerOptions{WriteIndented=true}));
        // Cross-model comparison is diagnostic only; it does not reselect any PID.
        var cross=new List<object>();
        foreach(var model in models)foreach(var pid in pids)
        {
            var m=SpResearchAutoTune.Evaluate(r,model.Value,pid.Value);
            double robust=PidSearch.Neighborhood(pid.Value,p=>SpResearchAutoTune.Evaluate(r,model.Value,p).Tracking,false);
            cross.Add(new{model=model.Key,candidate=pid.Key,pid=pid.Value,tracking=m.Tracking,robust,slew=m.Slew,effort=m.Effort});
        }
        File.WriteAllText(output+"/cross-model.json",JsonSerializer.Serialize(cross,new JsonSerializerOptions{WriteIndented=true}));
    }
}
