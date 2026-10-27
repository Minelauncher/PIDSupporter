using System;using System.IO;using System.Linq;using System.Text.Json;using System.Reflection;using System.Threading;using System.Collections.Generic;using PIDSupporter;
internal static class Audit {
 internal static NativeLinear.Selection Override=null!;internal static bool UseGate=true;internal static double[] Radius=null!;
 static readonly JsonSerializerOptions Opt=new(){NumberHandling=System.Text.Json.Serialization.JsonNumberHandling.AllowNamedFloatingPointLiterals};
 static double[] Arr(JsonElement e)=>e.EnumerateArray().Select(x=>x.GetDouble()).ToArray();
 static TuneRecord Record(JsonElement j){var r=new TuneRecord();foreach(var f in typeof(TuneRecord).GetFields(BindingFlags.Instance|BindingFlags.NonPublic))if(j.TryGetProperty(f.Name,out var v))f.SetValue(r,JsonSerializer.Deserialize(v.GetRawText(),f.FieldType));r.Speed=.5;return r;}
 static void Main(string[] args){
  string root=args[0];int shard=int.Parse(args[1]),shards=int.Parse(args[2]);Directory.CreateDirectory(Path.Combine(root,"search"));
  var files=Directory.GetFiles(Path.Combine(root,"jobs"),"*.json").OrderBy(x=>x).Where((_,i)=>i%shards==shard);
  foreach(var path in files){var job=JsonDocument.Parse(File.ReadAllText(path)).RootElement;string id=job.GetProperty("id").GetString()!;bool game=job.GetProperty("kind").GetString()=="game";
   var r=Record(job.GetProperty("record"));var records=job.GetProperty("records").EnumerateArray().Select(Record).ToArray();var m=job.GetProperty("model");double D(string key)=>m.GetProperty(key).GetDouble();
   Override=new NativeLinear.Selection{Model=new NativeLinear{Order=m.GetProperty("order").GetInt32(),Coefficients=Arr(m.GetProperty("coefficients")),Um=D("Um"),Us=D("Us"),Ym=D("Ym"),Ys=D("Ys")},Score=job.GetProperty("score").GetDouble()};
   foreach(string method in args.Length>3?args[3].Split(','):game?new[]{"off","legacy","paired"}:new[]{"legacy","paired"}){
    if(method=="off"&&!game)continue;
    string dest=Path.Combine(root,"search",id+"-"+method+".json");if(File.Exists(dest))continue;
    UseGate=method!="off";Radius=Arr(job.GetProperty("bounds").GetProperty("radii").GetProperty(UseGate?method:"legacy"));object? search=null;AutoTuneDiagnostics.Sink=(k,v)=>{if(k=="pidSearch")search=v;};
    var result=NativeAutoTune.Run(r,CancellationToken.None,_=>{},records:records);var tests=new List<object>();
    if(game){var gate=new EmpiricalFrequencyGate(Override.Model,records,new WorkBudget(CancellationToken.None));foreach(var p in new[]{new[]{.995,.1,1.9},new[]{1.0,.1,0.0},new[]{.321,.6,.5},r.Pid})tests.Add(new{pid=p,accepted=gate.Accept(p)});}
    File.WriteAllText(dest,JsonSerializer.Serialize(new{id,method,result,search,probe=tests},Opt));Console.WriteLine(id+" "+method+" "+result.reason+" "+string.Join(",",result.pid??r.Pid));
   }
  }
 }
}
