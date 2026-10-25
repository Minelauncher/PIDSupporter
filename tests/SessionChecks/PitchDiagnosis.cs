using System;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Threading;
using System.Reflection;
using PIDSupporter;
internal static class PitchDiagnosis
{
 static double[] A(JsonElement e,string k)=>e.GetProperty(k).EnumerateArray().Select(v=>v.GetDouble()).ToArray();
 internal static void Run()
 {
  const string folder="tests/output/pitch-diagnosis-20261020";
  Directory.CreateDirectory(folder);
  var runs=JsonDocument.Parse(File.ReadAllText("tests/output/air-context-20261020/runs.json")).RootElement;
  var errors=typeof(NativeLinear).GetMethod("Errors",BindingFlags.Instance|BindingFlags.NonPublic)!;
  foreach(var run in runs.EnumerateArray().Where(x=>x.GetProperty("index").GetInt32()>=3)){
   var d=run.GetProperty("record");var start=run.GetProperty("start");int index=run.GetProperty("index").GetInt32();
   var r=new TuneRecord{U=A(d,"U"),Y=A(d,"Y"),R=A(d,"R"),T=A(d,"T"),Pid=A(d,"Pid"),Integrals=A(d,"Integrals"),InjectedSP=A(d,"InjectedSP"),BaseSP=A(d,"BaseSP"),Speed=start.GetProperty("speed").GetDouble(),Amplitude=d.GetProperty("Amplitude").GetDouble(),Lower=d.GetProperty("Lower").GetDouble(),Upper=d.GetProperty("Upper").GetDouble(),InitialIntegral=d.GetProperty("InitialIntegral").GetDouble(),InitialError=d.GetProperty("InitialError").GetDouble()};
   var budget=new WorkBudget(CancellationToken.None);double dt=NativeAutoTune.Validate(r);int n=r.U.Length;
   EnsembleLinear.Models.Clear();EnsembleLinear.Select(r,dt,budget,_=>{});
   var rows=EnsembleLinear.Models.Select(m=>new{m.Order,m.Ridge,m.Score,coefficients=m.Model.Coefficients,m.Model.Um,m.Model.Us,m.Model.Ym,m.Model.Ys,
    validation=(double[])errors.Invoke(m.Model,new object[]{r,dt,n/2,n*2/3,budget})!,
    final=(double[])errors.Invoke(m.Model,new object[]{r,dt,n*2/3,n,budget})!}).ToArray();
   File.WriteAllText(folder+$"/models-{index}.json",JsonSerializer.Serialize(rows,new JsonSerializerOptions{NumberHandling=System.Text.Json.Serialization.JsonNumberHandling.AllowNamedFloatingPointLiterals}));
   Console.WriteLine($"Pitch {index}: {rows.Length} models, best {rows.Min(x=>x.Score)}");
  }
 }
}
