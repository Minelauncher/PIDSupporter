using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using System.Text.Json;
using System.Threading;
using PIDSupporter;
internal static class JointStudy {
 static double[] A(JsonElement e,string k)=>e.GetProperty(k).EnumerateArray().Select(v=>v.GetDouble()).ToArray();
 internal static void Run(){
 string folder="tests/output/joint-records-20261020";Directory.CreateDirectory(folder);
 var records=new List<TuneRecord>();JsonElement start=default;
 foreach(var line in File.ReadLines("tests/output/game-review-20261020/source.jsonl")){
 var e=JsonDocument.Parse(line).RootElement;var kind=e.GetProperty("kind").GetString();if(kind=="start")start=e.GetProperty("value").Clone();if(kind!="record")continue;var d=e.GetProperty("value");
 var injected=A(JsonDocument.Parse(File.ReadAllText($"tests/output/state-order-20261020/signals-{records.Count+1}.json")).RootElement,"injected");
 records.Add(new TuneRecord{U=A(d,"U"),Y=A(d,"Y"),R=A(d,"R"),T=A(d,"T"),Pid=A(d,"Pid"),Integrals=A(d,"Integrals"),InjectedSP=injected,BaseSP=A(d,"R").Zip(injected,(x,y)=>x-y).ToArray(),Speed=start.GetProperty("speed").GetDouble(),Amplitude=d.GetProperty("Amplitude").GetDouble(),Lower=d.GetProperty("Lower").GetDouble(),Upper=d.GetProperty("Upper").GetDouble(),InitialIntegral=d.GetProperty("InitialIntegral").GetDouble(),InitialError=d.GetProperty("InitialError").GetDouble()});
 }
 var output=new List<object>();
 for(int i=0;i<records.Count;i++) foreach(int window in new[]{1,3,5}){
 var r=records[i];JointLinear.Records=records.Skip(Math.Max(0,i-window+1)).Take(Math.Min(window,i+1)).ToArray();
 object? selected=null;object? search=null;AutoTuneDiagnostics.Sink=(k,v)=>{if(k=="linearSelected")selected=v;if(k=="pidSearch")search=v;};
 try{
 var result=JointTune.Run(r,CancellationToken.None,_=>{});
 var budget=new WorkBudget(CancellationToken.None);var model=JointLinear.Select(r,NativeAutoTune.Validate(r),budget,_=>{}).Model;
 double[]? nextErrors=i+1<records.Count?model.Forecast(records[i+1],NativeAutoTune.Validate(records[i+1]),budget):null;
 if(window==1){var expected=JsonDocument.Parse(File.ReadAllText("tests/output/low-order-production-20261020/comparison.json")).RootElement.EnumerateArray().First(v=>v.GetProperty("index").GetInt32()==i+1).GetProperty("result");if(!result.pid!.SequenceEqual(A(expected,"pid")))throw new Exception("Single-record regression mismatch");}
 var row=new{index=i+1,window,count=JointLinear.Records.Length,selected,result,search,nextErrors};output.Add(row);Console.WriteLine(JsonSerializer.Serialize(row,new JsonSerializerOptions{NumberHandling=System.Text.Json.Serialization.JsonNumberHandling.AllowNamedFloatingPointLiterals}));
 }catch(Exception ex){throw new Exception($"Run {i+1} window {window}",ex);}
 }
 AutoTuneDiagnostics.Sink=null;File.WriteAllText(folder+"/comparison.json",JsonSerializer.Serialize(output,new JsonSerializerOptions{WriteIndented=true,NumberHandling=System.Text.Json.Serialization.JsonNumberHandling.AllowNamedFloatingPointLiterals}));
 }
}
