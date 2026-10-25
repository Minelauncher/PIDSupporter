using System;using System.IO;using System.Linq;using System.Text.Json;using System.Reflection;using System.Threading;using System.Collections.Generic;using PIDSupporter;
namespace PIDSupporter {internal class AutoTuneSpectrum{} internal static class AutoTuneDiagnostics{internal static Action<string,object>? Sink;internal static void Event(string k,object v)=>Sink?.Invoke(k,v);}}
internal static class Audit {
 internal static int Delay=1;internal static int[] Orders={1,2,3};internal static NativeLinear.Selection? Override;internal static object[]? Probes;internal static double[][] ProbePids=Array.Empty<double[]>();
 internal static NativeLinear MakeModel(int n,double[] c,double um,double ym,double us,double ys){int o=n+Delay-1;var expanded=new double[2*o+1];Array.Copy(c,expanded,n);Array.Copy(c,n,expanded,o+Delay-1,n);expanded[2*o]=c[2*n];return new NativeLinear{Order=o,Coefficients=expanded,Um=um,Ym=ym,Us=us,Ys=ys};}
 static JsonSerializerOptions Opt=new(){NumberHandling=System.Text.Json.Serialization.JsonNumberHandling.AllowNamedFloatingPointLiterals};
 static TuneRecord Read(string file){var j=JsonDocument.Parse(File.ReadAllText(file)).RootElement;var r=new TuneRecord();foreach(var f in typeof(TuneRecord).GetFields(BindingFlags.Instance|BindingFlags.NonPublic))if(j.TryGetProperty(f.Name,out var v))f.SetValue(r,JsonSerializer.Deserialize(v.GetRawText(),f.FieldType));r.Speed=.5;return r;}
 static void Main(string[] args){string root=args[0];var manifest=JsonDocument.Parse(File.ReadAllText(Path.Combine(root,"manifest.json"))).RootElement;
 using var output=new StreamWriter(Path.Combine(root,"fits.jsonl"));
 foreach(var meta in manifest.EnumerateArray())foreach(bool cumulative in new[]{false,true}){
 var names=meta.GetProperty("pool").EnumerateArray().Select(v=>v.GetString()!).ToArray();if(cumulative&&names.Length<2)continue;var records=(cumulative?names:new[]{meta.GetProperty("file").GetString()!}).Select(f=>Read(Path.Combine(root,"records",f))).ToArray();var r=records.Last();Delay=meta.GetProperty("delay").GetInt32();string id=meta.GetProperty("id").GetString()!;
 foreach(bool cap in new[]{true,false}){NativeLinear.ResponseLimit=cap?10:double.PositiveInfinity;Override=null;var candidateLogs=new List<object>();object? search=null;
 AutoTuneDiagnostics.Sink=(k,v)=>{if(k=="linearConstrainedCandidate")candidateLogs.Add(v);if(k=="pidSearch")search=v;};
 try{var selected=NativeLinear.SelectAccumulated(records,new WorkBudget(CancellationToken.None),_=>{});Override=selected;var m=selected.Model;TuneResult? result=null;if(meta.GetProperty("group").GetString()!="synthetic")result=NativeAutoTune.Run(r,CancellationToken.None,_=>{});
 var row=new{id,cumulative,cap,delay=Delay,order=m.Order-Delay+1,expandedOrder=m.Order,c=m.Coefficients,um=m.Um,ym=m.Ym,us=m.Us,ys=m.Ys,gain=m.DcGain(),validation=selected.Score,selected.FinalFree,selected.FinalY,selected.FinalU,result,search,candidates=candidateLogs};output.WriteLine(JsonSerializer.Serialize(row,Opt));output.Flush();Console.WriteLine(JsonSerializer.Serialize(new{id,cumulative,cap,order=row.order,gain=row.gain,validation=row.validation,pid=result?.pid},Opt));}
 catch(Exception ex){output.WriteLine(JsonSerializer.Serialize(new{id,cumulative,cap,failed=ex.Message},Opt));output.Flush();Console.WriteLine(id+" FAILED "+ex.Message);}
 }
 }
 }
}
