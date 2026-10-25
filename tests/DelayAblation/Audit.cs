using System;using System.IO;using System.Linq;using System.Text.Json;using System.Reflection;using System.Threading;using System.Collections.Generic;using PIDSupporter;
namespace PIDSupporter {internal class AutoTuneSpectrum{} internal static class AutoTuneDiagnostics{internal static Action<string,object>? Sink;internal static void Event(string k,object v)=>Sink?.Invoke(k,v);}}
internal static class Audit {
 internal static int Delay=1;internal static int[] Orders={1,2,3};internal static NativeLinear.Selection? Override;internal static NativeLinear[]? Ensemble;internal static object[]? Probes;internal static double[][] ProbePids=Array.Empty<double[]>();
 internal static NativeLinear MakeModel(int n,double[] c,double um,double ym,double us,double ys){int o=n+Delay-1;var expanded=new double[2*o+1];Array.Copy(c,expanded,n);Array.Copy(c,n,expanded,o+Delay-1,n);expanded[2*o]=c[2*n];return new NativeLinear{Order=o,Coefficients=expanded,Um=um,Ym=ym,Us=us,Ys=ys};}
 static JsonSerializerOptions Opt=new(){NumberHandling=System.Text.Json.Serialization.JsonNumberHandling.AllowNamedFloatingPointLiterals};
 static TuneRecord Read(string file){var j=JsonDocument.Parse(File.ReadAllText(file)).RootElement;var r=new TuneRecord();foreach(var f in typeof(TuneRecord).GetFields(BindingFlags.Instance|BindingFlags.NonPublic))if(j.TryGetProperty(f.Name,out var v))f.SetValue(r,JsonSerializer.Deserialize(v.GetRawText(),f.FieldType));r.Speed=.5;return r;}
 static object Serialize(NativeLinear m)=>new{expandedOrder=m.Order,c=m.Coefficients,um=m.Um,ym=m.Ym,us=m.Us,ys=m.Ys,gain=m.DcGain()};
 static NativeLinear Load(JsonElement j)=>new NativeLinear{Order=j.GetProperty("expandedOrder").GetInt32(),Coefficients=j.GetProperty("c").EnumerateArray().Select(x=>x.GetDouble()).ToArray(),Um=j.GetProperty("um").GetDouble(),Ym=j.GetProperty("ym").GetDouble(),Us=j.GetProperty("us").GetDouble(),Ys=j.GetProperty("ys").GetDouble()};
 static void Main(string[] args){string root=args[0];NativeLinear.ResponseLimit=double.PositiveInfinity;
 if(args.Length>1 && args[1]=="search"){
  using var writer=new StreamWriter(Path.Combine(root,"search.jsonl"));
  foreach(var job in JsonDocument.Parse(File.ReadAllText(Path.Combine(root,"jobs.json"))).RootElement.EnumerateArray()){
   int record=job.GetProperty("record").GetInt32();var r=Read(Path.Combine(root,$"record{record}.json"));Ensemble=job.GetProperty("models").EnumerateArray().Select(Load).ToArray();Override=new NativeLinear.Selection{Model=Ensemble[0],Score=job.GetProperty("score").GetDouble()};object? search=null;AutoTuneDiagnostics.Sink=(k,v)=>{if(k=="pidSearch")search=v;};
   var result=NativeAutoTune.Run(r,CancellationToken.None,_=>{});writer.WriteLine(JsonSerializer.Serialize(new{record,stage=job.GetProperty("stage").GetString(),models=Ensemble.Length,result,search},Opt));writer.Flush();Console.WriteLine(JsonSerializer.Serialize(new{record,stage=job.GetProperty("stage").GetString(),result},Opt));
  }return;
 }
 using var output=new StreamWriter(Path.Combine(root,"fits.jsonl"));
 for(int i=0;i<4;i++){int first=i<2?0:2;var records=Enumerable.Range(first,i-first+1).Select(k=>Read(Path.Combine(root,$"record{k}.json"))).ToArray();
 for(Delay=1;Delay<=6;Delay++)for(int order=1;order<=3;order++){
  Orders=new[]{order};try{var selected=NativeLinear.SelectAccumulated(records,new WorkBudget(CancellationToken.None),_=>{});output.WriteLine(JsonSerializer.Serialize(new{record=i,delay=Delay,order,model=Serialize(selected.Model),score=selected.Score,final=new[]{selected.FinalFree,selected.FinalY,selected.FinalU}},Opt));}
  catch(Exception ex){output.WriteLine(JsonSerializer.Serialize(new{record=i,delay=Delay,order,failed=ex.Message}));}output.Flush();
 }Console.WriteLine($"Fitted record {i}");}
 }
}
