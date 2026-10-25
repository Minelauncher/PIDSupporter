using System;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Collections.Generic;
using System.Text.Json;
using System.Threading;
using PIDSupporter;
internal static class RobustReleaseChecks {
 static TuneRecord Read(string file){var j=JsonDocument.Parse(File.ReadAllText(file)).RootElement;var r=new TuneRecord();foreach(var f in typeof(TuneRecord).GetFields(BindingFlags.Instance|BindingFlags.NonPublic))if(j.TryGetProperty(f.Name,out var v))f.SetValue(r,JsonSerializer.Deserialize(v.GetRawText(),f.FieldType));r.Speed=.5;return r;}
 static void Require(bool v,string why){if(!v)throw new Exception(why);}
 internal static void Run(string source,string output){
  var denied=PidSearch.Run(new[]{.1,2.5,.4},p=>1,new WorkBudget(CancellationToken.None),_=>{},true,null,p=>false);
  Require(!denied.CandidatePassesConstraint && !denied.BaselinePassesConstraint && denied.RawBaselineRobust==1,"Constraint failure must not destroy raw baseline cost");
  var jobs=JsonDocument.Parse(File.ReadAllText(Path.Combine(source,"jobs.json"))).RootElement;
  var expected=JsonDocument.Parse(File.ReadAllText(Path.Combine(Path.GetDirectoryName(source)!,"robust-disk-20261025","bounds.json"))).RootElement;
  var rows=new List<object>();
  for(int i=0;i<4;i++){
   var records=Enumerable.Range(i<2?0:2,i-(i<2?0:2)+1).Select(k=>Read(Path.Combine(source,$"record{k}.json"))).ToArray();
   var j=jobs.EnumerateArray().First(x=>x.GetProperty("record").GetInt32()==i&&x.GetProperty("stage").GetString()=="baseline").GetProperty("models")[0];
   var model=new NativeLinear{Order=j.GetProperty("expandedOrder").GetInt32(),Coefficients=j.GetProperty("c").EnumerateArray().Select(x=>x.GetDouble()).ToArray(),Um=j.GetProperty("um").GetDouble(),Ym=j.GetProperty("ym").GetDouble(),Us=j.GetProperty("us").GetDouble(),Ys=j.GetProperty("ys").GetDouble()};
   // The offline scipy study fixed fs=40. Game float timestamps have a slightly
   // different median interval; use exactly that reference grid only for parity.
   var parity=records.Select(r=>{var copy=new TuneRecord();foreach(var f in typeof(TuneRecord).GetFields(BindingFlags.Instance|BindingFlags.NonPublic))f.SetValue(copy,f.GetValue(r));copy.T=Enumerable.Range(0,r.T.Length).Select(k=>k*.025).ToArray();return copy;}).ToArray();
   var gate=new EmpiricalFrequencyGate(model,parity,new WorkBudget(CancellationToken.None));var want=expected[i].GetProperty("radius").EnumerateArray().Select(x=>x.GetDouble()).ToArray();
   for(int k=0;k<want.Length;k++)Require(Math.Abs(gate.Radii[k]-want[k])<1e-5*Math.Max(1e-10,want[k]),$"C# spectral bounds disagree record={i} bin={k} actual={gate.Radii[k]} expected={want[k]} dt={NativeAutoTune.Validate(records.Last())}");
   Require(gate.Stable(new[]{1.0,-.5})&&!gate.Stable(new[]{1.0,-1.01})&&!gate.Stable(new[]{1.0,-1.0}),"Schur stability boundary");
   object? search=null;object? bound=null;
   AutoTuneDiagnostics.Sink=(kind,value)=>{if(kind=="pidSearch")search=value;if(kind=="empiricalFrequencyBounds")bound=value;};
   var result=NativeAutoTune.Run(records.Last(),CancellationToken.None,_=>{},records:records);
   Require(search is PidSearch.Result,"Production search was not reached");var sr=(PidSearch.Result)search!;
   Require(sr.JointNeighborhood&&sr.CandidatePassesConstraint,"Candidate must pass joint constraints");
   var actualGate=new EmpiricalFrequencyGate(model,records,new WorkBudget(CancellationToken.None));
   Require(PidSearch.Neighborhood(sr.Pid,p=>actualGate.Accept(p)?0:1,true)==0,"Returned PID must pass all 27 production checks");
   Require(result.reason!="baseline_prediction_failed" && !result.needs_more_data,"Baseline constraint must not masquerade as failed simulation");
   if(i>=2)Require(sr.Pid[0]<.3,"Spirit aggressive P returned");
   rows.Add(new{record=i,result,search,bound});Console.WriteLine(JsonSerializer.Serialize(new{record=i,result,search}));
  }
  AutoTuneDiagnostics.Sink=null;File.WriteAllText(output,JsonSerializer.Serialize(rows,new JsonSerializerOptions{WriteIndented=true}));Console.WriteLine("Robust release checks passed");
 }
}
