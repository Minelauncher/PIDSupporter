using System;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Threading;
using PIDSupporter;
internal static class BandwidthLogReview
{
 internal static void Full(string path,string output)
 {
  var rs=File.ReadLines(path).Select(l=>JsonDocument.Parse(l)).Where(d=>d.RootElement.GetProperty("kind").GetString()=="record").Select(d=>Read(d.RootElement.GetProperty("value"))).ToArray();
  var result=NativeAutoTune.Run(rs[rs.Length-1],CancellationToken.None,_=>{},records:rs);result.accumulated_records=rs.Length;
  File.WriteAllText(output,JsonSerializer.Serialize(result,new JsonSerializerOptions{WriteIndented=true}));Console.WriteLine(File.ReadAllText(output));
 }
 internal static void Run(string path,string output)
 {
  var records=File.ReadLines(path).Select(l=>JsonDocument.Parse(l)).Where(d=>d.RootElement.GetProperty("kind").GetString()=="record").Select(d=>Read(d.RootElement.GetProperty("value"))).ToArray();
  var rows=records.Select((r,i)=>{
   var s=NativeLinear.Select(r,NativeAutoTune.Validate(r),new WorkBudget(CancellationToken.None),_=>{});
   var j=i==0?s:NativeLinear.SelectAccumulated(records.Take(i+1).ToArray(),new WorkBudget(CancellationToken.None),_=>{});
   return new {run=i+1,single=new{order=s.Model.Order,gain=s.Model.DcGain(),selection=s.Score,free=s.FinalFree,y=s.FinalY,u=s.FinalU},joint=new{order=j.Model.Order,gain=j.Model.DcGain(),selection=j.Score,free=j.FinalFree,y=j.FinalY,u=j.FinalU}};
  }).ToArray();
  File.WriteAllText(output,JsonSerializer.Serialize(rows,new JsonSerializerOptions{WriteIndented=true}));
  Console.WriteLine(File.ReadAllText(output));
 }
 internal static TuneRecord Read(JsonElement v) {
  double[] A(string key)=>v.GetProperty(key).EnumerateArray().Select(x=>x.GetDouble()).ToArray();
  return new TuneRecord{U=A("U"),Y=A("Y"),R=A("R"),T=A("T"),InjectedSP=A("InjectedSP"),BaseSP=A("BaseSP"),Integrals=A("Integrals"),Pid=A("Pid"),Lower=v.GetProperty("Lower").GetDouble(),Upper=v.GetProperty("Upper").GetDouble(),Amplitude=v.GetProperty("Amplitude").GetDouble(),InitialIntegral=v.GetProperty("InitialIntegral").GetDouble(),InitialError=v.GetProperty("InitialError").GetDouble(),Speed=.5};
 }
}
