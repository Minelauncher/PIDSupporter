using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using System.Text.Json;
using System.Threading;
using PIDSupporter;
internal static class LowOrderRegression
{
 static double[] A(JsonElement e,string k)=>e.GetProperty(k).EnumerateArray().Select(v=>v.GetDouble()).ToArray();
 internal static void Run()
 {
  string folder="tests/output/low-order-production-20261020"; Directory.CreateDirectory(folder);
  string source="tests/output/game-review-20261020/source.jsonl";
  File.Copy(source,folder+"/source.jsonl",true);
  var output=new List<object>(); JsonElement start=default; int index=0;
  foreach(var line in File.ReadLines(source))
  {
   var e=JsonDocument.Parse(line).RootElement; string? kind=e.GetProperty("kind").GetString();
   if(kind=="start") start=e.GetProperty("value").Clone();
   if(kind!="record") continue;
   var d=e.GetProperty("value"); index++;
   if(start.GetProperty("excitation").GetString()!="sp-multisine-taper-lowfloor-v2") throw new Exception("Unknown waveform");
   var r=new TuneRecord{U=A(d,"U"),Y=A(d,"Y"),R=A(d,"R"),T=A(d,"T"),Pid=A(d,"Pid"),Integrals=A(d,"Integrals"),Speed=start.GetProperty("speed").GetDouble(),Amplitude=d.GetProperty("Amplitude").GetDouble(),Lower=d.GetProperty("Lower").GetDouble(),Upper=d.GetProperty("Upper").GetDouble(),InitialIntegral=d.GetProperty("InitialIntegral").GetDouble(),InitialError=d.GetProperty("InitialError").GetDouble()};
   var phase=new double[60];var weight=new double[60];var rng=new Random(826);
   for(int k=0;k<60;k++){phase[k]=rng.NextDouble()*2*Math.PI;double omega=2*Math.PI*(k+1)/30, hz=(k+1)/30.0, integral=r.Pid[1]>=250?0:1/(omega*Math.Max(.01,r.Pid[1]));weight[k]=Math.Max(.5*Math.Max(0,Math.Min(1,(.4-hz)/.2)),1/Math.Sqrt(1+Math.Pow(omega*r.Pid[2]-integral,2)));}
   var injected=new double[r.T.Length];double center=start.GetProperty("waveCenter").GetDouble();
   for(int i=0;i<injected.Length;i++){double t=i==0?0:r.T[i-1], raw=0,edge=Math.Min(t,30-t),env=edge>=1?1:.5*(1-Math.Cos(Math.PI*edge));for(int k=0;k<60;k++)raw+=weight[k]*Math.Sin(2*Math.PI*(k+1)*t/30+phase[k]);injected[i]=r.Amplitude*env*Math.Max(-1,Math.Min(1,(raw-center)/20));}
   r.BaseSP=r.R.Zip(injected,(a,b)=>a-b).ToArray();
   File.WriteAllText(folder+$"/signals-{index}.json",JsonSerializer.Serialize(new {r.R,r.U,r.Y,r.T,injected}));
   r.InjectedSP=injected;
   foreach(bool low in new[]{true}) foreach(bool recent in new[]{true})
   {

    object? selected=null;object? search=null;object? state=null;
    AutoTuneDiagnostics.Sink=(k,v)=>{if(k=="linearSelected")selected=v;if(k=="pidSearch")search=v;if(k=="startState")state=v;};
    try{var result=NativeAutoTune.Run(r,CancellationToken.None,_=>{});var expected=JsonDocument.Parse(File.ReadAllText("tests/output/state-order-20261020/comparison.json")).RootElement.EnumerateArray().First(v=>v.GetProperty("index").GetInt32()==index && v.GetProperty("low").GetBoolean() && v.GetProperty("recent").GetBoolean()).GetProperty("result");
    if(!result.pid!.SequenceEqual(A(expected,"pid")) || result.accepted!=expected.GetProperty("accepted").GetBoolean())throw new Exception("Research result mismatch");
    var row=new{index,low,recent,pid=r.Pid,selected,result,search,state};output.Add(row);Console.WriteLine(JsonSerializer.Serialize(row));}
    catch(Exception ex){throw new Exception($"Production regression run {index}",ex);}
   }
  }
  AutoTuneDiagnostics.Sink=null;File.WriteAllText(folder+"/comparison.json",JsonSerializer.Serialize(output,new JsonSerializerOptions{WriteIndented=true}));
 }
}
