using System;using System.IO;using System.Linq;using System.Text.Json;using System.Collections.Generic;using System.Reflection;using System.Threading;using PIDSupporter;using BrilliantSkies.Ai.Control.Pids;
partial class Run {
 static readonly JsonSerializerOptions Opt=new(){IncludeFields=true,NumberHandling=System.Text.Json.Serialization.JsonNumberHandling.AllowNamedFloatingPointLiterals};
 static object Pack(TuneRecord r)=>new{r.R,r.U,r.Y,r.T,r.InjectedSP,r.InstrumentSP,r.BaseSP,r.Integrals,r.Pid,r.Duration,r.Amplitude,r.Speed,r.Lower,r.Upper,r.InitialIntegral,r.InitialError,r.ExcitationSeed};
 static void Set(object o,string field,object value)=>o.GetType().GetField(field,BindingFlags.NonPublic|BindingFlags.Instance)!.SetValue(o,value);
 sealed class State {internal Plant Plant;internal VariableControllerMaster Ctrl=new();internal double Integral,Prev,U;internal float PreviousTime;internal int Steps;internal State(JsonElement c){Plant=new Plant(c);Ctrl.SetPointMultiple.Us=1;Ctrl.SetPointAdjust.Us=0;Ctrl.Pid.kP.Us=.4f;Ctrl.Pid.kI.Us=2.5f;Ctrl.Pid.kD.Us=.2f;}}
 static TuneRecord Collect(State s,JsonElement c,double duration,int repeat,TuneRecord? previous){
  AutoTuneSession.ResearchDuration=duration;AutoTuneSession.Captured=null;UnityEngine.Time.realtimeSinceStartup=(float)(s.Steps*.05);
  var session=new AutoTuneSession(s.Ctrl,.5f);Set(session,"ExcitationSeed",c.GetProperty("seed").GetInt32()+repeat*1009);
  if(previous!=null){Set(session,"gainPassed",true);typeof(AutoTuneSession).GetProperty("CollectionAmplitude",BindingFlags.NonPublic|BindingFlags.Instance)!.SetValue(session,previous.Amplitude);}
  int ticks=0;
  while(session.Collecting){
   float time=(float)(s.Steps*.05);UnityEngine.Time.realtimeSinceStartup=time;session.Heartbeat=time;
   if(++ticks>10000)throw new Exception("Preparation/collection timeout");
   double yy=s.Plant.Step(s.U);if(!double.IsFinite(yy)||Math.Abs(yy)>1e8)throw new Exception("Collection diverged");
   AutoTuneMeasurementPatch.Prefix(s.Ctrl,time,out var hook);float y=(float)yy,target=(float)(c.GetProperty("baseSP").GetDouble()+s.Ctrl.SetPointAdjust.Us),error=target-y;
   s.U=NativeAutoTune.PidStep(AutoTuneSession.ReadPid(s.Ctrl),error,s.Prev,ref s.Integral,time-s.PreviousTime,-1,1,s.Steps==0);s.Prev=error;s.PreviousTime=time;s.Ctrl.Pid.TestError=error;s.Ctrl.Pid.TestIntegral=(float)s.Integral;
   AutoTuneMeasurementPatch.Postfix(s.Ctrl,y,(float)s.U,hook);s.Steps++;
   if(session.Cancelled)throw new Exception(session.Message);
  }
  AutoTuneSession.Active=null;return AutoTuneSession.Captured??throw new Exception("Missing record");
 }
 static void Main(string[] args){
  string root=args[0];int shard=int.Parse(args[1]),shards=int.Parse(args[2]);var cases=JsonDocument.Parse(File.ReadAllText(Path.Combine(root,"cases-evaluation-only.json"))).RootElement;
  Directory.CreateDirectory(Path.Combine(root,"runs"));
  foreach(var c in cases.EnumerateArray().Where((_,i)=>i%shards==shard)){
   string id=c.GetProperty("id").GetString()!;
   foreach(int duration in new[]{30,60,120}){
    var state=new State(c);var history=new List<TuneRecord>();
    int repeats=duration==30?4:1;
    for(int repeat=0;repeat<repeats;repeat++){
     AutoTuneDiagnostics.Sink=null;var r=Collect(state,c,duration,repeat,history.LastOrDefault());history.Add(r);
     if(duration==30&&repeat==2)continue;
     string arm=duration==30?(repeat==0?"single30":"repeat"+(repeat+1)):"continuous"+duration;string path=Path.Combine(root,"runs",id+"-"+arm+".json");
     if(File.Exists(path))continue;
     object? model=null,search=null;var events=new List<object>();AutoTuneDiagnostics.Sink=(k,v)=>{if(k=="linearAccumulatedSelected")model=v;if(k=="pidSearch")search=v;if(k=="empiricalFrequencyChecks"||k=="empiricalFrequencyBounds")events.Add(new{kind=k,value=v});};
     var watch=System.Diagnostics.Stopwatch.StartNew();TuneResult result;
     try {result=NativeAutoTune.Run(r,CancellationToken.None,_=>{},records:history);}
     catch(Exception ex){result=new TuneResult{accepted=false,reason="calculation_failed",message=ex.Message,pid=r.Pid};}
     var endpoint=new{state.Plant.X,state.Plant.Q,state.Plant.Cross,state.Plant.Time,lastU=state.U,integral=state.Integral,error=state.Prev};
     var row=new{id,arm,record=Pack(r),records=history.Select(Pack).ToArray(),workingPid=r.Pid,result,model,search,events,endpoint,amplitude=r.Amplitude,timedout=false,message=result.message,preparationAttempts=0,elapsed=state.Steps*.05,seconds=watch.Elapsed.TotalSeconds};
     File.WriteAllText(path,JsonSerializer.Serialize(row,Opt));Console.WriteLine($"{id} {arm} {result.reason} [{string.Join(",",result.pid??r.Pid)}] {watch.Elapsed.TotalSeconds:F2}s");
    }
   }
  }
 }
}
