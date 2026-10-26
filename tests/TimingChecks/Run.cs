using System;
using System.Linq;
using System.Collections.Generic;
using System.Reflection;
using PIDSupporter;
using BrilliantSkies.Ai.Control.Pids;

static class Run {
 static int checks;
 static void Check(bool ok,string message){if(!ok)throw new Exception(message);checks++;}
 static TuneRecord Record(double start,double step,double duration=30) {
  int n=(int)Math.Round(duration/step);
  return new TuneRecord { T=Enumerable.Range(1,n).Select(i=>(double)(float)(start+i*step)-(float)start).ToArray(),
   U=new double[n],Y=new double[n],R=new double[n],InjectedSP=new double[n],BaseSP=new double[n],InstrumentSP=new double[n],Integrals=new double[n],
   Pid=new[]{.4,2.5,.2},Lower=-1,Upper=1,Amplitude=.2,Speed=.5,Duration=duration };
 }
 static void Reject(TuneRecord r,string label){try{NativeAutoTune.Validate(r);}catch(ArgumentException){checks++;return;}throw new Exception("Accepted invalid timing: "+label);}
 static void Remove(TuneRecord r,int index,int count=1){r.T=r.T.Take(index).Concat(r.T.Skip(index+count)).ToArray();r.U=new double[r.T.Length];r.Y=new double[r.T.Length];}
 static T Field<T>(AutoTuneSession s,string name)=>(T)typeof(AutoTuneSession).GetField(name,BindingFlags.NonPublic|BindingFlags.Instance)!.GetValue(s)!;
 static void Main(){
  foreach(double duration in new[]{24.0,30.0})foreach(double step in new[]{.01,.02,.025,.05,.1})foreach(double start in new[]{0.0,2048.0,4096.0,8192.0}) {
   if(duration/step<300)continue;
   var r=Record(start,step,duration);double dt=NativeAutoTune.Validate(r);Check(Math.Abs(dt-step)<2e-6,"mean dt accuracy");
   if(duration==30 && step==.025 && start==2048){var gaps=r.T.Skip(1).Zip(r.T,(a,b)=>a-b).OrderBy(v=>v).ToArray();Check(Math.Abs(r.T.Length*gaps[gaps.Length/2]-30)>.1,"old failure reproduced");}
   var bad=Record(start,step,duration);Remove(bad,bad.T.Length/2);Reject(bad,"one missing interior sample");
   bad=Record(start,step,duration);Remove(bad,3,4);Reject(bad,"missing block");
   bad=Record(start,step,duration);bad.T[5]=bad.T[4];Reject(bad,"duplicate timestamp");
   bad=Record(start,step,duration);bad.T[5]=bad.T[4]-step;Reject(bad,"backwards timestamp");
   bad=Record(start,step,duration);bad.T[5]=double.NaN;Reject(bad,"non-finite timestamp");
   bad=Record(start,step,duration);Remove(bad,0,2);Reject(bad,"truncated start");
   bad=Record(start,step,duration);Remove(bad,bad.T.Length-2,2);Reject(bad,"truncated end");
  }
  var legacy=Record(0,.025);for(int i=0;i<legacy.T.Length;i++)legacy.T[i]=i*.025;NativeAutoTune.Validate(legacy);checks++;
  var zero=Record(0,.025);Array.Clear(zero.T,0,zero.T.Length);Reject(zero,"zero span");
  var shortRecord=Record(0,.025);shortRecord.Duration=29;Reject(shortRecord,"unsupported duration");
  // Exercise the actual collection lifecycle using float absolute game times.
  foreach(double start in new[]{0.0,2048.0,4096.0,8192.0})foreach(double step in new[]{.02,.025,.05}) {
   UnityEngine.Time.realtimeSinceStartup=0;var c=new VariableControllerMaster();c.Pid.kP.Us=.4f;c.Pid.kI.Us=250;c.Pid.kD.Us=0;
   var s=new AutoTuneSession(c,.5f);s.BeginSample((float)start);
   for(int k=1;k<5000 && s.Collecting;k++){var wave=s.BeginSample((float)(start+k*step));if(wave.HasValue){s.SetExcitation(wave.Value);s.EndSample(.1,.1*Math.Sin(k*step));}}
   Check(!s.Cancelled && s.CollectionStarted && !s.Collecting,"session finished normally");
   var r=Record(0,step);r.T=Field<List<double>>(s,"t").ToArray();r.U=Field<List<double>>(s,"u").ToArray();r.Y=Field<List<double>>(s,"y").ToArray();
   Check(Math.Abs(NativeAutoTune.Validate(r)-step)<2e-6,"actual session timing valid");s.Cancel();
  }
  Console.WriteLine($"PASS {checks} timing assertions: float-clock normal records, actual collection, missing samples, duplicates, truncation and invalid times.");
 }
}
