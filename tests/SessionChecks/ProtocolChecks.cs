using System;
using System.Collections.Generic;
using System.Reflection;
using System.Threading.Tasks;
using PIDSupporter;
using BrilliantSkies.Ai.Control.Pids;
internal static class ProtocolChecks
{
 static void Check(bool ok,string name){if(!ok)throw new Exception(name);}
 static T Field<T>(AutoTuneSession s,string name)=>(T)typeof(AutoTuneSession).GetField(name,BindingFlags.NonPublic|BindingFlags.Instance)!.GetValue(s)!;
 static void Set(AutoTuneSession s,string name,object value)=>typeof(AutoTuneSession).GetField(name,BindingFlags.NonPublic|BindingFlags.Instance)!.SetValue(s,value);
 internal static void Run()
 {
  UnityEngine.Time.realtimeSinceStartup=0;
  var c=new VariableControllerMaster();c.Pid.kP.Us=.4f;c.Pid.kI.Us=250;c.Pid.kD.Us=0;
  var history=new AutoTuneHistory();var s=new AutoTuneSession(c,.5f,history);
  Check(s.Carrier(1)==0 && s.Wave(1)==0,"no injected excitation before gain confirmation");
  Check(s.BeginSample(0)==null,"prime timestamp");
  for(int i=1;i<=240;i++){
   var wave=s.BeginSample(i*.05);Check(wave.HasValue,"preparation alive");s.SetExcitation(wave!.Value);
   s.EndSample(i<=40?1:.1,.1*Math.Sin(i*.05));
   Check(Field<List<double>>(s,"u").Count==0,"preparation excluded from fit records");
  }
  Check(Math.Abs(c.Pid.kP.Us-.2)<1e-6,"gain adjusts before collection");
  Check(Math.Abs(s.Original[0]-.4)<1e-6 && Math.Abs(s.WorkingPid[0]-.2)<1e-6,"original and working PID separate");
  Check(Math.Abs(s.Amplitude-.2/s.WorkingPid[0])<1e-9,"cap rebased on prepared PID");
  Check(Math.Abs(s.CollectionAmplitude-.75*s.Amplitude)<1e-9,"actual amplitude rebased, not retained from original gain");

  var first=s.BeginSample(12.05);Check(s.CollectionStarted&&Math.Abs(first!.Value)<1e-12,"new fixed collection starts at zero after preparation");
  double frozenPhase=Field<double>(s,"squarePhase");
  Check(frozenPhase>=0&&frozenPhase<2*Math.PI,"fresh square phase");
  double amp=s.CollectionAmplitude;
  Check(Math.Abs(s.Carrier(13.2)-s.Carrier(18.2))<1e-10,"square repeats every5seconds");
  Check(amp<=s.Amplitude&&amp>0,"amplitude upper bound");
  s.SetExcitation(first!.Value);s.EndSample(.15,.2);
  var reference=Field<List<double>>(s,"references");var injected=Field<List<double>>(s,"injected");var baseline=Field<List<double>>(s,"baseReferences");
  Check(reference.Count==1 && Math.Abs(reference[0]-injected[0]-baseline[0])<1e-12,"actual SP separation");
  // Completion from preparation that arrives after freezing must never alter collection.

  for(int i=242;i<=840;i++){
   var wave=s.BeginSample(i*.05);Check(wave.HasValue,"continuous fixed collection");
   Check(Math.Abs(wave!.Value)<=s.Amplitude+1e-12,"SP cap");s.SetExcitation(wave.Value);s.EndSample(.15,.2*Math.Sin(i*.05));
   Check(c.Pid.kP.Us==s.WorkingPid[0]&&s.CollectionAmplitude==amp,"fixed PID and amplitude throughout collection");
   Check(Field<double>(s,"squarePhase")==frozenPhase,"carrier stays fixed");
   var iv=Field<List<double>>(s,"instruments");Check(Math.Abs(iv[iv.Count-1]*amp-wave.Value)<1e-12,"IV unscaled and independent of actual SP rounding");
  }
  Check(!s.Collecting&&Math.Abs(s.CollectionElapsed-30)<1e-6&&s.Elapsed>30,"preparation outside separate30s collection");
  Check(Field<List<double>>(s,"u").Count==600,"exact30seconds at20Hz");
  Check(Field<List<double>>(s,"t")[0]>.0 && Field<List<double>>(s,"t")[0]<.051,"collection timestamps restart after preparation");
  Check(c.SetPointAdjust.Us==3&&s.LiveOffset==0,"completion restores SP");s.Cancel();
  s=new AutoTuneSession(c,.5f);s.SetExcitation(.1);c.SetPointAdjust.Us=9;
  Check(!s.CheckAlive()&&c.SetPointAdjust.Us==9,"manual SP edit preserved");c.SetPointAdjust.Us=3;
  s=new AutoTuneSession(c,.5f);c.Pid.kP.Us=.3f;Check(!s.CheckAlive()&&c.Pid.kP.Us==.3f,"manual PID edit cancels without overwrite");
  s=new AutoTuneSession(c,.5f);s.SetExcitation(.1);UnityEngine.Time.realtimeSinceStartup=1;
  Check(!s.CheckAlive()&&s.LiveOffset==0&&c.SetPointAdjust.Us==3,"window expiry cancels and restores SP");
  s=new AutoTuneSession(c,.5f);s.BeginSample(0);s.BeginSample(1);Check(s.Cancelled,"timing interruption");
  s=new AutoTuneSession(c,.5f);bool refused=false;try{new AutoTuneSession(new VariableControllerMaster(),.5f);}catch(InvalidOperationException){refused=true;}Check(refused,"exclusive excitation");s.Cancel();
  s=new AutoTuneSession(c,.5f);s.BeginSample(0);
  for(int i=1;i<=720;i++){var wave=s.BeginSample(i*.05);Check(wave.HasValue,"unlimited preparation alive");s.SetExcitation(wave!.Value);s.EndSample(1,.2);}
  Check(!s.CollectionStarted&&s.Collecting&&s.PreparationAttempts==9&&Field<List<double>>(s,"u").Count==0,"failed gain checks repeat beyond30s without collection or attempt cap");s.Cancel();
  c.Pid.kP.Us=.4f;s=new AutoTuneSession(c,.5f);s.BeginSample(0);
  for(int i=1;i<=80;i++){s.SetExcitation(s.BeginSample(i*.05)!.Value);s.EndSample(i%2==0?.4:-.4,0);}
  Check(Math.Abs(s.WorkingPid[0]-.28)<1e-6&&!s.CollectionStarted,"oscillatory input lowers gain before collecting");s.Cancel();
  // Both gain directions and the I/D frequency weights use the working PID.
  c.Pid.kP.Us=.4f;s=new AutoTuneSession(c,.5f);
  s.WorkingPid[0]=.8;s.WorkingPid[1]=2;s.WorkingPid[2]=1;
  typeof(AutoTuneSession).GetMethod("RefreshExcitationBasis",BindingFlags.NonPublic|BindingFlags.Instance)!.Invoke(s,null);
  Check(Math.Abs(s.Amplitude-.25)<1e-12&&Math.Abs(s.CollectionAmplitude-.125)<1e-12,"increased gain lowers cap and amplitude");
  s.Cancel();
  // Game applies fake-SP override BEFORE adjustment and multiplier (current Ai.dll).
  c.FakeSetPointInUse.Us=true;c.FakeSetPoint.Us=5;c.Pid.TestError=0;
  s=new AutoTuneSession(c,.5f);s.BeginSample(0);Set(s,"gainPassed",true);

  for(int i=1;i<=12;i++){
   double wave=s.BeginSample(i*.05)!.Value;s.SetExcitation(wave);
   float effective=(c.FakeSetPoint.Us+c.SetPointAdjust.Us)*c.SetPointMultiple.Us;
   c.Pid.TestError=effective-1;s.EndSample(.1,1);
   int j=i-1;Check(Math.Abs(Field<List<double>>(s,"references")[j]-effective)<1e-6,"fake-SP effective R recorded");
   Check(Math.Abs(Field<List<double>>(s,"baseReferences")[j]-16)<2e-6,"fake base excludes injected SP");
   Check(Math.Abs(Field<List<double>>(s,"instruments")[j]*s.CollectionAmplitude-wave)<1e-12,"IV contains unscaled carrier only");
  }
  c.FakeSetPoint.Us=6;Check(!s.CheckAlive()&&c.FakeSetPoint.Us==6&&c.SetPointAdjust.Us==3,"fake value edit cancels without overwriting");
  s=new AutoTuneSession(c,.5f);c.FakeSetPointInUse.Us=false;Check(!s.CheckAlive(),"fake toggle cancels");c.Pid.TestError=0;
  history.Close();Check(history.Count==0,"window-close clears history");
  Check(history.DesignRecords(-1,1,2,.05).Length==0,"closed history clears next design records");
  Console.WriteLine("PASS: gain before collection, prep excluded, independent square carrier, frozen plan, repeated gain confirmation, separate30s collection, unlimited failed gain checks, input cap, actual SP, manual edits, cancel/window/timing and history isolation.");
 }
}
