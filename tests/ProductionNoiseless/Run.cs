using System;using System.IO;using System.Linq;using System.Text.Json;using System.Collections.Generic;using System.Reflection;using System.Threading;using PIDSupporter;using BrilliantSkies.Ai.Control.Pids;
internal static class Run {
static JsonSerializerOptions Opt=new(){NumberHandling=System.Text.Json.Serialization.JsonNumberHandling.AllowNamedFloatingPointLiterals};
sealed class Plant {
 internal double[] X,Q;internal double Cross,Time;readonly JsonElement spec;readonly double gain;readonly bool coupled;
 internal Plant(JsonElement c){spec=c.GetProperty("plant");X=new double[spec.GetProperty("n").GetInt32()];X[0]=c.GetProperty("initial").GetDouble();if(X.Length>1)X[1]=-.5*X[0];Q=new double[c.GetProperty("delay").GetInt32()];gain=c.GetProperty("gain").GetDouble();coupled=c.GetProperty("coupled").GetBoolean();}
 internal double Step(double u){Array.Copy(Q,1,Q,0,Q.Length-1);Q[Q.Length-1]=u;double act=gain*Q[0],y;
 if(spec.GetProperty("type").GetString()=="nonlinear"){double p1=spec.GetProperty("p1").GetDouble(),p2=spec.GetProperty("p2").GetDouble();X[0]=p1*X[0]+(1-p1)*(act-.2*X[0]*Math.Abs(X[0]));X[1]=p2*X[1]+(1-p2)*X[0];y=X[1];}
 else {var A=spec.GetProperty("A");var B=spec.GetProperty("B");var C=spec.GetProperty("C");var xn=new double[X.Length];for(int i=0;i<X.Length;i++){xn[i]=B[i].GetDouble()*act;for(int j=0;j<X.Length;j++)xn[i]+=A[i][j].GetDouble()*X[j];}X=xn;y=0;for(int i=0;i<X.Length;i++)y+=C[i].GetDouble()*X[i];}
 Time+=.05;if(coupled)Cross+=.05*(-.7*Cross+.4*y+.8*Math.Sin(2*Math.PI*.11*Time)+.4*Math.Sin(2*Math.PI*.23*Time));return y-(coupled?.8*Cross:0);}
}
static void Main(string[] args){string root=args[0];var cases=JsonDocument.Parse(File.ReadAllText(Path.Combine(root,"cases-evaluation-only.json"))).RootElement;Directory.CreateDirectory(Path.Combine(root,"runs"));
foreach(var c in cases.EnumerateArray()){
 string id=c.GetProperty("id").GetString()!,path=Path.Combine(root,"runs",id+".json");if(File.Exists(path))continue;
 var ctrl=new VariableControllerMaster();ctrl.SetPointMultiple.Us=1;ctrl.SetPointAdjust.Us=0;ctrl.Pid.kP.Us=.4f;ctrl.Pid.kI.Us=2.5f;ctrl.Pid.kD.Us=.2f;
 var events=new List<object>();object? record=null,model=null,search=null;AutoTuneDiagnostics.Sink=(k,v)=>{if(k=="record")record=v;if(k=="linearAccumulatedSelected")model=v;if(k=="pidSearch")search=v;if(k!="record"&&k!="operatingContextSamples"&&k!="linearConstrainedCandidate")lock(events)events.Add(new{kind=k,value=v});};
 UnityEngine.Time.realtimeSinceStartup=0;var session=new AutoTuneSession(ctrl,.5f);typeof(AutoTuneSession).GetField("ExcitationSeed",BindingFlags.Instance|BindingFlags.NonPublic)!.SetValue(session,c.GetProperty("seed").GetInt32());var plant=new Plant(c);double integral=0,prev=0,u=0;float previousTime=0;bool timedout=false;int count=0;var watch=System.Diagnostics.Stopwatch.StartNew();
 for(int k=0;session.Collecting;k++){
  float timestamp=(float)(k*.05);UnityEngine.Time.realtimeSinceStartup=timestamp;session.Heartbeat=timestamp;
  if(!session.CollectionStarted&&timestamp>=180){timedout=true;session.Cancel("Harness preparation limit180s");break;}
  double yy=plant.Step(u);if(!double.IsFinite(yy)||Math.Abs(yy)>1e8){session.Cancel("Harness diverged");break;}
  AutoTuneMeasurementPatch.Prefix(ctrl,timestamp,out var state);float y=(float)yy;float target=(float)(c.GetProperty("baseSP").GetDouble()+ctrl.SetPointAdjust.Us);float error=target-y;
  u=NativeAutoTune.PidStep(AutoTuneSession.ReadPid(ctrl),error,prev,ref integral,timestamp-previousTime,-1,1,k==0);prev=error;previousTime=timestamp;ctrl.Pid.TestError=error;ctrl.Pid.TestIntegral=(float)integral;
  AutoTuneMeasurementPatch.Postfix(ctrl,y,(float)u,state);count++;
 }
 var endpoint=new{plant.X,plant.Q,plant.Cross,plant.Time,lastU=u,integral,error=prev};
 if(!session.Cancelled){var wait=System.Diagnostics.Stopwatch.StartNew();while(session.Result==null&&wait.Elapsed.TotalMinutes<11){session.Poll();Thread.Sleep(5);}if(session.Result==null)session.Cancel("Harness worker timeout");}
 var row=new{id,timedout,cancelled=session.Cancelled,message=session.Message,preparationAttempts=session.PreparationAttempts,collectionStarted=session.CollectionStarted,elapsed=session.Elapsed,amplitude=session.CollectionAmplitude,workingPid=session.WorkingPid,result=session.Result,record,model,search,endpoint,events,wallSeconds=watch.Elapsed.TotalSeconds};File.WriteAllText(path,JsonSerializer.Serialize(row,Opt));
 Console.WriteLine(id+" "+(session.Result?.reason??session.Message)+" P="+string.Join(",",session.Result?.pid??session.WorkingPid)+" seconds="+watch.Elapsed.TotalSeconds.ToString("F1"));AutoTuneDiagnostics.Sink=null;AutoTuneSession.Active=null;
}
}
}
