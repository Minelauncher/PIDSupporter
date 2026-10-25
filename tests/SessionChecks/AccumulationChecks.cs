using System;
using System.Threading;
using PIDSupporter;
namespace BrilliantSkies.Ui.Consoles { public sealed class ConsoleWindow {} }
namespace PIDSupporter {
    // UI host substitute; exercise the actual window registry and actual history store.
    public sealed class AutoTuneTab {
        internal readonly AutoTuneHistory History=new AutoTuneHistory();
        internal bool IsClosed {get;private set;}
        internal void CloseHistory(){IsClosed=true;History.Close();}
    }
}
internal static class AccumulationChecks
{
 static void Check(bool b,string text){if(!b)throw new Exception(text);}
 static TuneRecord Record(double gain,double initial)
 {
  const int n=1200;var r=new TuneRecord{U=new double[n],Y=new double[n],T=new double[n],R=new double[n],InjectedSP=new double[n],BaseSP=new double[n],Integrals=new double[n],Pid=new[]{gain,2.0,.03},Lower=-1,Upper=1,Speed=.5,Amplitude=.4};
  double y=initial,u=0,integ=0,prev=0;
  for(int i=0;i<n;i++){double t=(i+1)*.025;y=.9*y+.08*u;double sp=.3*Math.Sin(t*.7)+.1*Math.Sin(t*2.1)+.1*Math.Sin(t*3.7);double e=sp-y;u=NativeAutoTune.PidStep(r.Pid,e,prev,ref integ,.025,-1,1,false);prev=e;r.T[i]=t;r.Y[i]=y;r.U[i]=u;r.R[i]=r.InjectedSP[i]=sp;r.Integrals[i]=integ;}
  r.InitialIntegral=integ;r.InitialError=prev;return r;
 }
 internal static void Run()
 {
  var a=Record(.3,.5);var b=Record(.6,-.3);var history=new AutoTuneHistory();var other=new AutoTuneHistory();
  Check(history.Add(a,1).Length==1,"first record");Check(history.Add(b,1).Length==2,"different PID retained");Check(other.Count==0,"axis isolation");
  for(int i=0;i<5;i++)history.Add(a,1);Check(history.Count==5,"bounded history");
  history.Add(b,2);Check(history.Count==1,"changed mapping clears incompatible history");history.Clear();Check(history.Count==0,"manual reset");history.Add(a,1);history.Close();Check(history.Count==0,"close clears history");
  bool blocked=false;try{history.Add(a,1);}catch(OperationCanceledException){blocked=true;}Check(blocked,"late worker cannot repopulate closed history");
  var parent=new BrilliantSkies.Ui.Consoles.ConsoleWindow();var tuner=new BrilliantSkies.Ui.Consoles.ConsoleWindow();var tab=new AutoTuneTab();var sibling=new AutoTuneTab();
  AutoTuneWindowLifetime.Register(parent,tuner,tab);AutoTuneWindowLifetime.Register(new BrilliantSkies.Ui.Consoles.ConsoleWindow(),new BrilliantSkies.Ui.Consoles.ConsoleWindow(),sibling);tab.History.Add(a,1);sibling.History.Add(b,1);
  AutoTuneWindowLifetime.Close(parent);Check(tab.IsClosed&&tab.History.Count==0,"PID parent close clears tuner history");Check(!sibling.IsClosed&&sibling.History.Count==1,"other window remains independent");AutoTuneWindowLifetime.CloseAll();Check(sibling.IsClosed&&sibling.History.Count==0,"close all");
  var selected=NativeLinear.SelectAccumulated(new[]{a,b},new WorkBudget(CancellationToken.None),_=>{});double dc=selected.Model.DcGain();Check(Math.Abs(dc-.8)<.03,"recover common plant under different controllers: "+dc);
  var coefficients=(double[])selected.Model.Coefficients.Clone();double score=selected.Score;
  for(int i=800;i<1200;i++){a.Y[i]+=100;b.Y[i]-=100;}
  var changed=NativeLinear.SelectAccumulated(new[]{a,b},new WorkBudget(CancellationToken.None),_=>{});
  Check(changed.Score==score,"held-out output cannot choose model");for(int i=0;i<coefficients.Length;i++)Check(coefficients[i]==changed.Model.Coefficients[i],"held-out coefficient leakage");
  var source=new CancellationTokenSource();source.Cancel();blocked=false;try{NativeLinear.SelectAccumulated(new[]{a,b},new WorkBudget(source.Token),_=>{});}catch(OperationCanceledException){blocked=true;}Check(blocked,"joint fitting cancellation");
  Console.WriteLine($"Accumulation checks passed: window/axis isolation, close/reset, FIFO5, PID changes, shared gain {dc:F6}, held-out isolation, cancellation.");
 }
}
