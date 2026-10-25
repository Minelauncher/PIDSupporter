using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using System.Text.Json;
using System.Threading;
using PIDSupporter;
internal static class RepeatStudy {
 const string Root="tests/output/repeat-synthetic-20261020";
 static readonly JsonSerializerOptions Opt=new(){WriteIndented=true,NumberHandling=System.Text.Json.Serialization.JsonNumberHandling.AllowNamedFloatingPointLiterals};
 static void Save(string f,object x)=>File.WriteAllText(f,JsonSerializer.Serialize(x,Opt));
 static double[] A(JsonElement e,string k)=>e.GetProperty(k).EnumerateArray().Select(v=>v.GetDouble()).ToArray();
 sealed class Plant {
 readonly double a,b;readonly double[] q;double x,y;int tick;
 internal Plant(JsonElement p){a=p.GetProperty("p1").GetDouble();b=p.GetProperty("p2").GetDouble();q=new double[p.GetProperty("delay").GetInt32()];}
 internal double Output(){x=a*x+(1-a)*(q[tick%q.Length]-.2*x*Math.Abs(x));y=b*y+(1-b)*x;return y;}
 internal void Input(double u){q[tick++%q.Length]=u;}
 }
 static double Noise(Random r,double sigma)=>sigma*Math.Sqrt(-2*Math.Log(Math.Max(1e-15,r.NextDouble())))*Math.Cos(2*Math.PI*r.NextDouble());
 static TuneRecord Collect(Plant plant,double[] pid,int seed,double sigma,ref double integral,ref double previous){
 var c=new BrilliantSkies.Ai.Control.Pids.VariableControllerMaster();c.Pid.kP.Us=(float)pid[0];c.Pid.kI.Us=(float)pid[1];c.Pid.kD.Us=(float)pid[2];var wave=new AutoTuneSession(c,.5f);
 var z=Enumerable.Range(0,600).Select(k=>wave.Wave(k*.05)).ToArray();double amplitude=wave.Amplitude;pid=(double[])wave.Original.Clone();wave.Cancel();
 var r=new TuneRecord{R=z,InjectedSP=z,BaseSP=new double[600],U=new double[600],Y=new double[600],T=new double[600],Integrals=new double[600],Pid=pid,Speed=.5,Amplitude=amplitude,Lower=-1,Upper=1};var random=new Random(seed);
 for(int k=0;k<600;k++){r.Y[k]=plant.Output()+Noise(random,sigma);double e=z[k]-r.Y[k];r.U[k]=NativeAutoTune.PidStep(pid,e,previous,ref integral,.05,-1,1,false);previous=e;plant.Input(r.U[k]);r.Integrals[k]=integral;r.T[k]=(k+1)*.05;}
 r.InitialIntegral=integral;r.InitialError=previous;return r;
 }
 static JointLinear Model(object value){var e=JsonSerializer.SerializeToElement(value);return new JointLinear{Order=e.GetProperty("order").GetInt32(),Coefficients=A(e,"coefficients"),Um=e.GetProperty("Um").GetDouble(),Ym=e.GetProperty("Ym").GetDouble(),Us=e.GetProperty("Us").GetDouble(),Ys=e.GetProperty("Ys").GetDouble()};}
 internal static void Run(string mode){Directory.CreateDirectory(Root);var plants=JsonDocument.Parse(File.ReadAllText("tests/output/morelli-sp-iv-20261020/plants-evaluation-only.json")).RootElement;
 if(mode.StartsWith("fit")){
 int window=int.Parse(mode.Substring(3));
 foreach(var f in plants.EnumerateArray())foreach(double sigma in new[]{0.0,.01})foreach(int seed in new[]{4101,4102}){
 string id=$"{f.GetProperty("name").GetString()}-n{sigma}-s{seed}-w{window}";var hist=new List<TuneRecord>();var rows=new List<object>();var plant=new Plant(f);double[] pid={1,2.5,.04};double integral=0,previous=0;JointLinear? prevModel=null;
 for(int iteration=1;iteration<=12;iteration++){
 var r=Collect(plant,pid,seed+iteration*10000,sigma,ref integral,ref previous);hist.Add(r);
 Save(Root+$"/{id}-r{iteration}.json",new{r.U,r.Y,r.R,r.InjectedSP,r.T,r.Integrals,r.Pid,r.Amplitude,r.InitialIntegral,r.InitialError});
 var nextError=prevModel?.Forecast(r,.05,new WorkBudget(CancellationToken.None));
 JointLinear.Records=hist.Skip(Math.Max(0,hist.Count-window)).ToArray();object? selected=null,search=null;AutoTuneDiagnostics.Sink=(k,v)=>{if(k=="linearSelected")selected=v;if(k=="pidSearch")search=v;};
 TuneResult result;try{result=JointTune.Run(r,CancellationToken.None,_=>{});}catch(Exception ex){result=new TuneResult{pid=pid,reason="fit_failed",message=ex.Message};}
 if(result.accepted && result.pid!=null)pid=result.pid.Select(v=>(double)(float)v).ToArray();
 prevModel=selected==null?null:Model(selected);
 rows.Add(new{iteration,usedPid=r.Pid,pid=(double[])pid.Clone(),result,selected,search,nextError,saturation=r.U.Count(v=>Math.Abs(v)>=.999999)/600.0});
 Save(Root+$"/{id}-trajectory.json",new{id,plant=f.GetProperty("name").GetString(),sigma,seed,window,rows});Console.WriteLine(id+" iter="+iteration+" accepted="+result.accepted);
 }
 }
 AutoTuneDiagnostics.Sink=null;return;
 }
 if(mode=="grade"){
 var trajectories=Directory.GetFiles(Root,"*-trajectory.json").Select(f=>JsonDocument.Parse(File.ReadAllText(f)).RootElement.Clone()).ToArray();var output=new List<object>();
 foreach(var f in plants.EnumerateArray())foreach(double sigma in new[]{0.0,.01}){
 string name=f.GetProperty("name").GetString()!;var group=trajectories.Where(e=>e.GetProperty("plant").GetString()==name&&e.GetProperty("sigma").GetDouble()==sigma).ToArray();var pool=new List<double[]>();
 foreach(double p in new[]{.001,.01,.02,.05,.1,.2,.4,.6,.8,1.0})foreach(double ti in new[]{.1,.2,.4,.8,1.5,3,6,12,30,100,250})foreach(double td in new[]{0,.1,.2,.4,.8,1.5,3})pool.Add(new[]{p,ti,td});
 pool.Add(new[]{1,2.5,.04});foreach(var tr in group)foreach(var row in tr.GetProperty("rows").EnumerateArray())pool.Add(A(row,"pid"));
 var cache=new Dictionary<string,double>();double Cost(double[] pid){string key=string.Join(",",pid);if(cache.TryGetValue(key,out var saved))return saved;double total=0;
 foreach(double amp in new[]{.1,.3,.6})foreach(double tau in new[]{.5,1.25,2.0}){double worst=0;foreach(int sign in new[]{-1,1}){double mean=0;foreach(int seed in new[]{6001,6002}){var plant=new Plant(f);var rng=new Random(seed);double integral=0,previous=0,cost=0;for(int k=0;k<300;k++){double y=plant.Output(),sp=sign*amp*(1-Math.Exp(-k*.05/tau)),e=sp-y-Noise(rng,sigma);double u=NativeAutoTune.PidStep(pid,e,previous,ref integral,.05,-1,1,false);previous=e;plant.Input(u);cost+=Math.Pow((sp-y)/amp,2)/300;}mean+=cost/2;}worst=Math.Max(worst,mean);}total+=worst/9;}cache[key]=total;return total;}
 var scores=pool.Select(Cost).ToArray();double best=scores.Min();int bi=Array.IndexOf(scores,best);
 // Evaluation-only search. Nothing here feeds collection, identification or decisions.
 foreach(var seedPid in new[]{new[]{1.0,2.5,.04},pool[bi]}) PidSearch.Run(seedPid,Cost,new WorkBudget(CancellationToken.None),_=>{});
 var winner=cache.OrderBy(kv=>kv.Value).First();best=winner.Value;var referencePid=winner.Key.Split(',').Select(v=>double.Parse(v,System.Globalization.CultureInfo.InvariantCulture)).ToArray();
 foreach(var tr in group){var graded=tr.GetProperty("rows").EnumerateArray().Select(row=>new{iteration=row.GetProperty("iteration").GetInt32(),pid=A(row,"pid"),accepted=row.GetProperty("result").GetProperty("accepted").GetBoolean(),cost=Cost(A(row,"pid")),regret=Cost(A(row,"pid"))/best}).ToArray();output.Add(new{id=tr.GetProperty("id").GetString(),plant=name,sigma,seed=tr.GetProperty("seed").GetInt32(),window=tr.GetProperty("window").GetInt32(),referenceBest=best,referencePid,referenceCandidates=cache.Count,initialCost=Cost(new[]{1,2.5,.04}),rows=graded});}
 Console.WriteLine("Graded "+name+" noise="+sigma);
 }
 Save(Root+"/actual-results.json",output);return;
 }throw new ArgumentException(mode);
 }
}
