using System;using System.IO;using System.Linq;using System.Collections.Generic;using System.Text.Json;using System.Threading;using PIDSupporter;
internal static class InfoStudy {
const string Root="tests/output/information-dose-20261020";
static readonly JsonSerializerOptions Options=new(){WriteIndented=true};
static void Save(string f,object x)=>File.WriteAllText(f,JsonSerializer.Serialize(x,Options));
static double[] A(JsonElement e,string k)=>e.GetProperty(k).EnumerateArray().Select(v=>v.GetDouble()).ToArray();
 sealed class Plant {
 readonly double a,b;readonly double[] q;double x,y;int tick;
 internal Plant(JsonElement p){a=p.GetProperty("p1").GetDouble();b=p.GetProperty("p2").GetDouble();q=new double[p.GetProperty("delay").GetInt32()];}
 internal double Output(){x=a*x+(1-a)*(q[tick%q.Length]-.2*x*Math.Abs(x));y=b*y+(1-b)*x;return y;}
 internal void Input(double u){q[tick++%q.Length]=u;}
 }
 static double Noise(Random r,double sigma)=>sigma*Math.Sqrt(-2*Math.Log(Math.Max(1e-15,r.NextDouble())))*Math.Cos(2*Math.PI*r.NextDouble());


static TuneRecord Read(string id,int repeat){var d=JsonDocument.Parse(File.ReadAllText(Root+$"/data/{id}-{repeat}.json")).RootElement;return new TuneRecord{U=A(d,"U"),Y=A(d,"Y"),R=A(d,"R"),InjectedSP=A(d,"InjectedSP"),T=A(d,"T"),Integrals=A(d,"Integrals"),Pid=A(d,"Pid"),Amplitude=d.GetProperty("Amplitude").GetDouble(),InitialIntegral=d.GetProperty("InitialIntegral").GetDouble(),InitialError=d.GetProperty("InitialError").GetDouble(),Lower=-1,Upper=1,Speed=.5};}
internal static void Run(string mode){Directory.CreateDirectory(Root);Directory.CreateDirectory(Root+"/data");
if(mode=="collect"){
var plants=JsonDocument.Parse(File.ReadAllText("tests/output/morelli-sp-iv-20261020/plants-evaluation-only.json")).RootElement;var plan=new List<object>();var truth=new List<object>();int index=0;
foreach(var f in plants.EnumerateArray())foreach(int controller in new[]{0,1})foreach(int seed in new[]{4101,4102,4103})foreach(string design in new[]{"base30","time60","time120","amp2","amp4","repeat2","repeat4"}){
 string id=$"d{index++:000}";int seconds=design=="time60"?60:design=="time120"?120:30,repeats=design=="repeat2"?2:design=="repeat4"?4:1;double scale=design=="amp2"?2:design=="amp4"?4:1;
 var pid=controller==0?new[]{1.0,2.5,.04}:new[]{.2,2.5,1.0};var metrics=new List<object>();
 for(int repetition=0;repetition<repeats;repetition++){
 var c=new BrilliantSkies.Ai.Control.Pids.VariableControllerMaster();c.Pid.kP.Us=(float)pid[0];c.Pid.kI.Us=(float)pid[1];c.Pid.kD.Us=(float)pid[2];var session=new AutoTuneSession(c,.5f);double amplitude=session.Amplitude;pid=(double[])session.Original.Clone();var wave=Enumerable.Range(0,600).Select(i=>session.Wave(i*.05)).ToArray();session.Cancel();
 var plant=new Plant(f);var rng=new Random(seed+repetition*10000);int n=seconds*20;var u=new double[n];var y=new double[n];var r=new double[n];var t=new double[n];var ints=new double[n];var clean=new double[n];double integ=0,prev=0;
 for(int k=0;k<n;k++){double gain=k<=40?1:k>=60?scale:1+(scale-1)*.5*(1-Math.Cos(Math.PI*(k-40)/20.0));r[k]=wave[k%600]*gain;clean[k]=plant.Output();y[k]=clean[k]+Noise(rng,.01);double err=r[k]-y[k];u[k]=NativeAutoTune.PidStep(pid,err,prev,ref integ,.05,-1,1,false);prev=err;plant.Input(u[k]);ints[k]=integ;t[k]=(k+1)*.05;}
 Save(Root+$"/data/{id}-{repetition}.json",new{U=u,Y=y,R=r,InjectedSP=r,T=t,Integrals=ints,Pid=pid,Amplitude=amplitude,InitialIntegral=integ,InitialError=prev});
 metrics.Add(new{seconds,saturation=u.Count(v=>Math.Abs(v)>=.999999)/(double)n,inputEnergy=.05*u.Sum(v=>v*v),outputRms=Math.Sqrt(clean.Average(v=>v*v)),cleanOutputStd=Math.Sqrt(clean.Select(v=>Math.Pow(v-clean.Average(),2)).Average()),maxInput=u.Max(v=>Math.Abs(v))});
 }
 plan.Add(new{id,seconds,repeats,scale,design,controller,seed});truth.Add(new{id,plant=f.GetProperty("name").GetString(),controller,seed,design,metrics});
}
Save(Root+"/plan.json",plan);Save(Root+"/evaluation-only.json",truth);return;
}
var cases=JsonDocument.Parse(File.ReadAllText(Root+"/plan.json")).RootElement;var rows=new List<object>();
if(mode=="iv" || mode=="recommend"){
 foreach(var c in cases.EnumerateArray()){
 string id=c.GetProperty("id").GetString()!;int repeats=c.GetProperty("repeats").GetInt32();var r=Read(id,0);object? selected=null;
 if(mode=="iv"){
 InfoIV.Records=Enumerable.Range(0,repeats).Select(k=>Read(id,k)).ToArray();AutoTuneDiagnostics.Sink=(k,v)=>{if(k=="linearSelected")selected=v;};InfoIV.Select(r,.05,new WorkBudget(CancellationToken.None),_=>{});rows.Add(new{id,model=selected});
 }else{
 var m=JsonDocument.Parse(File.ReadAllText(Root+"/models.json")).RootElement.EnumerateArray().First(v=>v.GetProperty("id").GetString()==id).GetProperty("model");
 InfoTune.Selected=new NativeLinear.Selection{Model=new NativeLinear{Order=m.GetProperty("order").GetInt32(),Coefficients=A(m,"coefficients"),Um=m.GetProperty("Um").GetDouble(),Us=m.GetProperty("Us").GetDouble(),Ym=m.GetProperty("Ym").GetDouble(),Ys=m.GetProperty("Ys").GetDouble()},Score=m.GetProperty("selection").GetDouble()};
 // Common observed 2s prefix is within every collection budget. Keep original SP amplitude for PID evaluation.
 var context=new TuneRecord{U=r.U.Take(40).ToArray(),Y=r.Y.Take(40).ToArray(),R=r.R.Take(40).ToArray(),InjectedSP=r.InjectedSP.Take(40).ToArray(),T=r.T.Take(40).ToArray(),Integrals=r.Integrals.Take(40).ToArray(),Pid=r.Pid,Amplitude=r.Amplitude,Speed=.5,Lower=-1,Upper=1,InitialIntegral=r.Integrals[39],InitialError=r.R[39]-r.Y[39]};
 var result=InfoTune.Run(context,CancellationToken.None,_=>{});rows.Add(new{id,result});
 }Console.WriteLine(mode+" "+id);
 }
 AutoTuneDiagnostics.Sink=null;Save(Root+(mode=="iv"?"/iv.json":"/recommendations.json"),rows);return;
}
if(mode=="grade"){
 var recommendations=JsonDocument.Parse(File.ReadAllText(Root+"/recommendations.json")).RootElement;var map=JsonDocument.Parse(File.ReadAllText(Root+"/evaluation-only.json")).RootElement;var plants=JsonDocument.Parse(File.ReadAllText("tests/output/morelli-sp-iv-20261020/plants-evaluation-only.json")).RootElement;var refs=JsonDocument.Parse(File.ReadAllText("tests/output/repeat-synthetic-20261020/actual-results.json")).RootElement;
 foreach(var f in plants.EnumerateArray()){
 string name=f.GetProperty("name").GetString()!;double sigma=.01;
 var cache=new Dictionary<string,double>();double Cost(double[] pid){string key=string.Join(",",pid);if(cache.TryGetValue(key,out var saved))return saved;double total=0;
 foreach(double amp in new[]{.1,.3,.6})foreach(double tau in new[]{.5,1.25,2.0}){double worst=0;foreach(int sign in new[]{-1,1}){double mean=0;foreach(int seed in new[]{6001,6002}){var plant=new Plant(f);var rng=new Random(seed);double integral=0,previous=0,cost=0;for(int k=0;k<300;k++){double y=plant.Output(),sp=sign*amp*(1-Math.Exp(-k*.05/tau)),e=sp-y-Noise(rng,sigma);double u=NativeAutoTune.PidStep(pid,e,previous,ref integral,.05,-1,1,false);previous=e;plant.Input(u);cost+=Math.Pow((sp-y)/amp,2)/300;}mean+=cost/2;}worst=Math.Max(worst,mean);}total+=worst/9;}cache[key]=total;return total;}


 var refrow=refs.EnumerateArray().First(v=>v.GetProperty("plant").GetString()==name&&v.GetProperty("sigma").GetDouble()==sigma);double best=Cost(A(refrow,"referencePid"));var group=recommendations.EnumerateArray().Where(v=>map.EnumerateArray().First(e=>e.GetProperty("id").GetString()==v.GetProperty("id").GetString()).GetProperty("plant").GetString()==name).ToArray();foreach(var row in group)best=Math.Min(best,Cost(A(row.GetProperty("result"),"pid")));
 foreach(var row in group){string id=row.GetProperty("id").GetString()!;var meta=map.EnumerateArray().First(v=>v.GetProperty("id").GetString()==id);var pid=A(row.GetProperty("result"),"pid");double cost=Cost(pid);rows.Add(new{id,plant=name,design=meta.GetProperty("design").GetString(),seed=meta.GetProperty("seed").GetInt32(),controller=meta.GetProperty("controller").GetInt32(),cost,regret=cost/best,referenceCost=best,pid,accepted=row.GetProperty("result").GetProperty("accepted").GetBoolean()});}
 }
 Save(Root+"/actual-results.json",rows);return;
}throw new ArgumentException(mode);
}
}
