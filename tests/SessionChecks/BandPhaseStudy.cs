using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text.Json;
using System.Threading;
using PIDSupporter;
internal static class BandPhaseStudy
{
 const string Root="tests/output/band-phase-iv-20261020";
 static readonly double[] Inc={1,2.5,.04};
 static double[] Arr(JsonElement e,string key)=>e.GetProperty(key).EnumerateArray().Select(x=>x.GetDouble()).ToArray();
 static void Save(string path,object value)=>File.WriteAllText(path,JsonSerializer.Serialize(value,new JsonSerializerOptions{WriteIndented=true}));
 sealed class Plant
 {
  readonly double p1,p2;readonly double[] q;double x1,x2;int k;
  internal Plant(JsonElement f){p1=f.GetProperty("p1").GetDouble();p2=f.GetProperty("p2").GetDouble();q=new double[f.GetProperty("delay").GetInt32()];}
  internal double Next(){x1=p1*x1+(1-p1)*(q[k%q.Length]-.2*x1*Math.Abs(x1));x2=p2*x2+(1-p2)*x1;return x2;}
  internal void Input(double u){q[k++%q.Length]=u;}
 }
 static double Noise(Random rng,double sigma)=>sigma*Math.Sqrt(-2*Math.Log(Math.Max(1e-15,rng.NextDouble())))*Math.Cos(2*Math.PI*rng.NextDouble());
 internal static void Run(string mode)
 {
  Directory.CreateDirectory(Root);Directory.CreateDirectory(Root+"/data");Directory.CreateDirectory(Root+"/fits");
  if(mode=="design-input")
  {
   var rows=new List<object>();
   foreach(var pid in new[]{new[]{1.0,2.5,.04},new[]{.2,2.5,1.0},new[]{1.0,2.5,3.0}})
   {
    var c=new BrilliantSkies.Ai.Control.Pids.VariableControllerMaster();c.Pid.kP.Us=(float)pid[0];c.Pid.kI.Us=(float)pid[1];c.Pid.kD.Us=(float)pid[2];
    var session=new AutoTuneSession(c,.5f);var random=new Random(826);
    rows.Add(new{pid=session.Original,amplitude=session.Amplitude,phases=Enumerable.Range(0,60).Select(_=>random.NextDouble()*2*Math.PI).ToArray(),wave=Enumerable.Range(0,600).Select(k=>session.Wave(k*.05)).ToArray()});session.Cancel();
   }
   Save(Root+"/design-input.json",rows);return;
  }
  if(mode=="collect" || mode=="collect-stress")
  {
   var plants=JsonDocument.Parse(File.ReadAllText(Root+"/plants-evaluation-only.json")).RootElement;
   var designs=JsonDocument.Parse(File.ReadAllText(Root+"/waves.json")).RootElement;
   var mapping=new List<object>();
   if(mode=="collect-stress")mapping.AddRange(JsonDocument.Parse(File.ReadAllText(Root+"/mapping-evaluation-only.json")).RootElement.EnumerateArray().Select(e=>(object)e.Clone()));
   int index=mapping.Count;
   foreach(var f in plants.EnumerateArray())foreach(double sigma in new[]{.01})foreach(int seed in new[]{4101,4102,4103,4104})foreach(var design in designs.EnumerateArray())
   {

    string id=$"e{index++:000}";var r=Arr(design,"wave");var pid=Arr(design,"pid");
    var u=new double[600];var y=new double[600];var truth=new double[600];var t=new double[600];var ints=new double[600];
    var plant=new Plant(f);var rng=new Random(seed);double integral=0,prev=0;
    for(int k=0;k<600;k++){truth[k]=plant.Next();y[k]=truth[k]+Noise(rng,sigma);double err=r[k]-y[k];u[k]=NativeAutoTune.PidStep(pid,err,prev,ref integral,.05,-1,1,false);plant.Input(u[k]);prev=err;ints[k]=integral;t[k]=(k+1)*.05;}
    Save(Root+"/data/"+id+".json",new{U=u,Y=y,R=r,T=t,Integrals=ints,Pid=pid,Lower=-1,Upper=1,Speed=.5,Amplitude=design.GetProperty("amplitude").GetDouble(),InitialIntegral=integral,InitialError=prev});
    mapping.Add(new{id,plant=f.GetProperty("name").GetString(),sigma,seed,controller=design.GetProperty("controller").GetInt32(),design=design.GetProperty("design").GetString(),uPeak=u.Max(v=>Math.Abs(v)),uEnergy=.05*u.Sum(v=>v*v),saturation=u.Count(v=>Math.Abs(v)>=.999999)/600.0,yRms=Math.Sqrt(truth.Average(v=>v*v)),yPeak=truth.Max(v=>Math.Abs(v)),spRms=Math.Sqrt(r.Average(v=>v*v))});
   }
   Save(Root+"/mapping-evaluation-only.json",mapping);Console.WriteLine("Collected "+index);return;
  }
  if(mode=="fit")
  {
   foreach(string path in Directory.GetFiles(Root+"/data","*.json").OrderBy(p=>p))
   {
    string id=Path.GetFileNameWithoutExtension(path),output=Root+"/fits/"+id+".json";if(File.Exists(output))continue;
    var d=JsonDocument.Parse(File.ReadAllText(path)).RootElement;
    var r=new TuneRecord{U=Arr(d,"U"),Y=Arr(d,"Y"),R=Arr(d,"R"),T=Arr(d,"T"),Integrals=Arr(d,"Integrals"),Pid=Arr(d,"Pid"),Lower=-1,Upper=1,Speed=.5,Amplitude=d.GetProperty("Amplitude").GetDouble(),InitialIntegral=d.GetProperty("InitialIntegral").GetDouble(),InitialError=d.GetProperty("InitialError").GetDouble()};
    object? model=null,search=null;AutoTuneDiagnostics.Sink=(kind,v)=>{if(kind=="linearSelected")model=v;if(kind=="pidSearch")search=v;};
    var clock=Stopwatch.StartNew();TuneResult result;try{result=NativeAutoTune.Run(r,CancellationToken.None,_=>{});}catch(Exception ex){result=new TuneResult{message=ex.Message};}
    Save(output,new{id,result,model,search,wallSeconds=clock.Elapsed.TotalSeconds});Console.WriteLine(id+" accepted="+result.accepted+" time="+clock.Elapsed.TotalSeconds.ToString("F2"));
   }
   AutoTuneDiagnostics.Sink=null;return;
  }
  if(mode=="grade")
  {
   var map=JsonDocument.Parse(File.ReadAllText(Root+"/mapping-evaluation-only.json")).RootElement;
   var plants=JsonDocument.Parse(File.ReadAllText(Root+"/plants-evaluation-only.json")).RootElement;
   var results=new List<object>();
   foreach(var f in plants.EnumerateArray())foreach(double sigma in new[]{.01})foreach(int controller in new[]{0,1,2})
   {
    string name=f.GetProperty("name").GetString()!;
    var cases=map.EnumerateArray().Where(e=>e.GetProperty("plant").GetString()==name&&e.GetProperty("sigma").GetDouble()==sigma&&e.GetProperty("controller").GetInt32()==controller).ToArray();
    var first=JsonDocument.Parse(File.ReadAllText(Root+"/data/"+cases[0].GetProperty("id").GetString()+".json")).RootElement;
    var incumbent=Arr(first,"Pid");var pool=NativeAutoTune.Candidates(incumbent);var indices=new Dictionary<string,int>();
    foreach(var c in cases){string id=c.GetProperty("id").GetString()!;var rr=JsonDocument.Parse(File.ReadAllText(Root+"/fits/"+id+".json")).RootElement.GetProperty("result");indices[id]=pool.Count;pool.Add(rr.GetProperty("accepted").GetBoolean()?Arr(rr,"pid"):incumbent);}
    double Cost(double[] pid,NativeLinear? model)
    {
     double total=0;
     foreach(double amp in new[]{.1,.3,.6})foreach(double tau in new[]{.5,1.25,2.0})
     {
      double worst=0;
      foreach(int sign in new[]{-1,1})
      {
       double mean=0;
       foreach(int seed in new[]{6001,6002})
       {
        var plant=new Plant(f);double[] state=model==null?Array.Empty<double>():new double[model.Order*2];
        if(model!=null)for(int k=0;k<model.Order;k++)model.Push(state,0,0);
        var rng=new Random(seed);double integral=0,prev=0,cost=0;
        for(int k=0;k<300;k++)
        {
         double y=model==null?plant.Next():model.Next(state);
         if(!NativeLinear.Finite(y)||Math.Abs(y)>100)return 1e12;
         double sp=sign*amp*(1-Math.Exp(-k*.05/tau)),err=sp-y-Noise(rng,sigma);
         double u=NativeAutoTune.PidStep(pid,err,prev,ref integral,.05,-1,1,false);prev=err;
         if(model==null)plant.Input(u);else model.Push(state,y,u);
         cost+=Math.Pow((sp-y)/amp,2)/300;
        }
        mean+=cost/2;
       }
       worst=Math.Max(worst,mean);
      }
      total+=worst/9;
     }
     return total;
    }
    var scores=pool.Select(pid=>Cost(pid,null)).ToArray();double best=scores.Min();int bi=Array.IndexOf(scores,best);
    foreach(var c in cases)
    {
     string id=c.GetProperty("id").GetString()!;int j=indices[id];var fit=JsonDocument.Parse(File.ReadAllText(Root+"/fits/"+id+".json")).RootElement;
     var m=fit.GetProperty("model");double freeError=-1;double[] predicted=Array.Empty<double>();
     if(m.ValueKind!=JsonValueKind.Null)
     {
      var model=new NativeLinear{Order=m.GetProperty("order").GetInt32(),Coefficients=Arr(m,"coefficients"),Um=m.GetProperty("Um").GetDouble(),Us=m.GetProperty("Us").GetDouble(),Ym=m.GetProperty("Ym").GetDouble(),Ys=m.GetProperty("Ys").GetDouble()};
      predicted=pool.Select(pid=>Cost(pid,model)).ToArray();
      // Independent prescribed-input validation, no output corrections and no identification feedback.
      var plant=new Plant(f);var state=new double[2*model.Order];for(int k=0;k<model.Order;k++)model.Push(state,0,0);
      double sum=0,energy=0;
      for(int k=0;k<600;k++)
      {
       double y=plant.Next(),yp=model.Next(state),u=.2*Math.Sin(.31*k*.05)+.15*Math.Sin(1.7*k*.05)+.1*Math.Sin(4.3*k*.05);
       if(!NativeLinear.Finite(yp)||Math.Abs(yp)>1e6){sum=1e24;break;}
       sum+=(yp-y)*(yp-y);energy+=y*y;plant.Input(u);model.Push(state,yp,u);
      }
      freeError=Math.Sqrt(sum/Math.Max(energy,1e-12));
     }
     results.Add(new{id,plant=name,sigma,controller,seed=c.GetProperty("seed").GetInt32(),design=c.GetProperty("design").GetString(),pid=pool[j],cost=scores[j],incumbent=scores[0],ratio=scores[j]/scores[0],referenceBest=best,referencePid=pool[bi],regret=scores[j]/best,freeError,actualLandscape=scores,predictedLandscape=predicted});
    }
    Console.WriteLine("Graded "+name+" noise="+sigma+" controller="+controller);
   }
   Save(Root+"/actual-results.json",results);return;
  }
  throw new ArgumentException(mode);
 }
}
