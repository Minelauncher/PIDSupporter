using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text.Json;
using System.Threading;
using PIDSupporter;
internal static class ExcitationStudy
{
 const string Root="tests/output/excitation-hierarchy-20261020";
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
 static double[] Wave(int tones,int seed,double peak,bool baseline)
 {
  var rng=new Random(seed);var phases=new double[tones];for(int j=0;j<tones;j++)phases[j]=rng.NextDouble()*2*Math.PI;
  var r=new double[600];
  for(int k=0;k<600;k++)for(int j=0;j<tones;j++)
  {
   double w=2*Math.PI*(j+1)/30;double reactive=w*Inc[2]-1/(w*Inc[1]);
   r[k]+=Math.Sin(w*k*.05+phases[j])/(baseline?Math.Sqrt(1+reactive*reactive):1);
  }
  double scale=baseline?20:r.Max(v=>Math.Abs(v));for(int k=0;k<600;k++)r[k]=peak*NativeAutoTune.Clamp(r[k]/scale,-1,1);return r;
 }
 internal static void Run(string mode)
 {
  Directory.CreateDirectory(Root);Directory.CreateDirectory(Root+"/data");Directory.CreateDirectory(Root+"/fits");
  if(mode=="collect")
  {
   var plants=JsonDocument.Parse(File.ReadAllText(Root+"/plants-evaluation-only.json")).RootElement;var mapping=new List<object>();int index=0;
   foreach(var f in plants.EnumerateArray())foreach(double sigma in new[]{0,.01})
   {
    var designs=new List<(string name,int tones,int seed,double peak,bool baseline)>{("current",60,826,.2,true)};
    foreach(int tones in new[]{12,30,60})foreach(double peak in new[]{.2,.6})foreach(int phase in new[]{826,827})designs.Add(($"f{tones}_a{peak}_p{phase}",tones,phase,peak,false));
    foreach(var design in designs)
    {
     string id=$"e{index++:000}";var r=Wave(design.tones,design.seed,design.peak,design.baseline);var u=new double[600];var y=new double[600];var t=new double[600];var ints=new double[600];var p=new Plant(f);var rng=new Random(4101);double integral=0,prev=0;
     for(int k=0;k<600;k++){y[k]=p.Next()+Noise(rng,sigma);double err=r[k]-y[k];u[k]=NativeAutoTune.PidStep(Inc,err,prev,ref integral,.05,-1,1,false);p.Input(u[k]);prev=err;ints[k]=integral;t[k]=(k+1)*.05;}
     Save(Root+"/data/"+id+".json",new{U=u,Y=y,R=r,T=t,Integrals=ints,Pid=Inc,Lower=-1,Upper=1,Speed=.5,Amplitude=1,InitialIntegral=integral,InitialError=prev});
     mapping.Add(new{id,plant=f.GetProperty("name").GetString(),sigma,design=design.name,tones=design.tones,peak=design.peak,phase=design.seed,uEnergy=.05*u.Sum(v=>v*v),uRms=Math.Sqrt(u.Average(v=>v*v)),saturation=u.Count(v=>Math.Abs(v)>=.999999)/600.0,yRms=Math.Sqrt(y.Average(v=>v*v)),yPeak=y.Max(v=>Math.Abs(v)),spRms=Math.Sqrt(r.Average(v=>v*v))});
    }
   }
   Save(Root+"/mapping-evaluation-only.json",mapping);Console.WriteLine("Collected "+index);return;
  }
  if(mode=="fit")
  {
   foreach(string path in Directory.GetFiles(Root+"/data","*.json").OrderBy(p=>p))
   {
    string id=Path.GetFileNameWithoutExtension(path),output=Root+"/fits/"+id+".json";if(File.Exists(output))continue;
    var d=JsonDocument.Parse(File.ReadAllText(path)).RootElement;
    var r=new TuneRecord{U=Arr(d,"U"),Y=Arr(d,"Y"),R=Arr(d,"R"),T=Arr(d,"T"),Integrals=Arr(d,"Integrals"),Pid=Arr(d,"Pid"),Lower=-1,Upper=1,Speed=.5,Amplitude=1,InitialIntegral=d.GetProperty("InitialIntegral").GetDouble(),InitialError=d.GetProperty("InitialError").GetDouble()};
    object? model=null,search=null;AutoTuneDiagnostics.Sink=(kind,v)=>{if(kind=="linearSelected")model=v;if(kind=="pidSearch")search=v;};
    var clock=Stopwatch.StartNew();TuneResult result;try{result=NativeAutoTune.Run(r,CancellationToken.None,_=>{});}catch(Exception ex){result=new TuneResult{message=ex.Message};}
    Save(output,new{id,result,model,search,wallSeconds=clock.Elapsed.TotalSeconds});Console.WriteLine(id+" accepted="+result.accepted+" time="+clock.Elapsed.TotalSeconds.ToString("F2"));
   }
   AutoTuneDiagnostics.Sink=null;return;
  }
  if(mode=="grade")
  {
   // Caller verifies frozen fit hashes before this separate process reads any plant data.
   var map=JsonDocument.Parse(File.ReadAllText(Root+"/mapping-evaluation-only.json")).RootElement;var plants=JsonDocument.Parse(File.ReadAllText(Root+"/plants-evaluation-only.json")).RootElement;
   var results=new List<object>();
   foreach(var f in plants.EnumerateArray())foreach(double sigma in new[]{0,.01})
   {
    string name=f.GetProperty("name").GetString()!;var cases=map.EnumerateArray().Where(e=>e.GetProperty("plant").GetString()==name&&e.GetProperty("sigma").GetDouble()==sigma).ToArray();
    var pool=NativeAutoTune.Candidates(Inc);var indices=new Dictionary<string,int>();var accepted=new Dictionary<string,bool>();
    foreach(var c in cases){string id=c.GetProperty("id").GetString()!;var rr=JsonDocument.Parse(File.ReadAllText(Root+"/fits/"+id+".json")).RootElement.GetProperty("result");bool ok=rr.GetProperty("accepted").GetBoolean();indices[id]=pool.Count;accepted[id]=ok;pool.Add(ok?Arr(rr,"pid"):Inc);}
    var scores=new double[pool.Count];var worst=new double[pool.Count];
    for(int j=0;j<pool.Count;j++)
    {
     foreach(double amp in new[]{.1,.3,.6})foreach(double tau in new[]{.5,1.25,2.0})
     {
      double worstSign=0;
      foreach(int sign in new[]{-1,1})
      {
       double mean=0;
       for(int seed=6001;seed<=6004;seed++)
       {
        var p=new Plant(f);var rng=new Random(seed);double integral=0,prev=0,cost=0;
        for(int k=0;k<300;k++){double y=p.Next(),sp=sign*amp*(1-Math.Exp(-k*.05/tau)),err=sp-y-Noise(rng,sigma);double u=NativeAutoTune.PidStep(pool[j],err,prev,ref integral,.05,-1,1,false);prev=err;p.Input(u);cost+=Math.Pow((sp-y)/amp,2)/300;}
        mean+=cost/4;
       }
       worstSign=Math.Max(worstSign,mean);
      }
      scores[j]+=worstSign/9;worst[j]=Math.Max(worst[j],worstSign);
     }
    }
    double best=scores.Min();int bi=Array.IndexOf(scores,best);
    foreach(var c in cases){string id=c.GetProperty("id").GetString()!;int j=indices[id];results.Add(new{id,plant=name,sigma,design=c.GetProperty("design").GetString(),accepted=accepted[id],pid=pool[j],cost=scores[j],worstTask=worst[j],incumbent=scores[0],ratio=scores[j]/scores[0],referenceBest=best,referencePid=pool[bi],regret=scores[j]/best});}
    Console.WriteLine("Graded "+name+" noise="+sigma);
   }
   Save(Root+"/actual-results.json",results);return;
  }
  throw new ArgumentException(mode);
 }
}
