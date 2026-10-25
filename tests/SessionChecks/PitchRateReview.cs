using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using System.Text.Json;
using System.Text.Json.Serialization;
using System.Threading;
using PIDSupporter;
internal static class PitchRateReview
{
 const string Root="tests/output/pitch-rate-20261020";
 static double[] A(JsonElement e,string k)=>e.GetProperty(k).EnumerateArray().Select(v=>v.GetDouble()).ToArray();
 sealed class Model
 {
  public int order {get;set;} public double ridge {get;set;} public double gain {get;set;} public double poleRadius {get;set;}
  public double[] coefficients {get;set;}=null!;
  public double Um {get;set;} public double Us {get;set;} public double Ym {get;set;} public double Ys {get;set;}
  public NativeLinear Net()=>new NativeLinear{Order=order,Coefficients=coefficients,Um=Um,Us=Us,Ym=Ym,Ys=Ys};
 }
 sealed class Context
 {
  internal TuneRecord r=null!;internal double[] omega=null!,roll=null!,yaw=null!;
  internal double dt,angleScale;internal WorkBudget budget=new WorkBudget(CancellationToken.None);
  internal double[] State(NativeLinear net,bool recent){
   var state=new double[2*net.Order];int end=recent?r.U.Length:net.Order;
   for(int i=end-net.Order;i<end;i++)net.Push(state,omega[i],r.U[i]);
   return state;
  }
  internal object Evaluate(Model model,string mode){
   var net=model.Net();var fs=State(net,false);var cs=State(net,false);int n=r.U.Length,o=net.Order;
   double yf=r.Y[o-1],yc=yf,wf=omega[o-1],wc=wf;
   int sw=(int)Math.Round(4/dt);double integral=r.Integrals[sw-1],prev=r.R[sw-1]-r.Y[sw-1];
   var sums=new[]{new double[4],new double[4]};var counts=new int[2];
   var free=new double[n];var closed=new double[n];var input=new double[n];var rates=new double[n];
   Array.Copy(r.Y,free,o);Array.Copy(r.Y,closed,o);
   for(int i=o;i<n;i++){
    double nf=net.Next(fs),nc=net.Next(cs),h=r.T[i]-r.T[i-1];
    int g=mode=="recorded-geometry"?i:o-1;
    double cos=Math.Cos(roll[g]),dist=-yaw[g]*Math.Sin(roll[g]);
    yf+=h*(.5*(wf+nf)*cos+dist);yc+=h*(.5*(wc+nc)*cos+dist);
    double u=r.U[i];
    if(i>=sw){double e=r.R[i]-yc;u=NativeAutoTune.PidStep(r.Pid,e,prev,ref integral,h,r.Lower,r.Upper,false);prev=e;}
    free[i]=yf;closed[i]=yc;input[i]=u;rates[i]=nf;
    if(i>=n/2){int k=i<n*2/3?0:1;counts[k]++;
     sums[k][0]+=Math.Pow((yf-r.Y[i])/angleScale,2);sums[k][1]+=Math.Pow((yc-r.Y[i])/angleScale,2);
     sums[k][2]+=Math.Pow((u-r.U[i])/net.Us,2);sums[k][3]+=Math.Pow((nf-omega[i])/net.Ys,2);}
    net.Push(fs,nf,r.U[i]);net.Push(cs,nc,u);wf=nf;wc=nc;
   }
   var validation=sums[0].Select(v=>Math.Sqrt(v/counts[0])).ToArray();var final=sums[1].Select(v=>Math.Sqrt(v/counts[1])).ToArray();
   return new{model,mode,validation,final,score=validation.Take(3).Max(),free,closed,input,rates};
  }
  internal Func<double[],PidSearch.Motion> Measure(Model model,bool holdYaw){
   var net=model.Net();var warm=State(net,true);int n=r.U.Length;
   double cos=Math.Cos(roll[n-1]),dist=holdYaw?-yaw[n-1]*Math.Sin(roll[n-1]):0;
   double delta=Math.Max(.5*r.Amplitude,.1*angleScale),bias=r.U.Skip(n-(int)Math.Round(1/dt)).Average();
   var cache=new Dictionary<(double,double,double),PidSearch.Motion>();
   return candidate=>{
    var key=(candidate[0],candidate[1],candidate[2]);if(cache.TryGetValue(key,out var found))return found;
    double worst=0,ws=0,we=0;int steps=(int)Math.Round(15/dt);
    foreach(int sign in new[]{-1,1}){
     var state=(double[])warm.Clone();double y=r.Y[n-1],w=omega[n-1],last=r.U[n-1],integral=r.InitialIntegral,prev=r.InitialError;
     double cost=0,slew=0,effort=0;
     for(int k=0;k<steps;k++){
      if((k&31)==0)budget.Check();double nextW=net.Next(state);y+=dt*(.5*(w+nextW)*cos+dist);
      double target=r.Y[n-1]+sign*delta*(1-Math.Exp(-k*dt/(2-1.5*r.Speed))),error=target-y;
      double u=NativeAutoTune.PidStep(candidate,error,prev,ref integral,dt,r.Lower,r.Upper,false);prev=error;
      double c=Math.Pow(error/delta,2);if(!NativeLinear.Finite(c)||Math.Abs(y-r.Y[n-1])>20*delta){cost=1e12*steps;break;}
      cost+=c;slew+=Math.Pow((u-last)/((r.Upper-r.Lower)*dt),2);effort+=Math.Pow((u-bias)/(r.Upper-r.Lower),2);
      net.Push(state,nextW,u);last=u;w=nextW;
     }
     worst=Math.Max(worst,cost/steps);ws=Math.Max(ws,slew/steps);we=Math.Max(we,effort/steps);
    }
    var result=new PidSearch.Motion{Tracking=worst,Slew=ws,Effort=we};cache[key]=result;return result;
   };
  }
 }
 internal static void Run(){
  var options=new JsonSerializerOptions{NumberHandling=JsonNumberHandling.AllowNamedFloatingPointLiterals,IncludeFields=true};
  var runs=JsonDocument.Parse(File.ReadAllText("tests/output/air-context-20261020/runs.json")).RootElement;
  var data=JsonDocument.Parse(File.ReadAllText(Root+"/rate-models.json")).RootElement;
  foreach(var datum in data.EnumerateArray()){
   int index=datum.GetProperty("index").GetInt32();var run=runs[index-1];var d=run.GetProperty("record");
   var r=new TuneRecord{U=A(d,"U"),Y=A(d,"Y"),R=A(d,"R"),T=A(d,"T"),Pid=A(d,"Pid"),Integrals=A(d,"Integrals"),InjectedSP=A(d,"InjectedSP"),BaseSP=A(d,"BaseSP"),Speed=run.GetProperty("start").GetProperty("speed").GetDouble(),Amplitude=d.GetProperty("Amplitude").GetDouble(),Lower=d.GetProperty("Lower").GetDouble(),Upper=d.GetProperty("Upper").GetDouble(),InitialIntegral=d.GetProperty("InitialIntegral").GetDouble(),InitialError=d.GetProperty("InitialError").GetDouble()};
   double mean=r.Y.Take(r.Y.Length/2).Average();
   var ctx=new Context{r=r,dt=NativeAutoTune.Validate(r),omega=A(datum,"omega"),roll=A(datum,"roll"),yaw=A(datum,"yaw"),angleScale=Math.Sqrt(r.Y.Take(r.Y.Length/2).Average(v=>(v-mean)*(v-mean)))};
   var models=datum.GetProperty("models").Deserialize<Model[]>()!;
   var lm=run.GetProperty("linearSelected");
   var oldNet=new NativeLinear{Order=lm.GetProperty("order").GetInt32(),Coefficients=A(lm,"coefficients"),Um=lm.GetProperty("Um").GetDouble(),Us=lm.GetProperty("Us").GetDouble(),Ym=lm.GetProperty("Ym").GetDouble(),Ys=lm.GetProperty("Ys").GetDouble()};
   var oldFn=EnsembleMeasure.Create(r,oldNet,ctx.budget);
   var evaluations=new List<object>();var selections=new List<object>();
   foreach(string mode in new[]{"recorded-geometry","frozen-geometry"}){
    Model? best=null;double score=double.PositiveInfinity;
    foreach(var model in models){
     var ev=ctx.Evaluate(model,mode);evaluations.Add(ev);
     var parsed=JsonSerializer.SerializeToElement(ev,options);double sc=Number(parsed.GetProperty("score"));
     if(NativeLinear.Finite(sc)&&sc<score){score=sc;best=model;}
    }
    if(best==null)continue;
    foreach(bool hold in new[]{false,true}){
     var fn=ctx.Measure(best,hold);
     var search=PidSearch.Run(r.Pid,p=>fn(p).Tracking,ctx.budget,_=>{},false,fn);
     string reason=NativeAutoTune.DecisionReason(search.RobustCost,search.BaselineRobust);
     selections.Add(new{mode,holdYaw=hold,best,score,search,reason,recommended=reason=="recommended"?search.Pid:r.Pid,
      oldRecommendationCost=PidSearch.Neighborhood(A(run.GetProperty("result"),"pid"),p=>fn(p).Tracking,false),
      newRecommendationOnOldModel=PidSearch.Neighborhood(search.Pid,p=>oldFn(p).Tracking,false),
      oldRecommendationOnOldModel=PidSearch.Neighborhood(A(run.GetProperty("result"),"pid"),p=>oldFn(p).Tracking,false)});
     Console.WriteLine($"run {index} {mode} holdYaw {hold} order {best.order} gain {best.gain} score {score} PID {string.Join(",",search.Pid)} reason {reason}");
    }
   }
   File.WriteAllText(Root+$"/evaluation-{index}.json",JsonSerializer.Serialize(new{index,original=r.Pid,logged=A(run.GetProperty("result"),"pid"),evaluations,selections},options));
  }
 }
 static double Number(JsonElement e)=>e.ValueKind==JsonValueKind.Number?e.GetDouble():double.Parse(e.GetString()!,System.Globalization.CultureInfo.InvariantCulture);
}
