using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using System.Text.Json;
using System.Text.Json.Serialization;
using System.Threading;
using PIDSupporter;
internal static class QuaternionModelReview
{
 const string Root="tests/output/quaternion-models-20261021";
 static double[] A(JsonElement e,string k)=>e.GetProperty(k).EnumerateArray().Select(v=>v.GetDouble()).ToArray();
 static double[][] Matrix(JsonElement e,string k)=>e.GetProperty(k).EnumerateArray().Select(row=>row.EnumerateArray().Select(v=>v.GetDouble()).ToArray()).ToArray();
 struct Quat {
  internal double x,y,z,w;
  internal Quat(double[] a){x=a[0];y=a[1];z=a[2];w=a[3];}
  internal void Advance(double[] before,double[] after,double dt){
   double k=.5*dt*Math.PI/180,rx=k*(before[0]+after[0]),ry=k*(before[1]+after[1]),rz=k*(before[2]+after[2]);
   double theta=Math.Sqrt(rx*rx+ry*ry+rz*rz),f=theta<1e-12?.5:Math.Sin(theta/2)/theta;
   double dx=rx*f,dy=ry*f,dz=rz*f,dw=Math.Cos(theta/2);
   double nx=w*dx+x*dw+y*dz-z*dy,ny=w*dy-x*dz+y*dw+z*dx,nz=w*dz+x*dy-y*dx+z*dw,nw=w*dw-x*dx-y*dy-z*dz;
   double norm=Math.Sqrt(nx*nx+ny*ny+nz*nz+nw*nw);x=nx/norm;y=ny/norm;z=nz/norm;w=nw/norm;
  }
  internal double Pitch=>Math.Asin(Math.Clamp(2*(w*x-y*z),-1,1))*180/Math.PI;
  internal double[] Angles()=>new[]{Pitch,Math.Atan2(2*(x*z+w*y),1-2*(x*x+y*y))*180/Math.PI,Math.Atan2(2*(x*y+w*z),1-2*(x*x+z*z))*180/Math.PI};
  internal double Distance(Quat b)=>2*Math.Acos(Math.Clamp(Math.Abs(x*b.x+y*b.y+z*b.z+w*b.w),0,1))*180/Math.PI;
 }
 sealed class Model {
  public string mode {get;set;}="";public int order {get;set;}public double ridge {get;set;}public double poleRadius {get;set;}
  public double[][] coefficients {get;set;}=null!;public double Um {get;set;}public double Us {get;set;}
  public double[] Means {get;set;}=null!;public double[] Scales {get;set;}=null!;public double[] gains {get;set;}=null!;
  internal void Next(double[] state,double[] output){for(int a=0;a<3;a++){double value=coefficients[a][4*order];for(int j=0;j<state.Length;j++)value+=state[j]*coefficients[a][j];output[a]=Means[a]+Scales[a]*value;}}
  internal void Push(double[] state,double[] rate,double u){for(int a=0;a<4;a++){int s=a*order;for(int j=order-1;j>0;j--)state[s+j]=state[s+j-1];state[s]=a<3?(rate[a]-Means[a])/Scales[a]:(u-Um)/Us;}}
 }
 sealed class Evaluation {
  public Model model {get;set;}=null!;public double score {get;set;}public double[] validation {get;set;}=null!;public double[] final {get;set;}=null!;
  public double[] free {get;set;}=null!;public double[] closed {get;set;}=null!;public double[] input {get;set;}=null!;
  public double[][] freeAngles {get;set;}=null!;public double[][] closedAngles {get;set;}=null!;
 }
 sealed class Context {
  internal TuneRecord r=null!;internal double[][] rates=null!,quats=null!;internal double dt,ys;
  internal WorkBudget budget=new WorkBudget(CancellationToken.None);
  internal double[] State(Model m,int end){var s=new double[4*m.order];for(int i=end-m.order;i<end;i++)m.Push(s,rates[i],r.U[i]);return s;}
  internal Evaluation Evaluate(Model m){
   int n=r.U.Length,o=m.order;var fs=State(m,o);var cs=State(m,o);var fq=new Quat(quats[o-1]);var cq=fq;
   var fw=(double[])rates[o-1].Clone();var cw=(double[])fw.Clone();var fn=new double[3];var cn=new double[3];
   var sums=new[]{new double[6],new double[6]};var counts=new int[2];int sw=(int)Math.Round(4/dt);
   double integral=r.Integrals[sw-1],prev=r.R[sw-1]-r.Y[sw-1];
   var ev=new Evaluation{model=m,free=new double[n],closed=new double[n],input=new double[n],freeAngles=new double[n][],closedAngles=new double[n][]};
   for(int i=0;i<o;i++){ev.free[i]=ev.closed[i]=r.Y[i];ev.freeAngles[i]=ev.closedAngles[i]=new Quat(quats[i]).Angles();}
   for(int i=o;i<n;i++){
    m.Next(fs,fn);m.Next(cs,cn);
    if(fn.Concat(cn).Any(v=>!NativeLinear.Finite(v)||Math.Abs(v)>1e6)){
     ev.score=double.PositiveInfinity;ev.validation=ev.final=Enumerable.Repeat(double.PositiveInfinity,6).ToArray();return ev;}
    double h=r.T[i]-r.T[i-1];fq.Advance(fw,fn,h);cq.Advance(cw,cn,h);double yf=fq.Pitch,yc=cq.Pitch,u=r.U[i];
    if(i>=sw){double error=r.R[i]-yc;u=NativeAutoTune.PidStep(r.Pid,error,prev,ref integral,h,r.Lower,r.Upper,false);prev=error;}
    ev.free[i]=yf;ev.closed[i]=yc;ev.input[i]=u;ev.freeAngles[i]=fq.Angles();ev.closedAngles[i]=cq.Angles();
    if(i>=n/2){int k=i<n*2/3?0:1;counts[k]++;sums[k][0]+=Math.Pow((yf-r.Y[i])/ys,2);sums[k][1]+=Math.Pow((yc-r.Y[i])/ys,2);sums[k][2]+=Math.Pow((u-r.U[i])/m.Us,2);
     sums[k][3]+=fn.Zip(rates[i],(a,b)=>(a-b)*(a-b)).Sum();sums[k][4]+=Math.Pow(fq.Distance(new Quat(quats[i])),2);sums[k][5]+=Math.Pow(cq.Distance(new Quat(quats[i])),2);}
    m.Push(fs,fn,r.U[i]);m.Push(cs,cn,u);Array.Copy(fn,fw,3);Array.Copy(cn,cw,3);
   }
   ev.validation=sums[0].Select(v=>Math.Sqrt(v/counts[0])).ToArray();ev.final=sums[1].Select(v=>Math.Sqrt(v/counts[1])).ToArray();ev.score=ev.validation.Take(3).Max();return ev;
  }
  internal Func<double[],PidSearch.Motion> Measure(Model m){
   int n=r.U.Length;var warm=State(m,n);double delta=Math.Max(.5*r.Amplitude,.1*ys),bias=r.U.Skip(n-(int)Math.Round(1/dt)).Average();
   var cache=new Dictionary<(double,double,double),PidSearch.Motion>();
   return candidate=>{
    var key=(candidate[0],candidate[1],candidate[2]);if(cache.TryGetValue(key,out var got))return got;
    int steps=(int)Math.Round(15/dt);double worst=0,slewWorst=0,effortWorst=0;
    foreach(int sign in new[]{-1,1}){
     var state=(double[])warm.Clone();var q=new Quat(quats[n-1]);var prevW=(double[])rates[n-1].Clone();var nextW=new double[3];
     double integral=r.InitialIntegral,prev=r.InitialError,last=r.U[n-1],cost=0,slew=0,effort=0;
     for(int k=0;k<steps;k++){
      if((k&31)==0)budget.Check();m.Next(state,nextW);
      if(nextW.Any(v=>!NativeLinear.Finite(v)||Math.Abs(v)>1e6)){cost=1e12*steps;break;}
      q.Advance(prevW,nextW,dt);double y=q.Pitch,target=r.Y[n-1]+sign*delta*(1-Math.Exp(-k*dt/(2-1.5*r.Speed))),error=target-y;
      double u=NativeAutoTune.PidStep(candidate,error,prev,ref integral,dt,r.Lower,r.Upper,false);prev=error;double c=Math.Pow(error/delta,2);
      if(!NativeLinear.Finite(c)||Math.Abs(y-r.Y[n-1])>20*delta){cost=1e12*steps;break;}
      cost+=c;slew+=Math.Pow((u-last)/((r.Upper-r.Lower)*dt),2);effort+=Math.Pow((u-bias)/(r.Upper-r.Lower),2);
      m.Push(state,nextW,u);Array.Copy(nextW,prevW,3);last=u;
     }
     worst=Math.Max(worst,cost/steps);slewWorst=Math.Max(slewWorst,slew/steps);effortWorst=Math.Max(effortWorst,effort/steps);
    }
    var value=new PidSearch.Motion{Tracking=worst,Slew=slewWorst,Effort=effortWorst};cache[key]=value;return value;
   };
  }
 }
 internal static void Run(){
  var options=new JsonSerializerOptions{NumberHandling=JsonNumberHandling.AllowNamedFloatingPointLiterals};
  var runs=JsonDocument.Parse(File.ReadAllText("tests/output/air-context-20261020/runs.json")).RootElement;
  var data=JsonDocument.Parse(File.ReadAllText(Root+"/models.json")).RootElement;
  foreach(var datum in data.EnumerateArray()){
   int index=datum.GetProperty("index").GetInt32();var run=runs[index-1];var d=run.GetProperty("record");
   var r=new TuneRecord{U=A(d,"U"),Y=A(d,"Y"),R=A(d,"R"),T=A(d,"T"),Pid=A(d,"Pid"),Integrals=A(d,"Integrals"),InjectedSP=A(d,"InjectedSP"),BaseSP=A(d,"BaseSP"),Speed=run.GetProperty("start").GetProperty("speed").GetDouble(),Amplitude=d.GetProperty("Amplitude").GetDouble(),Lower=d.GetProperty("Lower").GetDouble(),Upper=d.GetProperty("Upper").GetDouble(),InitialIntegral=d.GetProperty("InitialIntegral").GetDouble(),InitialError=d.GetProperty("InitialError").GetDouble()};
   double mean=r.Y.Take(r.Y.Length/2).Average();var ctx=new Context{r=r,dt=NativeAutoTune.Validate(r),ys=Math.Sqrt(r.Y.Take(r.Y.Length/2).Average(v=>(v-mean)*(v-mean))),rates=Matrix(datum,"rates"),quats=Matrix(datum,"quaternions")};
   var reference=Matrix(datum,"pythonReplay");var q=new Quat(ctx.quats[0]);double quaternionCheck=0;
   for(int i=1;i<r.U.Length;i++){q.Advance(ctx.rates[i-1],ctx.rates[i],r.T[i]-r.T[i-1]);quaternionCheck=Math.Max(quaternionCheck,q.Distance(new Quat(reference[i])));}
   if(quaternionCheck>1e-4)throw new Exception("Quaternion integration mismatch");
   var models=datum.GetProperty("models").Deserialize<Model[]>()!;var evaluations=models.Select(ctx.Evaluate).ToArray();
   File.WriteAllText(Root+$"/replay-{index}.json",JsonSerializer.Serialize(new{index,quaternionCheck,evaluations},options));
   var selections=new List<object>();
   foreach(var group in evaluations.GroupBy(e=>e.model.mode)){
    var best=group.Where(e=>NativeLinear.Finite(e.score)).OrderBy(e=>e.score).First();
    // Verify forecasts cannot consume other-axis observations after the warmup.
    var saveRates=ctx.rates;var saveQuats=ctx.quats;ctx.rates=ctx.rates.Select((a,i)=>i>=160?new[]{12345.0,-12345.0,9999.0}:a).ToArray();ctx.quats=ctx.quats.Select((a,i)=>i>=160?new[]{0.0,0.0,0.0,1.0}:a).ToArray();
    var check=ctx.Evaluate(best.model);double leakDifference=best.free.Zip(check.free,(a,b)=>Math.Abs(a-b)).Concat(best.closed.Zip(check.closed,(a,b)=>Math.Abs(a-b))).Concat(best.input.Zip(check.input,(a,b)=>Math.Abs(a-b))).Max();
    ctx.rates=saveRates;ctx.quats=saveQuats;if(leakDifference!=0)throw new Exception("Future observation leakage");
    var fn=ctx.Measure(best.model);var search=PidSearch.Run(r.Pid,p=>fn(p).Tracking,ctx.budget,_=>{},false,fn);var reason=NativeAutoTune.DecisionReason(search.RobustCost,search.BaselineRobust);
    selections.Add(new{mode=group.Key,best=best.model,validation=best.validation,final=best.final,leakDifference,search,reason,recommended=reason=="recommended"?search.Pid:r.Pid,loggedRecommendationCost=PidSearch.Neighborhood(A(run.GetProperty("result"),"pid"),p=>fn(p).Tracking,false)});
    Console.WriteLine($"run {index} {group.Key} score {best.score} gain {best.model.gains[0]} order {best.model.order} final {string.Join(",",best.final)} PID {string.Join(",",search.Pid)} {reason}");
   }
   File.WriteAllText(Root+$"/selection-{index}.json",JsonSerializer.Serialize(new{index,original=r.Pid,logged=A(run.GetProperty("result"),"pid"),quaternionCheck,selections},options));
  }
 }
}
