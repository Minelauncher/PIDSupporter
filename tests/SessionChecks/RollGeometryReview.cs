using System;using System.IO;using System.Linq;using System.Text.Json;using System.Threading;using System.Collections.Generic;using PIDSupporter;
internal static class RollGeometryReview {
 static double[] A(JsonElement e)=>e.EnumerateArray().Select(x=>x.GetDouble()).ToArray();
 struct Q {
  public double x,y,z,w;public Q(double[] v){x=v[0];y=v[1];z=v[2];w=v[3];}
  public double Y=>-Math.Atan2(2*(x*y+w*z),1-2*(x*x+z*z))*180/Math.PI;
  public void Step(double p,double q,double r){double k=.025*Math.PI/180;p*=k;q*=k;r*=k;double t=Math.Sqrt(p*p+q*q+r*r),f=t<1e-12?.5:Math.Sin(t/2)/t,d=Math.Cos(t/2);p*=f;q*=f;r*=f;double X=w*p+x*d+y*r-z*q,Y=w*q-x*r+y*d+z*p,Z=w*r+x*q-y*p+z*d,W=w*d-x*p-y*q-z*r;double n=Math.Sqrt(X*X+Y*Y+Z*Z+W*W);x=X/n;y=Y/n;z=Z/n;w=W/n;}
 }
 sealed class Entry {public NativeLinear m=null!;public string mode="";public double[][] forecast=null!;public double score;public double[] validation=null!,final=null!;}
 sealed class Context {
  public TuneRecord r=null!;public double[][] rates=null!;public double[] quat=null!;public double ys,us;
  public (double[],double[]) Sim(Entry e,double[] pid,bool closed,double[]? reference=null){
   int start=600,n=reference==null?1200:reference.Length+600;var yy=new double[n];var uu=new double[n];Array.Copy(r.Y,yy,600);Array.Copy(r.U,uu,600);var st=new double[2*e.m.Order];
   for(int j=600-e.m.Order;j<600;j++)e.m.Push(st,e.mode=="angle"?r.Y[j]:-rates[j][2],r.U[j]);
   var q=new Q(quat);double wx=rates[599][0],wy=rates[599][1],wz=rates[599][2],integral=r.Integrals[599],prev=r.R[599]-r.Y[599];
   for(int i=start;i<n;i++){
    double value=e.m.Next(st),y=value;
    if(e.mode!="angle") {double nx=e.forecast[0][0]+e.forecast[0][1]*(wx-e.forecast[0][0]),ny=e.forecast[1][0]+e.forecast[1][1]*(wy-e.forecast[1][0]);q.Step((wx+nx)/2,(wy+ny)/2,(wz-value)/2);wx=nx;wy=ny;wz=-value;y=q.Y;}
    double target=reference==null?r.R[i]:reference[i-600],err=target-y,u=closed?NativeAutoTune.PidStep(pid,err,prev,ref integral,.025,-1,1,false):r.U[i];prev=err;
    if(!double.IsFinite(y)||!double.IsFinite(value)||Math.Abs(value)>1e6){for(int j=i;j<n;j++){yy[j]=1e6;uu[j]=1;}break;}
    yy[i]=y;uu[i]=u;e.m.Push(st,value,u);
   }return(yy,uu);
  }
  public void Eval(Entry e){var f=Sim(e,r.Pid,false);var c=Sim(e,r.Pid,true);double[] Errors(int lo,int hi)=>new[]{Math.Sqrt(Enumerable.Range(lo,hi-lo).Average(i=>Math.Pow((f.Item1[i]-r.Y[i])/ys,2))),Math.Sqrt(Enumerable.Range(lo,hi-lo).Average(i=>Math.Pow((c.Item1[i]-r.Y[i])/ys,2))),Math.Sqrt(Enumerable.Range(lo,hi-lo).Average(i=>Math.Pow((c.Item2[i]-r.U[i])/us,2)))};e.validation=Errors(600,800);e.final=Errors(800,1200);e.score=e.validation.Max();}
  public PidSearch.Motion Cost(Entry e,double[] pid){double worst=0,sw=0,ew=0,delta=Math.Max(.1,ys);foreach(int sign in new[]{-1,1}) {var reference=Enumerable.Range(0,600).Select(i=>r.Y[599]+sign*delta*(1-Math.Exp(-i*.025/1.25))).ToArray();var s=Sim(e,pid,true,reference);double cost=0,slew=0,effort=0;for(int i=600;i<1200;i++){cost+=Math.Pow((s.Item1[i]-reference[i-600])/delta,2);slew+=Math.Pow((s.Item2[i]-s.Item2[i-1])/.05,2);effort+=Math.Pow(s.Item2[i]/2,2);}worst=Math.Max(worst,cost/600);sw=Math.Max(sw,slew/600);ew=Math.Max(ew,effort/600);}return new PidSearch.Motion{Tracking=worst,Slew=sw,Effort=ew};}
 }
 public static void Run(string folder){var input=JsonDocument.Parse(File.ReadAllText(folder+"/input.json")).RootElement;var output=new List<object>();var budget=new WorkBudget(CancellationToken.None);
 foreach(var row in input.EnumerateArray()){
  var ctx=new Context{r=BandwidthLogReview.Read(row.GetProperty("record")),rates=row.GetProperty("rates").EnumerateArray().Select(A).ToArray(),quat=A(row.GetProperty("quaternion"))};ctx.ys=Math.Sqrt(ctx.r.Y.Take(560).Average(v=>Math.Pow(v-ctx.r.Y.Take(560).Average(),2)));ctx.us=Math.Sqrt(ctx.r.U.Take(560).Average(v=>Math.Pow(v-ctx.r.U.Take(560).Average(),2)));
  var entries=new List<Entry>();
  foreach(var g in row.GetProperty("groups").EnumerateArray())foreach(var f in g.GetProperty("fits").EnumerateArray()){
   int order=f.GetProperty("order").GetInt32();var xx=f.GetProperty("x");var x=new double[xx.GetArrayLength(),xx[0].GetArrayLength()];for(int i=0;i<x.GetLength(0);i++)for(int j=0;j<x.GetLength(1);j++)x[i,j]=xx[i][j].GetDouble();
   foreach(var fit in NativeLinear.FitConstrained(x,A(f.GetProperty("y")),order,1,.025,budget)){
    var m=new NativeLinear{Order=order,Coefficients=fit.C,Um=g.GetProperty("um").GetDouble(),Us=g.GetProperty("us").GetDouble(),Ym=g.GetProperty("ym").GetDouble(),Ys=g.GetProperty("ys").GetDouble()};
    if(g.GetProperty("mode").GetString()=="angle"){var e=new Entry{m=m,mode="angle"};ctx.Eval(e);entries.Add(e);}else foreach(var fc in row.GetProperty("forecasts").EnumerateArray()){var e=new Entry{m=m,mode=fc.GetProperty("mode").GetString()!,forecast=fc.GetProperty("values").EnumerateArray().Select(A).ToArray()};ctx.Eval(e);entries.Add(e);}
   }
  }
  var results=new List<object>();foreach(var group in entries.GroupBy(e=>e.mode=="angle"?"angle":"rate_geometry")){
   var best=group.OrderBy(e=>e.score).First();var fn=new Func<double[],PidSearch.Motion>(p=>ctx.Cost(best,p));var search=PidSearch.Run(ctx.r.Pid,p=>fn(p).Tracking,budget,_=>{},false,fn);string reason=best.m.DcGain()<=0?"negative_gain":NativeAutoTune.DecisionReason(search.RobustCost,search.BaselineRobust);
   var sim=ctx.Sim(best,ctx.r.Pid,true);var saveY=(double[])ctx.r.Y.Clone();var saveU=(double[])ctx.r.U.Clone();for(int i=600;i<1200;i++){ctx.r.Y[i]=9999;ctx.r.U[i]=-9999;}var poisoned=ctx.Sim(best,ctx.r.Pid,true);ctx.r.Y=saveY;ctx.r.U=saveU;double leakage=sim.Item1.Zip(poisoned.Item1,(a,b)=>Math.Abs(a-b)).Concat(sim.Item2.Zip(poisoned.Item2,(a,b)=>Math.Abs(a-b))).Max();if(leakage!=0)throw new Exception("Future observation leakage");
   var result=new{mode=group.Key,forecast=best.mode,order=best.m.Order,gain=best.m.DcGain(),best.validation,best.final,best.score,search,reason,leakage,recommended=reason=="recommended"?search.Pid:ctx.r.Pid,closedY=sim.Item1,closedU=sim.Item2,candidates=group.Select(e=>new{e.mode,e.m.Order,gain=e.m.DcGain(),e.score,e.validation,e.final}).ToArray()};results.Add(result);Console.WriteLine(JsonSerializer.Serialize(new{name=row.GetProperty("name").GetString(),result.mode,result.forecast,result.order,result.gain,result.validation,result.final,result.recommended,result.reason}));
  }output.Add(new{name=row.GetProperty("name").GetString(),results});File.WriteAllText(folder+"/results.json",JsonSerializer.Serialize(output,new JsonSerializerOptions{WriteIndented=true,NumberHandling=System.Text.Json.Serialization.JsonNumberHandling.AllowNamedFloatingPointLiterals}));
 }
 }
}
