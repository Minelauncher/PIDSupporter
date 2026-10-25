using System;
using System.Collections.Generic;
using System.Linq;
using PIDSupporter;
internal sealed class AdaptiveExciter
{
 readonly double[,] tones=new double[600,6];readonly double amplitude;
 readonly double[] gain={1,1,1,1,1,1},from={1,1,1,1,1,1};int ramp=-100;
 internal List<object> Updates=new List<object>();
 internal AdaptiveExciter(double[] pid,double[] phases,double amp)
 {
  amplitude=amp;
  double Si(double x)=>Math.Abs(x)<1e-10?1:Math.Sin(x)/x;
  for(int j=0;j<60;j++)
  {
   int band=j<3?0:j<6?1:j<12?2:j<24?3:j<40?4:5;
   double w=2*Math.PI*(j+1)/30,react=w*pid[2]-(pid[1]>=250?0:1/(w*Math.Max(.01,pid[1])));
   double weight=Math.Max(.5*Math.Max(0,Math.Min(1,(.4-(j+1)/30.0)/.2)),1/Math.Sqrt(1+react*react));
   double center=weight*Math.Sin(phases[j])*(-(Si(w)+.5*(Si(w-Math.PI)+Si(w+Math.PI))))/29;
   for(int k=0;k<600;k++)
   {double t=k*.05,edge=Math.Min(t,30-t),env=edge>=1?1:.5*(1-Math.Cos(Math.PI*edge));tones[k,band]+=amp/20*env*(weight*Math.Sin(w*t+phases[j])-center);}
  }
 }
 internal double Next(int k,double[] u,double[] y)
 {
  // Only completed past samples; adapt at 5 and 10 seconds, then hold through validation.
  if(k==100 || k==200)
  {
   int lo=k-100;double um=0,ym=0,uv=0,yv=0;int sat=0;
   for(int i=lo;i<k;i++){um+=u[i]/100;ym+=y[i]/100;if(Math.Abs(u[i])>=.999999)sat++;}
   for(int i=lo;i<k;i++){uv+=(u[i]-um)*(u[i]-um)/100;yv+=(y[i]-ym)*(y[i]-ym)/100;}
   var scores=new double[6];var counts=new int[6];
   for(int j=0;j<60;j++)
   {
    int b=j<3?0:j<6?1:j<12?2:j<24?3:j<40?4:5;double ur=0,ui=0,yr=0,yi=0,w=2*Math.PI*(j+1)/30;
    for(int i=lo;i<k;i++){double c=Math.Cos(w*i*.05),s=Math.Sin(w*i*.05);ur+=(u[i]-um)*c;ui+=(u[i]-um)*s;yr+=(y[i]-ym)*c;yi+=(y[i]-ym)*s;}
    scores[b]+=Math.Sqrt((ur*ur+ui*ui)/Math.Max(uv,1e-12)*(yr*yr+yi*yi)/Math.Max(yv,1e-12))/10000;counts[b]++;
   }
   for(int b=0;b<6;b++)scores[b]/=counts[b];
   double median=scores.Skip(2).OrderBy(x=>x).Skip(1).Take(2).Average();
   for(int b=0;b<6;b++)
   {
    from[b]=gain[b];
    if(sat>5)gain[b]=Math.Max(.5,gain[b]*.8);
    else if(b>=2 && scores[b]<median)gain[b]=Math.Min(2,gain[b]*1.25);
   }
   ramp=k;Updates.Add(new{sample=k,scores,saturation=sat/100.0,gains=(double[])gain.Clone()});
  }
  double mix=Math.Min(1,Math.Max(0,(k-ramp)/20.0));mix=.5*(1-Math.Cos(Math.PI*mix));double r=0;
  for(int b=0;b<6;b++)r+=tones[k,b]*(from[b]+mix*(gain[b]-from[b]));
  return NativeAutoTune.Clamp(r,-amplitude,amplitude);
 }
}
