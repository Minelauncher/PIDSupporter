using System;
using System.Collections.Generic;
using System.Linq;
internal sealed class LegacyExciter
{
 int state=1,bitTicks=8,ticks,ampTicks,lastSpectrum;
 double bit=1,hpf,lastRaw,amp=.3,uamp=.03;
 internal double Injection;
 internal List<object> Updates=new List<object>();
 // Port of legacy FritTuningTab.ApplyExcitation; fixed 30s wrapper, common SP cap.
 internal double Next(int k,double[] u,double[] y,double[] r,double cap,bool hybrid)
 {
  if(k>=256 && k-lastSpectrum>=240){Spectrum(k,y,r);lastSpectrum=k;}
  ampTicks++;
  if(ampTicks>=80 && k>=80)
  {
   double sat=u.Skip(k-80).Take(80).Count(v=>Math.Abs(v)>=.98)/80.0;
   double scale=sat>.30?.7:sat<.10?1.4:1;
   amp=Math.Max(.01,Math.Min(10,amp*scale));uamp=Math.Max(.005,Math.Min(1,uamp*scale));
   ampTicks=0;Updates.Add(new{sample=k,kind="amplitude",sat,amp,uamp});
  }
  if(++ticks>=bitTicks){int next=((state>>9)^(state>>6))&1;state=((state<<1)|next)&1023;bit=next==1?1:-1;ticks=0;}
  hpf=.9984*(hpf+bit-lastRaw);lastRaw=bit;
  double headroom=.5*Math.Max(0,1-Math.Abs(k==0?0:u[k-1]));
  Injection=hybrid?Math.Max(-headroom,Math.Min(headroom,uamp*hpf)):0;
  return Math.Max(-cap,Math.Min(cap,amp*hpf));
 }
 void Spectrum(int n,double[] y,double[] r)
 {
  const int size=256;var yy=new double[129];var rr=new double[129];var re=new double[129];var im=new double[129];int count=0;
  for(int start=0;start+size<=n;start+=128)
  {
   double ym=0,rm=0;for(int i=0;i<size;i++){ym+=y[start+i]/size;rm+=r[start+i]/size;}
   for(int bin=0;bin<129;bin++)
   {
    double yr=0,yi=0,zr=0,zi=0;
    for(int i=0;i<size;i++)
    {double w=.5-.5*Math.Cos(2*Math.PI*i/255),c=w*Math.Cos(2*Math.PI*bin*i/size),s=-w*Math.Sin(2*Math.PI*bin*i/size);yr+=(y[start+i]-ym)*c;yi+=(y[start+i]-ym)*s;zr+=(r[start+i]-rm)*c;zi+=(r[start+i]-rm)*s;}
    yy[bin]+=yr*yr+yi*yi;rr[bin]+=zr*zr+zi*zi;re[bin]+=yr*zr+yi*zi;im[bin]+=yi*zr-yr*zi;
   }count++;
  }
  for(int b=0;b<129;b++){yy[b]/=count;rr[b]/=count;re[b]/=count;im[b]/=count;}
  double Band(int lo,int hi)
  {double sum=0,weight=0;for(int b=lo;b<hi;b++){if(yy[b]<1e-15||rr[b]<1e-15)continue;double coh=Math.Min(1,(re[b]*re[b]+im[b]*im[b])/(yy[b]*rr[b]));sum+=coh*Math.Sqrt(Math.Pow(1-re[b]/rr[b],2)+Math.Pow(im[b]/rr[b],2));weight+=coh;}return weight>1e-9?sum/weight:0;}
  double width=1/(256*.05),low=Band(Math.Max(1,(int)Math.Round(.05/width)),(int)Math.Round(.5/width)),mid=Band((int)Math.Round(.5/width),(int)Math.Round(5/width)),high=Band((int)Math.Round(5/width),128);
  bitTicks=low>=mid&&low>=high?64:mid>=high?8:2;
  Updates.Add(new{sample=n,kind="spectrum",low,mid,high,bitTicks});
 }
}
