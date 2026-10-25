using System;
using System.Linq;
using System.Numerics;
using System.Collections.Generic;
using PIDSupporter;
namespace PIDSupporter { internal sealed class EmpiricalFrequencyGate {
 double[] den=Array.Empty<double>(),num=Array.Empty<double>(),w=Array.Empty<double>();
 Complex[] z=Array.Empty<Complex>(),g=Array.Empty<Complex>();
 internal int Checks,Rejected;
 internal double[] Frequencies => hz;
 internal double[] Radii => w;
 private double[] hz=Array.Empty<double>();
 private double dt;
 // Empirical disks at measured harmonics only, NOT full-band stability certification.
 internal EmpiricalFrequencyGate(NativeLinear m,IReadOnlyList<TuneRecord> records,WorkBudget budget){
  dt=NativeAutoTune.Validate(records[records.Count-1]);den=new double[m.Order+1];num=new double[m.Order+1];den[0]=1;
  for(int j=0;j<m.Order;j++){den[j+1]=-m.Coefficients[j];num[j+1]=m.Coefficients[m.Order+j]*m.Ys/m.Us;}
  hz=Enumerable.Range(0,10).Select(j=>.2+.4*j).Where(f=>f<.5/dt).ToArray();
  z=hz.Select(f=>Complex.Exp(new Complex(0,-2*Math.PI*f*dt))).ToArray();g=z.Select(v=>Eval(num,v)/Eval(den,v)).ToArray();w=new double[hz.Length];
  if(hz.Length==0)throw new ArgumentException("No usable frequency band.");
  foreach(var r in records){
   double h=NativeAutoTune.Validate(r);if(Math.Abs(h-dt)>dt*.01)throw new ArgumentException("Different sampling intervals in history.");
   var instrument=r.InstrumentSP; if(instrument==null||instrument.Length!=r.U.Length)throw new ArgumentException("Independent excitation history is missing.");
   int count=(int)Math.Round(5/dt);int blocks=(r.U.Length*2/3)/count;
   if(blocks<1)throw new ArgumentException("Insufficient frequency history.");
   for(int block=0;block<blocks;block++)for(int j=0;j<hz.Length;j++){
    budget.Check();Complex iz=Fourier(instrument,block*count,count,hz[j],dt),iu=Fourier(r.U,block*count,count,hz[j],dt),iy=Fourier(r.Y,block*count,count,hz[j],dt);
    Complex zu=Complex.Conjugate(iz)*iu,zy=Complex.Conjugate(iz)*iy;
    if(!NativeLinear.Finite(zu.Magnitude)||zu.Magnitude<=1e-24)throw new ArgumentException("Excitation-linked input cannot be resolved.");
    double error=(zy/zu-g[j]).Magnitude;if(!NativeLinear.Finite(error))throw new ArgumentException("Frequency response cannot be estimated.");
    w[j]=Math.Max(w[j],error);
   }
  }
 }
 // scipy.signal.csd defaults for one 5-second segment: constant detrend, periodic Hann.
 // Single-window cross-spectrum ratios cancel the instrument algebraically;
 // these are empirical discrepancy samples, not independent IV confidence bounds.
 private Complex Fourier(double[] values,int start,int count,double frequency,double h){
  if(start+count>values.Length)throw new ArgumentException("Incomplete frequency window.");
  double mean=0;for(int k=0;k<count;k++)mean+=values[start+k];mean/=count;
  Complex sum=0;for(int k=0;k<count;k++)sum+=(values[start+k]-mean)*(.5-.5*Math.Cos(2*Math.PI*k/count))*Complex.Exp(new Complex(0,-2*Math.PI*frequency*h*k));return sum;
 }
 Complex Eval(double[] c,Complex z){Complex v=0;for(int j=c.Length-1;j>=0;j--)v=v*z+c[j];return v;}
 double[] Conv(double[] a,double[] b){var c=new double[a.Length+b.Length-1];for(int i=0;i<a.Length;i++)for(int j=0;j<b.Length;j++)c[i+j]+=a[i]*b[j];return c;}
 // Schur recursion for real descending-power characteristic coefficients.
 internal bool Stable(double[] input){
  var a=(double[])input.Clone();
  for(int n=a.Length-1;n>0;n--){
   double scale=Math.Abs(a[0]);if(!NativeLinear.Finite(scale)||scale==0)return false;
   for(int j=0;j<=n;j++)a[j]/=scale;
   if(Math.Abs(a[n])>=Math.Abs(a[0]))return false;
   var b=new double[n];for(int j=0;j<n;j++)b[j]=a[0]*a[j]-a[n]*a[n-j];a=b;
  }return NativeLinear.Finite(a[0])&&a[0]!=0;
 }
 internal bool Accept(double[] p){
  Checks++;double kp=p[0],ti=p[1],td=p[2],h=dt;
  double[] cn=ti>=250?new[]{kp*(1+td/h),-kp*td/h}:new[]{kp*(1+h/ti+td/h),kp*(-1-2*td/h),kp*td/h};
  double[] cd=ti>=250?new[]{1.0}:new[]{1.0,-1.0};
  var d=Conv(den,cd);var v=Conv(num,cn);var polynomial=new double[Math.Max(d.Length,v.Length)];for(int j=0;j<d.Length;j++)polynomial[j]+=d[j];for(int j=0;j<v.Length;j++)polynomial[j]+=v[j];
  if(!Stable(polynomial)){Rejected++;return false;}
  for(int j=0;j<z.Length;j++){
   Complex c=Eval(cn,z[j])/Eval(cd,z[j]);double rho=w[j]*(c/(1+g[j]*c)).Magnitude;
   if(!NativeLinear.Finite(rho)||rho>=1){Rejected++;return false;}
  }return true;
 }
}

}
