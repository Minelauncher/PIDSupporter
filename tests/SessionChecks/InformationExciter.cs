using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Diagnostics;
using System.Text.Json;
using PIDSupporter;
internal sealed class InformationExciter
{
 readonly double[,] tones=new double[600,6];readonly double amplitude;readonly double[] controller;
 readonly double[] gain={1,1,1,1,1,1},from={1,1,1,1,1,1};int ramp=-100;
 internal List<object> Updates=new List<object>();
 internal InformationExciter(double[] pid,double[] phases,double amp)
 {
  amplitude=amp;controller=(double[])pid.Clone();
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
 internal double Next(int k,double[] u,double[] y,double[] r,double integral,double previous)
 {
  if(k==100 || k==200)
  {
   var future=new double[100][];for(int i=0;i<100;i++){future[i]=new double[6];for(int b=0;b<6;b++)future[i][b]=tones[k+i,b];}
   string path="tests/output/information-sp-iv-20261020/design-request.json";
   File.WriteAllText(path,JsonSerializer.Serialize(new{u=u.Take(k),y=y.Take(k),r=r.Take(k),pid=controller,integral,previous,amplitude,gain,tones=future}));
   var psi=new ProcessStartInfo("research_lti/.venv_ren/Scripts/python.exe"){UseShellExecute=false,CreateNoWindow=true,RedirectStandardError=true};
   psi.ArgumentList.Add("research_lti/sequential_information.py");psi.ArgumentList.Add(path);
   using(var process=Process.Start(psi)!){string err=process.StandardError.ReadToEnd();process.WaitForExit();if(process.ExitCode!=0)throw new Exception(err);}
   var result=JsonDocument.Parse(File.ReadAllText(path+".out")).RootElement;
   var g=result.GetProperty("gain").EnumerateArray().Select(x=>x.GetDouble()).ToArray();
   for(int b=0;b<6;b++){from[b]=gain[b];gain[b]=g[b];}
   ramp=k;Updates.Add(new{sample=k,result=result.Clone()});
  }
  double mix=Math.Min(1,Math.Max(0,(k-ramp)/20.0));mix=.5*(1-Math.Cos(Math.PI*mix));double value=0;
  for(int b=0;b<6;b++)value+=tones[k,b]*(from[b]+mix*(gain[b]-from[b]));
  return NativeAutoTune.Clamp(value,-amplitude,amplitude);
 }
}
