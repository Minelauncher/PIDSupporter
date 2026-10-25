using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using System.Text.Json;
using System.Text.Json.Serialization;
using PIDSupporter;
internal static class PitchConfoundReview
{
 static double[] A(JsonElement e,string k)=>e.GetProperty(k).EnumerateArray().Select(v=>v.GetDouble()).ToArray();
 internal static void Run(){
  const string root="tests/output/pitch-confounds-20261021";
  var runs=JsonDocument.Parse(File.ReadAllText("tests/output/air-context-20261020/runs.json")).RootElement;
  var cases=JsonDocument.Parse(File.ReadAllText(root+"/diagnosis.json")).RootElement;
  var options=new JsonSerializerOptions{NumberHandling=JsonNumberHandling.AllowNamedFloatingPointLiterals};
  foreach(var item in cases.EnumerateArray()){
   int index=item.GetProperty("index").GetInt32();var run=runs[index-1];var d=run.GetProperty("record");
   var y=A(d,"Y");var u=A(d,"U");var r=A(d,"R");var t=A(d,"T");var integ=A(d,"Integrals");var pid=A(d,"Pid");
   var offset=A(item,"yawAngleOffset");int n=u.Length;double ym=y.Take(n/2).Average(),ys=Math.Sqrt(y.Take(n/2).Average(v=>(v-ym)*(v-ym)));
   var output=new List<object>();
   foreach(var m in item.GetProperty("angleCorrection").EnumerateArray()){
    bool corrected=m.GetProperty("target").GetString()=="yaw-subtracted";
    int order=m.GetProperty("order").GetInt32();
    var net=new NativeLinear{Order=order,Coefficients=A(m,"coefficients"),Um=m.GetProperty("Um").GetDouble(),Us=m.GetProperty("Us").GetDouble(),Ym=m.GetProperty("Ym").GetDouble(),Ys=m.GetProperty("Ys").GetDouble()};
    var fs=new double[2*order];var cs=new double[2*order];
    for(int i=0;i<order;i++){double value=y[i]-(corrected?offset[i]:0);net.Push(fs,value,u[i]);net.Push(cs,value,u[i]);}
    int sw=160;double integral=integ[sw-1],prev=r[sw-1]-y[sw-1];
    var sums=new[]{new double[3],new double[3]};var count=new int[2];
    for(int i=order;i<n;i++){
     double yf=net.Next(fs),yc=net.Next(cs),external=corrected?offset[i]:0,uc=u[i];
     if(i>=sw){double error=r[i]-(yc+external);uc=NativeAutoTune.PidStep(pid,error,prev,ref integral,t[i]-t[i-1],d.GetProperty("Lower").GetDouble(),d.GetProperty("Upper").GetDouble(),false);prev=error;}
     if(i>=n/2){int k=i<n*2/3?0:1;count[k]++;sums[k][0]+=Math.Pow((yf+external-y[i])/ys,2);sums[k][1]+=Math.Pow((yc+external-y[i])/ys,2);sums[k][2]+=Math.Pow((uc-u[i])/net.Us,2);}
     net.Push(fs,yf,u[i]);net.Push(cs,yc,uc);
    }
    var val=sums[0].Select(v=>Math.Sqrt(v/count[0])).ToArray();var final=sums[1].Select(v=>Math.Sqrt(v/count[1])).ToArray();
    output.Add(new{model=m,validation=val,final,score=val.Max()});
   }
   File.WriteAllText(root+$"/ablation-{index}.json",JsonSerializer.Serialize(output,options));
   Console.WriteLine($"pitch {index}: original and yaw-subtracted 36 models evaluated");
  }
 }
}
