using System;using System.IO;using System.Linq;using System.Text.Json;using System.Reflection;using System.Threading;using System.Collections.Generic;using PIDSupporter;
internal static class RollAttributionChecks
{
 static double[] A(JsonElement v)=>v.EnumerateArray().Select(x=>x.GetDouble()).ToArray();
 internal static void Run(string folder){
 var groups=JsonDocument.Parse(File.ReadAllText(Path.Combine(folder,"input.json"))).RootElement;var output=new List<object>();
 var error=typeof(NativeLinear).GetMethod("Errors",BindingFlags.NonPublic|BindingFlags.Instance)!;
 foreach(var g in groups.EnumerateArray()){
  var rs=g.GetProperty("records").EnumerateArray().Select(BandwidthLogReview.Read).ToArray();int count=rs.Length;var pool=A(g.GetProperty("pool"));var budget=new WorkBudget(CancellationToken.None);double bestScore=double.PositiveInfinity;NativeLinear[]? best=null;var candidates=new List<object>();
  double[] Errors(NativeLinear m,int k,int start,int end){var e=(double[])error.Invoke(m,new object[]{rs[k],NativeAutoTune.Validate(rs[k]),start,end,budget})!;var sc=A(g.GetProperty("scales")[k]);e[0]*=pool[1]/sc[1];e[1]*=pool[1]/sc[1];e[2]*=pool[0]/sc[0];return e;}
  foreach(var f in g.GetProperty("fits").EnumerateArray()){
   int order=f.GetProperty("order").GetInt32();var xx=f.GetProperty("x");var x=new double[xx.GetArrayLength(),xx[0].GetArrayLength()];for(int i=0;i<x.GetLength(0);i++)for(int j=0;j<x.GetLength(1);j++)x[i,j]=xx[i][j].GetDouble();
   foreach(var fit in NativeLinear.FitConstrained(x,A(f.GetProperty("y")),order,count,.025,budget)){
    var models=new NativeLinear[count];double score=0;
    for(int k=0;k<count;k++){var coef=new double[2*order+1];Array.Copy(fit.C,coef,2*order);coef[2*order]=fit.C[2*order+k];var mean=A(g.GetProperty("means")[k]);models[k]=new NativeLinear{Order=order,Coefficients=coef,Um=mean[0],Ym=mean[1],Us=pool[0],Ys=pool[1]};score=Math.Max(score,Errors(models[k],k,600,800).Max());}
    candidates.Add(new{order,gain=models[0].DcGain(),score,fit.Loss,fit.Peak});if(score<bestScore){bestScore=score;best=models;}
   }
  }
  if(best==null)throw new Exception("No candidate");var finals=Enumerable.Range(0,count).Select(k=>Errors(best[k],k,800,1200)).ToArray();
  var row=new{variant=g.GetProperty("variant").GetString(),band=g.GetProperty("band").GetString(),indices=g.GetProperty("indices").EnumerateArray().Select(v=>v.GetInt32()).ToArray(),order=best[0].Order,gain=best[0].DcGain(),selection=bestScore,finals,candidates};output.Add(row);Console.WriteLine(JsonSerializer.Serialize(new{row.variant,row.band,row.indices,row.order,row.gain,row.selection,row.finals}));
 }
 File.WriteAllText(Path.Combine(folder,"results.json"),JsonSerializer.Serialize(output,new JsonSerializerOptions{WriteIndented=true,NumberHandling=System.Text.Json.Serialization.JsonNumberHandling.AllowNamedFloatingPointLiterals}));
 }
}
