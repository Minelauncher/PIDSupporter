using System;using System.IO;using System.Linq;using System.Text.Json;using System.Reflection;using System.Threading;using System.Collections.Generic;using PIDSupporter;
internal static class PostRidgeReview
{
 static double[] A(JsonElement v)=>v.EnumerateArray().Select(x=>x.GetDouble()).ToArray();
 internal static void Run(string log,string folder)
 {
  var allRecords=File.ReadLines(log).Select(l=>JsonDocument.Parse(l)).Where(d=>d.RootElement.GetProperty("kind").GetString()=="record").Select(d=>BandwidthLogReview.Read(d.RootElement.GetProperty("value"))).ToArray();
  var groups=JsonDocument.Parse(File.ReadAllText(Path.Combine(folder,"projected.json"))).RootElement;
  var error=typeof(NativeLinear).GetMethod("Errors",BindingFlags.Instance|BindingFlags.NonPublic)!;var output=new List<object>();
  foreach(var g in groups.EnumerateArray())
  {
   var indices=g.GetProperty("record_indices").EnumerateArray().Select(v=>v.GetInt32()).ToArray();var rs=indices.Select(i=>allRecords[i]).ToArray();int count=rs.Length;string method=g.GetProperty("method").GetString()!;double cutoff=g.GetProperty("cutoff").GetDouble();var pool=A(g.GetProperty("pool"));var budget=new WorkBudget(CancellationToken.None);
   NativeLinear[]? best=null;double score=double.PositiveInfinity;double[][]? bestValidation=null;
   double[] Errors(NativeLinear m,int k,int start,int end){var e=(double[])error.Invoke(m,new object[]{rs[k],NativeAutoTune.Validate(rs[k]),start,end,budget})!;var sc=A(g.GetProperty("scales")[k]);e[0]*=pool[1]/sc[1];e[1]*=pool[1]/sc[1];e[2]*=pool[0]/sc[0];return e;}
   foreach(var fit in g.GetProperty("fits").EnumerateArray())
   {
    int order=fit.GetProperty("order").GetInt32();var xx=fit.GetProperty("x");var x=new double[xx.GetArrayLength(),xx[0].GetArrayLength()];for(int i=0;i<x.GetLength(0);i++)for(int j=0;j<x.GetLength(1);j++)x[i,j]=xx[i][j].GetDouble();
    var cs=method=="ridge"?fit.GetProperty("coefficients").EnumerateArray().Select(v=>A(v)).ToArray():NativeLinear.FitConstrained(x,A(fit.GetProperty("y")),order,count,NativeAutoTune.Validate(rs[count-1]),budget).Select(f=>f.C).ToArray();
    foreach(var c in cs){var models=new NativeLinear[count];var val=new double[count][];double worst=0;
     for(int k=0;k<count;k++){var coef=new double[2*order+1];Array.Copy(c,coef,2*order);coef[2*order]=c[2*order+k];var mean=A(g.GetProperty("means")[k]);var m=new NativeLinear{Order=order,Coefficients=coef,Um=mean[0],Ym=mean[1],Us=pool[0],Ys=pool[1]};models[k]=m;val[k]=Errors(m,k,600,800);worst=Math.Max(worst,val[k].Max());}
     if(double.IsFinite(worst)&&worst<score){score=worst;best=models;bestValidation=val;}
    }
   }
   if(best==null)throw new Exception("No model");
   // Selection is locked before final windows are evaluated.
   var final=Enumerable.Range(0,count).Select(k=>Errors(best[k],k,800,1200)).ToArray();var last=best[count-1];
   var selection=new NativeLinear.Selection{Model=last,Score=score,FinalFree=final[count-1][0],FinalY=final[count-1][1],FinalU=final[count-1][2]};SvdProbeAutoTune.Override=selection;
   var result=SvdProbeAutoTune.Run(rs[count-1],CancellationToken.None,_=>{});result.accumulated_records=count;
   var row=new{record_indices=indices,count,method,cutoff,order=last.Order,gain=last.DcGain(),coefficients=last.Coefficients,selection=score,validation=bestValidation,final,result};output.Add(row);Console.WriteLine(JsonSerializer.Serialize(new{count,method,cutoff,gain=last.DcGain(),selection=score,final=final[count-1],result}));
  }
  File.WriteAllText(Path.Combine(folder,"comparison.json"),JsonSerializer.Serialize(output,new JsonSerializerOptions{WriteIndented=true,NumberHandling=System.Text.Json.Serialization.JsonNumberHandling.AllowNamedFloatingPointLiterals}));
 }
}
