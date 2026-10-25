using System;using System.IO;using System.Linq;using System.Text.Json;using System.Threading;using System.Reflection;using System.Collections.Generic;using PIDSupporter;
internal static class NegativeGainReview {
 internal static void Run(string log,string folder){
 var rs=File.ReadLines(log).Select(l=>JsonDocument.Parse(l)).Where(d=>d.RootElement.GetProperty("kind").GetString()=="record").Select(d=>BandwidthLogReview.Read(d.RootElement.GetProperty("value"))).ToArray();
 var cases=JsonDocument.Parse(File.ReadAllText(Path.Combine(folder,"cases.json"))).RootElement;
 var errors=typeof(NativeLinear).GetMethod("Errors",BindingFlags.Instance|BindingFlags.NonPublic)!;var rows=new List<object>();
 foreach(var c in cases.EnumerateArray()){
  var vals=new List<double[]>();var finals=new List<double[]>();double gain=0;
  for(int k=0;k<c.GetProperty("models").GetArrayLength();k++){
   var v=c.GetProperty("models")[k];var m=new NativeLinear{Order=v.GetProperty("Order").GetInt32(),Coefficients=v.GetProperty("Coefficients").EnumerateArray().Select(x=>x.GetDouble()).ToArray(),Um=v.GetProperty("Um").GetDouble(),Ym=v.GetProperty("Ym").GetDouble(),Us=v.GetProperty("Us").GetDouble(),Ys=v.GetProperty("Ys").GetDouble()};gain=m.DcGain();
   var budget=new WorkBudget(CancellationToken.None);double dt=NativeAutoTune.Validate(rs[k]);
   double[] E(int start,int end){var e=(double[])errors.Invoke(m,new object[]{rs[k],dt,start,end,budget})!;var sc=c.GetProperty("scales")[k];e[0]*=m.Ys/sc[1].GetDouble();e[1]*=m.Ys/sc[1].GetDouble();e[2]*=m.Us/sc[0].GetDouble();return e.Select(x=>double.IsFinite(x)?x:1e100).ToArray();}
   vals.Add(E(600,800));finals.Add(E(800,1200));
  }
  rows.Add(new{variant=c.GetProperty("variant").GetString(),order=c.GetProperty("order").GetInt32(),ridge=c.GetProperty("ridge").GetDouble(),gain,validation=vals,final=finals});
 }
 File.WriteAllText(Path.Combine(folder,"evaluated.json"),JsonSerializer.Serialize(rows));Console.WriteLine($"Evaluated {rows.Count} candidates with production free and PID closed-loop replay.");
 }
}
