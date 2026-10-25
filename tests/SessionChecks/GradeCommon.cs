using System;using System.IO;using System.Linq;using System.Collections.Generic;using System.Text.Json;using PIDSupporter;
internal static class GradeCommon {
 static double[] A(JsonElement e,string k)=>e.GetProperty(k).EnumerateArray().Select(v=>v.GetDouble()).ToArray();
 sealed class Plant {
 readonly double a,b;readonly double[] q;double x,y;int tick;
 internal Plant(JsonElement p){a=p.GetProperty("p1").GetDouble();b=p.GetProperty("p2").GetDouble();q=new double[p.GetProperty("delay").GetInt32()];}
 internal double Output(){x=a*x+(1-a)*(q[tick%q.Length]-.2*x*Math.Abs(x));y=b*y+(1-b)*x;return y;}
 internal void Input(double u){q[tick++%q.Length]=u;}
 }
 static double Noise(Random r,double sigma)=>sigma*Math.Sqrt(-2*Math.Log(Math.Max(1e-15,r.NextDouble())))*Math.Cos(2*Math.PI*r.NextDouble());

 internal static void Run(){
 const string Root="tests/output/common-plant-state-20261020";
 var recommendations=JsonDocument.Parse(File.ReadAllText(Root+"/recommendations.json")).RootElement;
 var plan=JsonDocument.Parse(File.ReadAllText(Root+"/plan.json")).RootElement;
 var plants=JsonDocument.Parse(File.ReadAllText("tests/output/morelli-sp-iv-20261020/plants-evaluation-only.json")).RootElement;
 var refs=JsonDocument.Parse(File.ReadAllText("tests/output/repeat-synthetic-20261020/actual-results.json")).RootElement;var output=new List<object>();
 foreach(var f in plants.EnumerateArray())foreach(double sigma in new[]{0.0,.01}){
 string name=f.GetProperty("name").GetString()!;
 var cache=new Dictionary<string,double>();double Cost(double[] pid){string key=string.Join(",",pid);if(cache.TryGetValue(key,out var saved))return saved;double total=0;
 foreach(double amp in new[]{.1,.3,.6})foreach(double tau in new[]{.5,1.25,2.0}){double worst=0;foreach(int sign in new[]{-1,1}){double mean=0;foreach(int seed in new[]{6001,6002}){var plant=new Plant(f);var rng=new Random(seed);double integral=0,previous=0,cost=0;for(int k=0;k<300;k++){double y=plant.Output(),sp=sign*amp*(1-Math.Exp(-k*.05/tau)),e=sp-y-Noise(rng,sigma);double u=NativeAutoTune.PidStep(pid,e,previous,ref integral,.05,-1,1,false);previous=e;plant.Input(u);cost+=Math.Pow((sp-y)/amp,2)/300;}mean+=cost/2;}worst=Math.Max(worst,mean);}total+=worst/9;}cache[key]=total;return total;}

 var reference=refs.EnumerateArray().First(v=>v.GetProperty("plant").GetString()==name && v.GetProperty("sigma").GetDouble()==sigma);
 double referenceCost=Cost(A(reference,"referencePid"));var group=new List<(JsonElement rec,JsonElement c)>();
 foreach(var rec in recommendations.EnumerateArray()){
 string id=rec.GetProperty("id").GetString()!;var c=plan.EnumerateArray().First(v=>v.GetProperty("id").GetString()==id);string prefix=c.GetProperty("prefix").GetString()!;
 var tr=JsonDocument.Parse(File.ReadAllText("tests/output/repeat-synthetic-20261020/"+prefix+"-trajectory.json")).RootElement;
 if(tr.GetProperty("plant").GetString()!=name || tr.GetProperty("sigma").GetDouble()!=sigma)continue;
 group.Add((rec.Clone(),c.Clone()));referenceCost=Math.Min(referenceCost,Cost(A(rec.GetProperty("result"),"pid")));
 }
 foreach(var pair in group){var result=pair.rec.GetProperty("result");string prefix=pair.c.GetProperty("prefix").GetString()!;int iteration=pair.c.GetProperty("iteration").GetInt32();var data=JsonDocument.Parse(File.ReadAllText($"tests/output/repeat-synthetic-20261020/{prefix}-r{iteration}.json")).RootElement;double cost=Cost(A(result,"pid"));output.Add(new{id=pair.rec.GetProperty("id").GetString(),method=pair.rec.GetProperty("method").GetString(),plant=name,sigma,window=pair.c.GetProperty("window").GetInt32(),pid=A(result,"pid"),accepted=result.GetProperty("accepted").GetBoolean(),cost,baseline=Cost(A(data,"Pid")),referenceCost,regret=cost/referenceCost});}
 }
 File.WriteAllText(Root+"/actual-results.json",JsonSerializer.Serialize(output,new JsonSerializerOptions{WriteIndented=true}));
 }
}
