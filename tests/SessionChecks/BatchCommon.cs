using System;using System.IO;using System.Linq;using System.Collections.Generic;using System.Text.Json;using System.Threading;using PIDSupporter;
internal static class BatchCommon {
const string Root="tests/output/common-plant-state-20261020";
static double[] A(JsonElement e,string k)=>e.GetProperty(k).EnumerateArray().Select(v=>v.GetDouble()).ToArray();
static TuneRecord Read(string prefix,int n){var d=JsonDocument.Parse(File.ReadAllText($"tests/output/repeat-synthetic-20261020/{prefix}-r{n}.json")).RootElement;return new TuneRecord{U=A(d,"U"),Y=A(d,"Y"),R=A(d,"R"),InjectedSP=A(d,"InjectedSP"),T=A(d,"T"),Integrals=A(d,"Integrals"),Pid=A(d,"Pid"),Amplitude=d.GetProperty("Amplitude").GetDouble(),InitialIntegral=d.GetProperty("InitialIntegral").GetDouble(),InitialError=d.GetProperty("InitialError").GetDouble(),Lower=-1,Upper=1,Speed=.5,BaseSP=new double[600]};}
internal static void Run(string mode){
var plan=JsonDocument.Parse(File.ReadAllText(Root+"/plan.json")).RootElement;var rows=new List<object>();
foreach(var c in plan.EnumerateArray()){
 string id=c.GetProperty("id").GetString()!,prefix=c.GetProperty("prefix").GetString()!;int n=c.GetProperty("iteration").GetInt32(),w=c.GetProperty("window").GetInt32();var r=Read(prefix,n);object? selected=null;
 if(mode=="iv"){
 JointLinear.Records=Enumerable.Range(n-w+1,w).Select(i=>Read(prefix,i)).ToArray();AutoTuneDiagnostics.Sink=(k,v)=>{if(k=="linearSelected")selected=v;};JointLinear.Select(r,.05,new WorkBudget(CancellationToken.None),_=>{});rows.Add(new{id,model=selected});
 }else{
 foreach(string method in new[]{"iv","statefit"}){
 var m=JsonDocument.Parse(File.ReadAllText(Root+"/models-"+method+".json")).RootElement.EnumerateArray().First(e=>e.GetProperty("id").GetString()==id).GetProperty("model");
 var net=new NativeLinear{Order=m.GetProperty("order").GetInt32(),Coefficients=A(m,"coefficients"),Um=m.GetProperty("Um").GetDouble(),Us=m.GetProperty("Us").GetDouble(),Ym=m.GetProperty("Ym").GetDouble(),Ys=m.GetProperty("Ys").GetDouble()};
 CommonModelTune.Selected=new NativeLinear.Selection{Model=net,Score=m.GetProperty("selection").GetDouble(),FinalFree=0,FinalY=0,FinalU=0};
 var result=CommonModelTune.Run(r,CancellationToken.None,_=>{});rows.Add(new{id,method,result});Console.WriteLine(id+" "+method);
 }
 }
}
AutoTuneDiagnostics.Sink=null;File.WriteAllText(Root+(mode=="iv"?"/models-iv.json":"/recommendations.json"),JsonSerializer.Serialize(rows,new JsonSerializerOptions{WriteIndented=true}));
}
}
