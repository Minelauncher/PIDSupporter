using System.Text.Json.Nodes;
using System.Text.Json;
using System.Numerics;
partial class Program {
static double[] Arr(JsonNode x) => x.AsArray().Select(v => (double)v!).ToArray();
static void Main(string[] args) {
 var root=args[0]; var dest=Path.Combine(root,"frit"); Directory.CreateDirectory(dest);
 foreach(var path in Directory.GetFiles(Path.Combine(root,"runs"),"*.json").Order()) {
  var o=JsonNode.Parse(File.ReadAllText(path))!; var r=o["record"]!;
  var u=Arr(r["U"]!); var y=Arr(r["Y"]!); var z=Arr(r["R"]!); var pid=Arr(o["workingPid"]!);
  var rr=new double[128];var yy=new double[128];var re=new double[128];var im=new double[128];var coh=new double[128];int n=0;
  for(int k=0;k+256<=u.Length;k+=128) {
   double ym=y.Skip(k).Take(256).Average(), zm=z.Skip(k).Take(256).Average();var Y=new Complex[256];var Z=new Complex[256];
   for(int i=0;i<256;i++){double w=.5-.5*Math.Cos(2*Math.PI*i/255);Y[i]=(y[k+i]-ym)*w;Z[i]=(z[k+i]-zm)*w;}
   MathNet.Numerics.IntegralTransforms.Fourier.Forward(Y);MathNet.Numerics.IntegralTransforms.Fourier.Forward(Z);
   for(int i=0;i<128;i++){rr[i]+=Z[i].Magnitude*Z[i].Magnitude;yy[i]+=Y[i].Magnitude*Y[i].Magnitude;var c=Y[i]*Complex.Conjugate(Z[i]);re[i]+=c.Real;im[i]+=c.Imaginary;}n++;
  }
  for(int i=0;i<128;i++){rr[i]/=n;yy[i]/=n;re[i]/=n;im[i]/=n;coh[i]=rr[i]*yy[i]>1e-30?Math.Clamp((re[i]*re[i]+im[i]*im[i])/(rr[i]*yy[i]),0,1):0;}
  var sw=System.Diagnostics.Stopwatch.StartNew();
  var f=RunFritBandwidthSweep(u,null,y,u.Select(v=>Math.Abs(v)>=.999999).ToArray(),.05,.05,coh,rr,re,im,1/(256*.05),pid[0],pid[1],pid[2],out var ts,out var nm,out var diag);
  bool ok=double.IsFinite(f.Cost)&&ts>0;
  var picked=ok?new[]{Math.Clamp(Math.Round(f.Kp*1000)/1000,.001,1),Math.Clamp(Math.Round(f.Ti*10)/10,.1,250),Math.Clamp(Math.Round(f.Td*100)/100,0,10)}:pid;
  var result=new {id=(string)o["id"]!,accepted=ok,pid=picked,ts,nm,diag,detail=f.Diag,seconds=sw.Elapsed.TotalSeconds,cost=double.IsFinite(f.Cost)?(double?)f.Cost:null};
  File.WriteAllText(Path.Combine(dest,Path.GetFileName(path)),JsonSerializer.Serialize(result));Console.WriteLine($"{result.id} {ok} [{string.Join(",",picked)}] {diag} {f.Diag}");
 }
}
}
