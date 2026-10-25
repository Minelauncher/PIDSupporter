using System;using System.Linq;using System.Threading;using PIDSupporter;
internal static class ConstraintChecks
{
 static void Check(bool ok,string text){if(!ok)throw new Exception(text);}
 internal static void Run()
 {
  var budget=new WorkBudget(CancellationToken.None);var rng=new Random(431);
  var x=new double[100,2];var y=new double[100];
  for(int i=0;i<100;i++){x[i,0]=rng.NextDouble()-.5;x[i,1]=x[i,0]+1e-7*(rng.NextDouble()-.5);y[i]=2*x[i,0]-.5*x[i,1];}
  var solved=NativeLinear.LeastSquares(x,y,budget)!;Check(solved!=null&&Math.Abs(solved[0]-2)<1e-7&&Math.Abs(solved[1]+.5)<1e-7,"QR resolves correlated regressors without ridge");
  for(int i=0;i<100;i++)x[i,1]=x[i,0];Check(NativeLinear.LeastSquares(x,y,budget)==null,"exact deficient rank rejected");
  Check(Math.Abs(NativeLinear.PoleRadius(new[]{1.7,-.72},2)-.9)<1e-8,"two real poles");
  Check(Math.Abs(NativeLinear.PoleRadius(new[]{1.2,-.81},2)-.9)<1e-8,"complex pair");
  Check(Math.Abs(NativeLinear.PoleRadius(new[]{2.4,-1.91,.504},3)-.9)<1e-7,"three real poles");
  Check(Math.Abs(NativeLinear.PoleRadius(new[]{1.9,-1.65,.567},3)-.9)<1e-7,"complex pair plus real root");
  foreach(var truth in new[]{new[]{.9,-.08,0.0},new[]{1.1,2.0,.1},new[]{.5,100.0,-.2}})
  {
   var design=new double[100,3];var target=new double[100];
   for(int i=0;i<100;i++){design[i,0]=rng.NextDouble()*2-1;design[i,1]=rng.NextDouble()*2-1;design[i,2]=1;for(int j=0;j<3;j++)target[i]+=design[i,j]*truth[j];}
   var fits=NativeLinear.FitConstrained(design,target,1,1,.025,budget);Check(fits.Count>0,"feasible constrained fit exists");
   foreach(var f in fits){Check(NativeLinear.PoleRadius(f.C,1)<=Math.Exp(-.025/300)+1e-8,"unstable source cannot escape pole bound");Check(NativeLinear.ResponsePeak(f.C,1,.025)<=10.000001,"active response cap respected");}
   if(truth[1]<0)Check(Math.Abs(fits[0].C[0]-.9)<1e-9&&Math.Abs(fits[0].C[1]+.08)<1e-9,"no positive-gain prior and no shrinkage for feasible least squares");
  }
  var cancel=new CancellationTokenSource();cancel.Cancel();bool stopped=false;try{NativeLinear.FitConstrained(new double[10,3],new double[10],1,1,.025,new WorkBudget(cancel.Token));}catch(OperationCanceledException){stopped=true;}Check(stopped,"cancellation during fit");
  Console.WriteLine("Constraint checks passed: correlated/rank-deficient QR, real/complex poles, active stability and response bounds, negative gain retained when feasible, cancellation.");
 }
}
