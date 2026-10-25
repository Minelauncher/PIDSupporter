// Research-only source copy; never linked into game mod.
using System;
using System.Collections.Generic;
namespace PIDSupporter
{
    internal sealed partial class AblationLinear
    {
        // Common A/B, independent offsets and initial histories; never concatenate record boundaries.
        internal static Selection SelectAccumulated(IReadOnlyList<TuneRecord> records, WorkBudget budget, Action<string> progress)
        {
            int count=records.Count;var meansU=new double[count];var meansY=new double[count];
            var scalesU=new double[count];var scalesY=new double[count];var steps=new double[count];
            var starts=new int[count];var qs=new double[count][,];double us=0,ys=0;
            for(int k=0;k<count;k++)
            {
                var r=records[k];steps[k]=NativeAutoTune.Validate(r);int fit=r.U.Length/2;
                if(r.InjectedSP==null||r.InjectedSP.Length!=r.U.Length||r.R==null||r.R.Length!=r.U.Length||r.Integrals==null||r.Integrals.Length!=r.U.Length)
                    throw new ArgumentException("Complete injected SP and controller state required.");
                for(int i=0;i<r.U.Length;i++)if(!Finite(r.InjectedSP[i])||!Finite(r.R[i])||!Finite(r.Integrals[i]))throw new ArgumentException("Invalid history state.");
                for(int i=0;i<fit;i++){meansU[k]+=r.U[i]/fit;meansY[k]+=r.Y[i]/fit;}
                for(int i=0;i<fit;i++){scalesU[k]+=Math.Pow(r.U[i]-meansU[k],2)/fit;scalesY[k]+=Math.Pow(r.Y[i]-meansY[k],2)/fit;}
                us+=scalesU[k]/count;ys+=scalesY[k]/count;scalesU[k]=Math.Max(1e-4,Math.Sqrt(scalesU[k]));scalesY[k]=Math.Max(1e-5,Math.Sqrt(scalesY[k]));
                int lag=Math.Min(80,Math.Max(1,(int)Math.Round(2/steps[k])));starts[k]=Math.Max(16,lag);
                var instrument=r.InstrumentSP??r.InjectedSP;
                if(instrument.Length!=r.U.Length)throw new ArgumentException("Invalid instrument length");
                foreach(double value in instrument)if(!Finite(value))throw new ArgumentException("Invalid instrument");
                qs[k]=Instruments(instrument,starts[k],fit,lag,budget);
            }
            us=Math.Sqrt(us);ys=Math.Sqrt(ys);if(us<1e-4||ys<1e-5)throw new ArgumentException("Insufficient accumulated excitation.");
            var best=new Selection();AblationLinear[]? bestModels=null;
            foreach(int order in new[]{1,2,3})
            {
                int dim=2*order+count,totalRows=0;for(int k=0;k<count;k++)totalRows+=qs[k].GetLength(1);
                var projected=new double[totalRows,dim];var target=new double[totalRows];int cursor=0;
                for(int k=0;k<count;k++)
                {
                    var r=records[k];int fit=r.U.Length/2,start=starts[k];var q=qs[k];var xp=new double[q.GetLength(1),dim];var yp=new double[q.GetLength(1)];var row=new double[dim];
                    for(int i=start;i<fit;i++)
                    {
                        budget.Check();for(int j=0;j<order;j++){row[j]=(r.Y[i-1-j]-meansY[k])/ys;row[order+j]=(r.U[i-1-j]-meansU[k])/us;}row[2*order+k]=1;
                        for(int v=0;v<q.GetLength(1);v++){double z=q[i-start,v];yp[v]+=z*(r.Y[i]-meansY[k])/ys;for(int j=0;j<dim;j++)xp[v,j]+=z*row[j];}
                    }
                    double weight=1/Math.Sqrt((fit-start)*count);
                    for(int v=0;v<q.GetLength(1);v++){target[cursor]=yp[v]*weight;for(int j=0;j<dim;j++)projected[cursor,j]=xp[v,j]*weight;cursor++;}
                }
                foreach(var fitted in FitConstrained(projected,target,order,count,steps[count-1],budget))
                {
                    budget.Check();var c=fitted.C;double score=0;var models=new AblationLinear[count];
                    for(int k=0;k<count;k++)
                    {
                        var coefficients=new double[2*order+1];Array.Copy(c,coefficients,2*order);coefficients[2*order]=c[2*order+k];
                        var m=new AblationLinear{Order=order,Coefficients=coefficients,Um=meansU[k],Ym=meansY[k],Us=us,Ys=ys};models[k]=m;var r=records[k];var e=m.Errors(r,steps[k],r.U.Length/2,r.U.Length*2/3,budget);
                        // Keep per-record validation units unchanged despite pooled fit scaling.
                        score=Math.Max(score,Math.Max(e[0]*ys/scalesY[k],Math.Max(e[1]*ys/scalesY[k],e[2]*us/scalesU[k])));
                    }
                    AutoTuneDiagnostics.Event("linearConstrainedCandidate",new{records=count,order,method="injected-sp-iv-constrained",ridge=0.0,structure=fitted.Structure,trainingLoss=fitted.Loss,responsePeak=fitted.Peak,responseLimit=ResponseLimit,poleRadius=PoleRadius(c,order),score});
                    if(Finite(score)&&score<best.Score){best.Score=score;best.Model=models[count-1];bestModels=models;}
                }
                progress($"Joint identification: {count} records | order {order} | error {best.Score:0.000}");
            }
            if(bestModels==null)throw new InvalidOperationException("No finite accumulated model.");
            int last=count-1;var latest=records[last];var final=best.Model.Errors(latest,steps[last],latest.U.Length*2/3,latest.U.Length,budget);
            best.FinalFree=final[0]*ys/scalesY[last];best.FinalY=final[1]*ys/scalesY[last];best.FinalU=final[2]*us/scalesU[last];
            AutoTuneDiagnostics.Event("linearAccumulatedSelected",new{records=count,method="injected-sp-iv-constrained",ridge=0.0,order=best.Model.Order,coefficients=best.Model.Coefficients,best.Model.Um,best.Model.Us,best.Model.Ym,best.Model.Ys,validation=best.Score,best.FinalFree,best.FinalY,best.FinalU,poleRadius=PoleRadius(best.Model.Coefficients,best.Model.Order),poleLimit=Math.Exp(-steps[last]/LongestTimeConstant),responsePeak=ResponsePeak(best.Model.Coefficients,best.Model.Order,steps[last]),responseLimit=ResponseLimit});
            return best;
        }
        internal double DcGain()
        {
            double denominator=1,numerator=0;for(int j=0;j<Order;j++){denominator-=Coefficients[j];numerator+=Coefficients[Order+j];}
            return numerator/denominator*Ys/Us;
        }
    }
}
