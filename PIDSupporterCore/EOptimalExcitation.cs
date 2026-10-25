using System;
using System.Collections.Generic;
namespace PIDSupporter
{
    // Finite-library robust geometric E-design; not covariance-weighted Fisher information.
    internal static class EOptimalExcitation
    {
        private sealed class Block { internal double[] R=null!,U=null!,Y=null!; internal double[,] Q=null!; internal int Start; }
        internal static AdaptiveExcitation Design(double[] r,double[] u,double[] y,double dt,
            TuneRecord[] history,double[] pid,double[] weights,double amplitude,double lower,double upper,WorkBudget budget)
        {
            if(u.Length<32||r.Length!=u.Length||y.Length!=u.Length)throw new ArgumentException("Insufficient preparation data");
            var blocks=new List<Block>();
            // Completed records can be reused for NEXT experiment design, never for their own held-out scoring.
            foreach(var record in history)blocks.Add(Make(Half(record.InstrumentSP??record.InjectedSP),Half(record.U),Half(record.Y),dt,budget));
            blocks.Add(Make(r,u,y,dt,budget));
            double us=0,ys=0;int samples=0;
            foreach(var b in blocks){double um=Mean(b.U),ym=Mean(b.Y);for(int i=0;i<b.U.Length;i++){us+=Math.Pow(b.U[i]-um,2);ys+=Math.Pow(b.Y[i]-ym,2);}samples+=b.U.Length;}
            us=Math.Max(1e-4,Math.Sqrt(us/samples));ys=Math.Max(1e-5,Math.Sqrt(ys/samples));
            var models=new List<NativeLinear>();var prior=new List<double[,]>();
            for(int order=1;order<=3;order++)
            {
                var observedParts=new List<double[,]>();foreach(var b in blocks)observedParts.Add(Project(b,order,us,ys,budget));
                var observedSvd=NumericalSvd.Decompose(Stack(observedParts,2*order),budget);
                if(observedSvd.Rank<2*order){AutoTuneDiagnostics.Event("eOptimalUnsupportedOrder",new{order,rank=observedSvd.Rank,cutoff=observedSvd.Cutoff});continue;}
                int rows=0;foreach(var b in blocks)rows+=b.Q.GetLength(1);
                var x=new double[rows,2*order+blocks.Count];var target=new double[rows];int cursor=0;
                for(int k=0;k<blocks.Count;k++){
                    var b=blocks[k];double um=Mean(b.U),ym=Mean(b.Y);
                    for(int i=b.Start;i<b.U.Length;i++){
                        budget.Check();for(int v=0;v<b.Q.GetLength(1);v++){
                            double q=b.Q[i-b.Start,v];target[cursor+v]+=q*(b.Y[i]-ym)/ys;
                            for(int j=0;j<order;j++){x[cursor+v,j]+=q*(b.Y[i-j-1]-ym)/ys;x[cursor+v,order+j]+=q*(b.U[i-j-1]-um)/us;}
                            x[cursor+v,2*order+k]+=q;
                        }
                    }cursor+=b.Q.GetLength(1);
                }
                NativeLinear.ConstrainedFit? best=null;
                foreach(var fit in NativeLinear.FitConstrained(x,target,order,blocks.Count,dt,budget))if(best==null||fit.Loss<best.Loss)best=fit;
                if(best==null)throw new InvalidOperationException("No finite E-optimal design model");
                var c=new double[2*order+1];Array.Copy(best.C,c,2*order);
                // Incremental response around operating point: offsets are nuisance parameters.
                models.Add(new NativeLinear{Order=order,Coefficients=c,Us=us,Ys=ys});
                // Preparation is not part of the collected identification set. Only completed records count here.
                var parts=new List<double[,]>();for(int k=0;k<history.Length;k++)parts.Add(Project(blocks[k],order,us,ys,budget));
                prior.Add(Stack(parts,2*order));
            }
            if(models.Count==0)throw new InvalidOperationException("No numerically identifiable design order; more independent excitation required");
            var library=new List<double[]>{new[]{1.0,1,1,1}};
            for(int b=0;b<4;b++)foreach(double f in new[]{.25,.5,2.0,4.0}){var a=new[]{1.0,1,1,1};a[b]=f;library.Add(a);}
            double bestScore=double.NegativeInfinity,baseline=0;int chosen=0;
            for(int k=0;k<library.Count;k++){
                Normalize(library[k],weights);double score=double.PositiveInfinity;
                // Fixed design phase is independent of the freshly drawn collection phase.
                var wave=Wave(library[k],weights,amplitude,dt,827);
                var template=Make(wave,new double[wave.Length],new double[wave.Length],dt,budget);
                for(int m=0;m<models.Count;m++){
                    var model=models[m];var state=new double[2*model.Order];double integral=0,previous=0;
                    for(int i=0;i<wave.Length;i++){
                        budget.Check();double output=model.Next(state),e=wave[i]-output;
                        double input=NativeAutoTune.PidStep(pid,e,previous,ref integral,dt,lower,upper,false);previous=e;
                        if(!NativeLinear.Finite(output)||Math.Abs(output)>1e10){score=double.NegativeInfinity;break;}
                        template.Y[i]=output;template.U[i]=input;model.Push(state,output,input);
                    }
                    if(double.IsNegativeInfinity(score))break;
                    var projected=Project(template,model.Order,us,ys,budget);
                    var combined=Stack(new List<double[,]>{prior[m],projected},2*model.Order);
                    int informationRows=template.U.Length-template.Start;
                    for(int h=0;h<history.Length;h++)informationRows+=blocks[h].U.Length-blocks[h].Start;
                    double rootN=Math.Sqrt(informationRows);
                    for(int i=0;i<combined.GetLength(0);i++)for(int j=0;j<combined.GetLength(1);j++)combined[i,j]/=rootN;
                    var svd=NumericalSvd.Decompose(combined,budget);double minimum=double.PositiveInfinity;
                    foreach(double s in svd.S)minimum=Math.Min(minimum,s);
                    score=Math.Min(score,minimum);
                }
                if(k==0)baseline=score;
                if(score>bestScore){bestScore=score;chosen=k;}
            }
            if(!NativeLinear.Finite(bestScore))throw new InvalidOperationException("E-optimal prediction failed");
            AutoTuneDiagnostics.Event("eOptimalDesign",new{historyRecords=history.Length,orders=models.ConvertAll(model=>model.Order).ToArray(),candidates=library.Count,chosen,baseline,bestScore,us,ys,amplitude,allocation=library[chosen],criterion="min-order sigma-min projected centered X; fixed scales",designSeed=827});
            return new AdaptiveExcitation{Factors=library[chosen],MinimumSingular=bestScore};
        }
        private static double[] Half(double[] values){var result=new double[values.Length/2];Array.Copy(values,result,result.Length);return result;}
        private static double Mean(double[] values){double s=0;foreach(double x in values)s+=x/values.Length;return s;}
        private static Block Make(double[] r,double[] u,double[] y,double dt,WorkBudget budget){
            int lag=Math.Min(u.Length/4,Math.Min(80,Math.Max(1,(int)Math.Round(2/dt)))),start=Math.Max(3,lag);
            return new Block{R=r,U=u,Y=y,Start=start,Q=NativeLinear.Instruments(r,start,u.Length,lag,budget)};
        }
        private static double[,] Project(Block b,int order,double us,double ys,WorkBudget budget){
            int rows=b.U.Length-b.Start;
            var x=new double[b.Q.GetLength(1),2*order];
            // Center each lag separately to project out record-specific intercepts.
            for(int j=0;j<order;j++){
                double mu=0,my=0;for(int i=b.Start;i<b.U.Length;i++){mu+=b.U[i-j-1]/rows;my+=b.Y[i-j-1]/rows;}
                for(int i=b.Start;i<b.U.Length;i++){budget.Check();for(int v=0;v<b.Q.GetLength(1);v++){
                    x[v,j]+=b.Q[i-b.Start,v]*(b.Y[i-j-1]-my)/ys;x[v,order+j]+=b.Q[i-b.Start,v]*(b.U[i-j-1]-mu)/us;
                }}
            }return x;
        }
        private static double[,] Stack(List<double[,]> parts,int columns){int n=0;foreach(var p in parts)n+=p.GetLength(0);var result=new double[n,columns];int row=0;foreach(var p in parts){for(int i=0;i<p.GetLength(0);i++)for(int j=0;j<columns;j++)result[row+i,j]=p[i,j];row+=p.GetLength(0);}return result;}
        internal static void Normalize(double[] allocation,double[] weights){double original=0,energy=0;for(int k=0;k<weights.Length;k++){original+=weights[k]*weights[k];energy+=Math.Pow(weights[k]*allocation[AdaptiveExcitation.Band((k+1)/30.0)],2);}double scale=Math.Sqrt(original/energy);for(int b=0;b<4;b++)allocation[b]*=scale;}
        internal static double[] Wave(double[] allocation,double[] weights,double amplitude,double dt,int seed){
            // Only the first 15 s enters fitting; retain the full 30 s envelope in prediction.
            int n=(int)Math.Round(15/dt);var wave=new double[n];var random=new Random(seed);var phase=new double[60];for(int k=0;k<60;k++)phase[k]=random.NextDouble()*2*Math.PI;
            for(int i=0;i<n;i++){double t=i*dt,edge=Math.Min(t,30-t),envelope=edge>=1?1:.5*(1-Math.Cos(Math.PI*edge)),sum=0;
                for(int k=0;k<60;k++)sum+=weights[k]*allocation[AdaptiveExcitation.Band((k+1)/30.0)]*Math.Sin(2*Math.PI*(k+1)*t/30+phase[k]);
                wave[i]=amplitude*envelope*Math.Max(-1,Math.Min(1,sum/20));}return wave;
        }
    }
}
