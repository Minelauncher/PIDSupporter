using System;
using System.Collections.Generic;
namespace PIDSupporter
{
    internal sealed partial class NativeLinear
    {
        // Preserve finite-response checks without clipping genuine slow/high-gain dynamics.
        internal const double ResponseLimit = double.PositiveInfinity;
        internal const double LongestTimeConstant = 300;
        internal sealed class ConstrainedFit
        {
            internal double[] C = null!;
            internal double Loss, Peak;
            internal string Structure = "";
        }

        // Column-pivoted Householder QR. No normal equations, ridge, or 1% cutoff
        // here: only numerically unresolved pivots are rejected (relative 1e-12).
        internal static double[]? LeastSquares(double[,] x, double[] y, WorkBudget budget)
        {
            int rows=x.GetLength(0), cols=x.GetLength(1); if(rows<cols)return null;
            var a=(double[,])x.Clone();var b=(double[])y.Clone();var permutation=new int[cols];
            for(int j=0;j<cols;j++)permutation[j]=j;
            double scale=0;
            for(int k=0;k<cols;k++)
            {
                budget.Check();int pivot=k;double largest=-1;
                for(int j=k;j<cols;j++){double norm=0;for(int i=k;i<rows;i++)norm+=a[i,j]*a[i,j];if(norm>largest){largest=norm;pivot=j;}}
                double length=Math.Sqrt(Math.Max(0,largest));if(k==0)scale=length;
                if(!Finite(length)||length<=Math.Max(1e-30,scale*1e-12))return null;
                if(pivot!=k){for(int i=0;i<rows;i++){double t=a[i,k];a[i,k]=a[i,pivot];a[i,pivot]=t;}int p=permutation[k];permutation[k]=permutation[pivot];permutation[pivot]=p;}
                double alpha=a[k,k]>=0?-length:length;var v=new double[rows-k];v[0]=a[k,k]-alpha;
                for(int i=k+1;i<rows;i++)v[i-k]=a[i,k];double vv=0;foreach(double t in v)vv+=t*t;
                for(int j=k+1;j<cols;j++){double dot=0;for(int i=k;i<rows;i++)dot+=v[i-k]*a[i,j];dot*=2/vv;for(int i=k;i<rows;i++)a[i,j]-=dot*v[i-k];}
                double by=0;for(int i=k;i<rows;i++)by+=v[i-k]*b[i];by*=2/vv;for(int i=k;i<rows;i++)b[i]-=by*v[i-k];
                a[k,k]=alpha;for(int i=k+1;i<rows;i++)a[i,k]=0;
            }
            var result=new double[cols];var z=new double[cols];
            for(int i=cols-1;i>=0;i--){double t=b[i];for(int j=i+1;j<cols;j++)t-=a[i,j]*z[j];z[i]=t/a[i,i];if(!Finite(z[i]))return null;result[permutation[i]]=z[i];}
            return result;
        }

        // Search coordinates are real roots, or radius/angle of a conjugate pair
        // plus an optional real root. All admissible order1-3 pole types are covered.
        private static double[] PoleCoefficients(double[] p,int order,bool pair)
        {
            if(!pair){var poly=new double[]{1};for(int k=0;k<order;k++){var next=new double[poly.Length+1];for(int j=0;j<poly.Length;j++){next[j]+=poly[j];next[j+1]-=p[k]*poly[j];}poly=next;}var a=new double[order];for(int j=0;j<order;j++)a[j]=-poly[j+1];return a;}
            double sum=2*p[0]*Math.Cos(p[1]),product=p[0]*p[0];
            return order==2?new[]{sum,-product}:new[]{sum+p[2],-product-sum*p[2],product*p[2]};
        }
        private static double CubeRoot(double x)=>Math.Sign(x)*Math.Pow(Math.Abs(x),1.0/3);
        internal static double[] RootsAsParameters(double[] c,int order,out bool pair)
        {
            pair=false;if(order==1)return new[]{c[0]};
            double real=0, b=-c[0],d=-c[1];
            if(order==3)
            {
                double e=-c[2],p=d-b*b/3,q=2*b*b*b/27-b*d/3+e,disc=q*q/4+p*p*p/27;
                if(disc>0){real=CubeRoot(-q/2+Math.Sqrt(disc))+CubeRoot(-q/2-Math.Sqrt(disc))-b/3;}
                else if(p<0){double angle=Math.Acos(Math.Max(-1,Math.Min(1,3*q/(2*p)*Math.Sqrt(-3/p))))/3;double size=2*Math.Sqrt(-p/3);return new[]{size*Math.Cos(angle)-b/3,size*Math.Cos(angle-2*Math.PI/3)-b/3,size*Math.Cos(angle-4*Math.PI/3)-b/3};}
                else return new[]{-b/3,-b/3,-b/3};
                // Divide the cubic by its real root.
                d+=real*(b+real);b+=real;
            }
            double discriminant=b*b-4*d;
            if(discriminant>=0){double t=Math.Sqrt(discriminant);return order==2?new[]{(-b+t)/2,(-b-t)/2}:new[]{(-b+t)/2,(-b-t)/2,real};}
            pair=true;double radius=Math.Sqrt(d),theta=Math.Acos(Math.Max(-1,Math.Min(1,-b/(2*radius))));
            return order==2?new[]{radius,theta}:new[]{radius,theta,real};
        }
        internal static double PoleRadius(double[] c,int order)
        {
            var p=RootsAsParameters(c,order,out bool pair);double radius=Math.Abs(p[0]);for(int i=pair?2:1;i<p.Length;i++)radius=Math.Max(radius,Math.Abs(p[i]));return radius;
        }
        // Same sampled envelope as the research comparison, plus independent dense
        // verification before a candidate is allowed into model selection.
        private static double[,] ResponseTrig(int order,double dt,bool dense)
        {
            var f=new List<double>{0};int low=dense?160:30,mid=dense?800:90,high=dense?800:70;
            for(int i=0;i<low;i++)f.Add(.0001*Math.Pow(330,(double)i/(low-1)));
            for(int i=0;i<mid;i++)f.Add(.033+(2-.033)*i/(mid-1));
            for(int i=0;i<high;i++)f.Add(2+(.5/dt-2)*i/(high-1));
            var trig=new double[f.Count,2*order];for(int k=0;k<f.Count;k++)for(int j=0;j<order;j++){double phase=2*Math.PI*f[k]*dt*(j+1);trig[k,j]=Math.Cos(phase);trig[k,order+j]=-Math.Sin(phase);}return trig;
        }
        private static double ResponsePeak(double[] c,int order,double[,] trig)
        {
            double peak=0;for(int k=0;k<trig.GetLength(0);k++){double ar=1,ai=0,br=0,bi=0;for(int j=0;j<order;j++){ar-=c[j]*trig[k,j];ai-=c[j]*trig[k,order+j];br+=c[order+j]*trig[k,j];bi+=c[order+j]*trig[k,order+j];}double den=ar*ar+ai*ai;if(den<=1e-28)return double.PositiveInfinity;peak=Math.Max(peak,Math.Sqrt((br*br+bi*bi)/den));}return peak;
        }
        internal static double ResponsePeak(double[] c,int order,double dt)=>ResponsePeak(c,order,ResponseTrig(order,dt,true));

        internal static List<ConstrainedFit> FitConstrained(double[,] x,double[] y,int order,int count,double dt,WorkBudget budget)
        {
            int rows=x.GetLength(0),dim=x.GetLength(1);double radius=Math.Exp(-dt/LongestTimeConstant);
            var trig=ResponseTrig(order,dt,false);var dense=ResponseTrig(order,dt,true);
            var tail=new double[rows,order+count];for(int i=0;i<rows;i++)for(int j=0;j<order+count;j++)tail[i,j]=x[i,order+j];
            var offsets=new double[rows,count];for(int i=0;i<rows;i++)for(int j=0;j<count;j++)offsets[i,j]=x[i,2*order+j];
            var unconstrained=LeastSquares(x,y,budget);var output=new List<ConstrainedFit>();
            double Loss(double[] c){double sum=0;for(int i=0;i<rows;i++){double e=-y[i];for(int j=0;j<dim;j++)e+=x[i,j]*c[j];sum+=e*e;}return sum;}
            double[]? RefitOffsets(double[] c){var target=(double[])y.Clone();for(int i=0;i<rows;i++)for(int j=0;j<2*order;j++)target[i]-=x[i,j]*c[j];var off=LeastSquares(offsets,target,budget);if(off==null)return null;Array.Copy(off,0,c,2*order,count);return c;}
            double[]? Limit(double[] c,double[,] grid){double peak=ResponsePeak(c,order,grid);if(!Finite(peak))return null;if(peak>ResponseLimit){double factor=ResponseLimit*(1-1e-10)/peak;for(int j=order;j<2*order;j++)c[j]*=factor;}return RefitOffsets(c);}
            foreach(bool pair in order==1?new[]{false}:new[]{false,true})
            {
                string structure=pair?"conjugate-pair":"real-poles";
                if(unconstrained!=null)
                {
                    RootsAsParameters(unconstrained,order,out bool unPair);
                    if(unPair==pair&&PoleRadius(unconstrained,order)<=radius&&ResponsePeak(unconstrained,order,dense)<=ResponseLimit)
                    {output.Add(new ConstrainedFit{C=(double[])unconstrained.Clone(),Loss=Loss(unconstrained),Peak=ResponsePeak(unconstrained,order,dense),Structure=structure});continue;}
                }
                void Clamp(double[] p){for(int j=0;j<order;j++)p[j]=Math.Max(pair&&j<2?0:-radius,Math.Min(pair&&j==1?Math.PI:radius,p[j]));}
                double[]? VariableFit(double[] p){budget.Check();var c=new double[dim];var a=PoleCoefficients(p,order,pair);Array.Copy(a,c,order);var target=(double[])y.Clone();for(int i=0;i<rows;i++)for(int j=0;j<order;j++)target[i]-=x[i,j]*a[j];var b=LeastSquares(tail,target,budget);if(b==null)return null;Array.Copy(b,0,c,order,b.Length);return Limit(c,trig);}
                var seeds=new List<double[]>();
                foreach(double value in new[]{.5,.95,.995,-.5}){var p=new double[order];for(int j=0;j<order;j++)p[j]=value;if(pair){p[0]=Math.Abs(value);p[1]=value<0?1:.05;}Clamp(p);seeds.Add(p);}
                if(unconstrained!=null){var p=RootsAsParameters(unconstrained,order,out bool unPair);if(unPair==pair){Clamp(p);seeds.Add(p);}}
                double[]? best=null,bestP=null;double bestLoss=double.PositiveInfinity;
                foreach(var seed in seeds)
                {
                    var p=(double[])seed.Clone();var c=VariableFit(p);if(c==null)continue;double loss=Loss(c),step=.08;
                    for(int iteration=0;iteration<100&&step>1e-7;iteration++)
                    {
                        bool improved=false;
                        for(int j=0;j<order;j++)foreach(int sign in new[]{-1,1})
                        {var trial=(double[])p.Clone();trial[j]+=sign*step;Clamp(trial);var candidate=VariableFit(trial);if(candidate==null)continue;double next=Loss(candidate);if(next<loss-1e-14*Math.Max(1,loss)){p=trial;c=candidate;loss=next;improved=true;}}
                        if(!improved)step*=.5;
                    }
                    if(loss<bestLoss){best=c;bestP=p;bestLoss=loss;}
                }
                if(best==null||bestP==null)continue;
                // Refine numerator direction as well as poles. Radial projection is
                // a feasible parameterization, not a coefficient penalty or sign prior.
                var parameters=new double[2*order];Array.Copy(bestP,parameters,order);Array.Copy(best,order,parameters,order,order);
                var steps=new double[2*order];for(int j=0;j<steps.Length;j++)steps[j]=j<order?.02:Math.Max(.02,Math.Abs(parameters[j])*.2);
                for(int iteration=0;iteration<120;iteration++)
                {
                    budget.Check();bool improved=false;double largest=0;
                    for(int j=0;j<parameters.Length;j++)foreach(int sign in new[]{-1,1})
                    {
                        var trial=(double[])parameters.Clone();trial[j]+=sign*steps[j];Clamp(trial);var c=new double[dim];Array.Copy(PoleCoefficients(trial,order,pair),c,order);Array.Copy(trial,order,c,order,order);c=Limit(c,trig);if(c==null)continue;double loss=Loss(c);
                        if(loss<bestLoss-1e-14*Math.Max(1,bestLoss)){parameters=trial;best=c;bestLoss=loss;improved=true;}
                    }
                    if(!improved)for(int j=0;j<steps.Length;j++)steps[j]*=.5;
                    foreach(double step in steps)largest=Math.Max(largest,step);if(largest<1e-7)break;
                }
                best=Limit(best,dense);if(best==null||PoleRadius(best,order)>radius+1e-8)continue;
                double finalPeak=ResponsePeak(best,order,dense);if(!Finite(finalPeak)||finalPeak>ResponseLimit*(1+1e-8))continue;
                output.Add(new ConstrainedFit{C=best,Loss=Loss(best),Peak=finalPeak,Structure=structure});
            }
            return output;
        }
    }
}
