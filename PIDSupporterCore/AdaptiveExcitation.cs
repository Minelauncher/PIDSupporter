using System;
namespace PIDSupporter
{
    // A bounded allocation heuristic, not an optimal-design or stability guarantee.
    // All observations passed here precede the block that will use the result.
    internal sealed class AdaptiveExcitation
    {
        internal double[] Factors = {1,1,1,1};
        internal double[] SingularValues=null!, Response=new double[4], Sensitivity=new double[4];
        internal double MinimumSingular;
        internal static int Band(double hz)=>hz<.1?0:hz<.3?1:hz<1?2:3;
        internal static AdaptiveExcitation Analyze(double[] carrier,double[] u,double[] y,double dt,WorkBudget budget)
        {
            int n=u.Length,lag=Math.Min(n/4,Math.Min(80,(int)Math.Round(2/dt))),start=Math.Max(lag,3),rows=n-start;
            if(rows<32)throw new ArgumentException("More observations required for allocation");
            double um=0,ym=0,us=0,ys=0;
            for(int i=0;i<n;i++){um+=u[i]/n;ym+=y[i]/n;}
            for(int i=0;i<n;i++){us+=Math.Pow(u[i]-um,2)/n;ys+=Math.Pow(y[i]-ym,2)/n;}
            us=Math.Max(1e-8,Math.Sqrt(us));ys=Math.Max(1e-8,Math.Sqrt(ys));
            var q=NativeLinear.Instruments(carrier,start,n,lag,budget);
            var x=new double[rows,6];var norm=new double[6];
            for(int i=start;i<n;i++)for(int j=0;j<3;j++){
                x[i-start,j]=(y[i-j-1]-ym)/ys;x[i-start,j+3]=(u[i-j-1]-um)/us;
            }
            for(int j=0;j<6;j++){
                for(int i=0;i<rows;i++)norm[j]+=x[i,j]*x[i,j];norm[j]=Math.Max(1e-12,Math.Sqrt(norm[j]));
            }
            // Normalize X before projection. Normalizing projected columns would hide weak excitation.
            var projected=new double[q.GetLength(1),6];
            for(int k=0;k<q.GetLength(1);k++)for(int i=0;i<rows;i++){
                budget.Check();for(int j=0;j<6;j++)projected[k,j]+=q[i,k]*x[i,j]/norm[j];
            }
            var svd=NumericalSvd.Decompose(projected,budget);int weak=0;
            for(int j=1;j<6;j++)if(svd.S[j]<svd.S[weak])weak=j;
            var result=new AdaptiveExcitation{SingularValues=svd.S,MinimumSingular=svd.S[weak]};
            var counts=new int[4];
            // Split-half consistency suppresses transient/coincidental correlation. This is
            // a response indicator, not a calibrated coherence estimate at short durations.
            for(int k=1;k<=60;k++){
                budget.Check();double hz=k/30.0;if(hz*n*dt<2)continue;
                int band=Band(hz);double[] ur=new double[2],ui=new double[2],yr=new double[2],yi=new double[2],rr=new double[2],ri=new double[2];
                for(int h=0;h<2;h++)for(int i=h*n/2;i<(h+1)*n/2;i++){
                    double phase=2*Math.PI*hz*i*dt,c=Math.Cos(phase)*4/n,s=Math.Sin(phase)*4/n;
                    rr[h]+=carrier[i]*c;ri[h]+=carrier[i]*s;ur[h]+=(u[i]-um)*c;ui[h]+=(u[i]-um)*s;yr[h]+=(y[i]-ym)*c;yi[h]+=(y[i]-ym)*s;
                }
                for(int h=0;h<2;h++){
                    double rn=Math.Sqrt(rr[h]*rr[h]+ri[h]*ri[h]);
                    if(rn<1e-8){ur[h]=ui[h]=yr[h]=yi[h]=0;continue;}
                    double a=ur[h],b=ui[h];ur[h]=(a*rr[h]+b*ri[h])/rn;ui[h]=(b*rr[h]-a*ri[h])/rn;
                    a=yr[h];b=yi[h];yr[h]=(a*rr[h]+b*ri[h])/rn;yi[h]=(b*rr[h]-a*ri[h])/rn;
                }
                double linkedU=Math.Sqrt(Math.Max(0,ur[0]*ur[1]+ui[0]*ui[1]))/us;
                double linkedY=Math.Sqrt(Math.Max(0,yr[0]*yr[1]+yi[0]*yi[1]))/ys;
                double sr=0,si=0;
                for(int j=0;j<3;j++){
                    double phase=2*Math.PI*hz*(j+1)*dt,c=Math.Cos(phase),s=Math.Sin(phase);
                    double ar=svd.V[j,weak]*(yr[0]+yr[1])/(2*ys*norm[j])+svd.V[j+3,weak]*(ur[0]+ur[1])/(2*us*norm[j+3]);
                    double ai=svd.V[j,weak]*(yi[0]+yi[1])/(2*ys*norm[j])+svd.V[j+3,weak]*(ui[0]+ui[1])/(2*us*norm[j+3]);
                    sr+=ar*c-ai*s;si+=ar*s+ai*c;
                }
                result.Response[band]+=Math.Min(linkedU,linkedY);
                result.Sensitivity[band]+=Math.Sqrt(sr*sr+si*si);counts[band]++;
            }
            double maxSensitivity=0;
            for(int b=0;b<4;b++){if(counts[b]>0){result.Response[b]/=counts[b];result.Sensitivity[b]/=counts[b];}maxSensitivity=Math.Max(maxSensitivity,result.Sensitivity[b]);}
            for(int b=0;b<4;b++)if(counts[b]>0){
                double lack=1/(1+10*result.Response[b]);
                double weakAlignment=maxSensitivity>1e-12?result.Sensitivity[b]/maxSensitivity:0;
                result.Factors[b]=1+.5*lack+.5*weakAlignment/(1+10*result.MinimumSingular);
            }
            return result;
        }
        internal static double GainFactor(double saturation,double effort,double errorRms,double previousErrorRms,int reversals)
        {
            if(saturation>.1 || (reversals>=4 && errorRms>previousErrorRms*1.25))return .5;
            if(saturation==0 && effort<.12 && errorRms>.1 && reversals<2)return 1.25;
            return 1;
        }
    }
}
