using System;
using System.Threading;
namespace PIDSupporter
{
    internal sealed class AutoTuneSpectrum
    {
        internal double[] Frequency=null!, CoherenceFrequency=null!;
        internal double[][] Power=null!, Coherence=null!, BandRms=null!;
        internal int Windows;
        internal static readonly string[] Bands={"0.03-0.10","0.10-0.30","0.30-1.00","1.00-2.00"};
        // One-sided Hann periodograms in signal-units squared / Hz.
        private static void Fourier(double[] x,int start,int n,double dt,double hz,out double re,out double im)
        {
            double mean=0;for(int i=0;i<n;i++)mean+=x[start+i]/n;re=im=0;
            for(int i=0;i<n;i++){double w=.5-.5*Math.Cos(2*Math.PI*i/(n-1));double phase=2*Math.PI*hz*i*dt;double v=(x[start+i]-mean)*w;re+=v*Math.Cos(phase);im-=v*Math.Sin(phase);}
        }
        private static double WindowEnergy(int n){double s=0;for(int i=0;i<n;i++){double w=.5-.5*Math.Cos(2*Math.PI*i/(n-1));s+=w*w;}return s;}
        internal static AutoTuneSpectrum Analyze(TuneRecord r)
        {
            NativeAutoTune.Validate(r);int n=r.U.Length;
            // Float game timestamps quantize individual intervals. Use total elapsed
            // time for the frequency axis, and nominal bin indices for band membership.
            double dt=(r.T[n-1]-r.T[0])/(n-1);
            if(r.InjectedSP==null||r.InjectedSP.Length!=n)throw new ArgumentException("Injected SP required for bandwidth diagnostics.");
            foreach(double value in r.InjectedSP)if(!NativeLinear.Finite(value))throw new ArgumentException("Non-finite injected SP.");
            var budget=new WorkBudget(CancellationToken.None);var signals=new[]{r.InjectedSP,r.U,r.Y};
            var s=new AutoTuneSpectrum{Frequency=new double[60],Power=new[]{new double[60],new double[60],new double[60]},CoherenceFrequency=new double[20],Coherence=new[]{new double[20],new double[20]},BandRms=new[]{new double[4],new double[4],new double[4]}};
            double factor=2*dt/WindowEnergy(n),df=r.Duration/(30*n*dt);
            for(int k=0;k<60;k++)
            {
                budget.Check();double hz=(k+1)*df;s.Frequency[k]=hz;
                int band=k+1<3?0:k+1<9?1:k+1<30?2:3;
                for(int a=0;a<3;a++){Fourier(signals[a],0,n,dt,hz,out double re,out double im);double power=factor*(re*re+im*im);s.Power[a][k]=power;s.BandRms[a][band]+=power*df;}
            }
            for(int a=0;a<3;a++)for(int b=0;b<4;b++)s.BandRms[a][b]=Math.Sqrt(s.BandRms[a][b]);
            int length=(int)Math.Round(10/dt),hop=length/2;s.Windows=1+(n-length)/hop;
            var excitationPower=new double[20];double peakExcitation=0;
            for(int k=0;k<20;k++)
            {
                budget.Check();double hz=(k+1)/(length*dt);s.CoherenceFrequency[k]=hz;
                var p=new double[3];var cr=new double[2];var ci=new double[2];
                for(int start=0;start+length<=n;start+=hop)
                {
                    var re=new double[3];var im=new double[3];for(int a=0;a<3;a++){Fourier(signals[a],start,length,dt,hz,out re[a],out im[a]);p[a]+=re[a]*re[a]+im[a]*im[a];}
                    for(int a=0;a<2;a++){cr[a]+=re[0]*re[a+1]+im[0]*im[a+1];ci[a]+=im[0]*re[a+1]-re[0]*im[a+1];}
                }
                for(int a=0;a<2;a++)s.Coherence[a][k]=s.Windows<3||p[0]<1e-16||p[a+1]<1e-16?double.NaN:Math.Min(1,(cr[a]*cr[a]+ci[a]*ci[a])/(p[0]*p[a+1]));
                excitationPower[k]=p[0];peakExcitation=Math.Max(peakExcitation,p[0]);
            }
            // Suppress near-empty excitation bins instead of displaying spurious unit coherence.
            for(int k=0;k<20;k++)if(excitationPower[k]<peakExcitation*1e-6)for(int a=0;a<2;a++)s.Coherence[a][k]=double.NaN;
            return s;
        }
    }
}
