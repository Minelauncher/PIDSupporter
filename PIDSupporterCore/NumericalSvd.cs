using System;
namespace PIDSupporter
{
    // One-sided Jacobi SVD. Rank uses floating-point resolution, not a statistical cutoff.
    internal sealed class NumericalSvd
    {
        internal double[,] Rotated=null!, V=null!;
        internal double[] S=null!;
        internal double Maximum, Cutoff;
        internal int Rank;
        internal static NumericalSvd Decompose(double[,] input,WorkBudget budget)
        {
            int n=input.GetLength(0),d=input.GetLength(1);
            var a=(double[,])input.Clone();var v=new double[d,d];
            for(int j=0;j<d;j++)v[j,j]=1;
            const double eps=2.2204460492503131e-16;
            bool converged=false;
            for(int sweep=0;sweep<100;sweep++) {
                int changed=0;double maxNorm=0;
                for(int j=0;j<d;j++){double ss=0;for(int i=0;i<n;i++)ss+=a[i,j]*a[i,j];maxNorm=Math.Max(maxNorm,ss);}
                double negligible=Math.Pow(Math.Max(n,d)*eps,2)*maxNorm;
                for(int j=0;j<d;j++)for(int k=j+1;k<d;k++) {
                    budget.Check();double x=0,y=0,z=0;
                    for(int i=0;i<n;i++){x+=a[i,j]*a[i,j];y+=a[i,k]*a[i,k];z+=a[i,j]*a[i,k];}
                    if(Math.Min(x,y)<=negligible || Math.Abs(z)<=8*eps*Math.Sqrt(x*y))continue;
                    double tau=(y-x)/(2*z),t=(tau>=0?1:-1)/(Math.Abs(tau)+Math.Sqrt(1+tau*tau));
                    double c=1/Math.Sqrt(1+t*t),s=c*t;
                    for(int i=0;i<n;i++){double aj=a[i,j],ak=a[i,k];a[i,j]=c*aj-s*ak;a[i,k]=s*aj+c*ak;}
                    for(int i=0;i<d;i++){double vj=v[i,j],vk=v[i,k];v[i,j]=c*vj-s*vk;v[i,k]=s*vj+c*vk;}
                    changed++;
                }
                if(changed==0){converged=true;break;}
            }
            if(!converged)throw new InvalidOperationException("Direct SVD did not converge");
            var result=new NumericalSvd{Rotated=a,V=v,S=new double[d]};
            for(int j=0;j<d;j++){for(int i=0;i<n;i++)result.S[j]+=a[i,j]*a[i,j];result.S[j]=Math.Sqrt(result.S[j]);result.Maximum=Math.Max(result.Maximum,result.S[j]);}
            result.Cutoff=Math.Max(n,d)*eps*result.Maximum;
            for(int j=0;j<d;j++)if(result.S[j]>result.Cutoff)result.Rank++;
            return result;
        }
    }
}
