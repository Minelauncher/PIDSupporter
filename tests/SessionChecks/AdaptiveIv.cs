using System;
using PIDSupporter;
internal static class AdaptiveIv {
static bool Finite(double x)=>NativeLinear.Finite(x);
static double[] Errors(NativeLinear m,TuneRecord r,double dt,int start,int end,WorkBudget budget)=>(double[])typeof(NativeLinear).GetMethod("Errors",System.Reflection.BindingFlags.Instance|System.Reflection.BindingFlags.NonPublic)!.Invoke(m,new object[]{r,dt,start,end,budget})!;
        internal static NativeLinear.Selection Select(TuneRecord r, double[] instrument, double dt, WorkBudget budget, Action<string> progress)
        {
            int n = r.U.Length, fit = n / 2, end = n * 2 / 3;
            if (r.R == null || r.Integrals == null || r.R.Length != n || r.Integrals.Length != n)
                throw new ArgumentException("SP and integral-state records required");
            for (int i = 0; i < n; i++) if (!Finite(r.R[i]) || !Finite(r.Integrals[i])) throw new ArgumentException("Invalid recorded controller state");
            double um = 0, ym = 0, us = 0, ys = 0;
            for (int i = 0; i < fit; i++) { um += r.U[i] / fit; ym += r.Y[i] / fit; }
            for (int i = 0; i < fit; i++) { us += Math.Pow(r.U[i] - um, 2) / fit; ys += Math.Pow(r.Y[i] - ym, 2) / fit; }
            us = Math.Sqrt(us); ys = Math.Sqrt(ys);
            if (us < 1e-4 || ys < 1e-5) throw new ArgumentException("Insufficient training excitation");
            int lag = Math.Min(80, Math.Max(1, (int)Math.Round(2 / dt)));
            int start = Math.Max(16, lag);
            var q = Instruments(instrument, start, fit, lag, budget);
            var best = new NativeLinear.Selection();
            foreach (int order in new[] { 1, 2, 3, 4, 6, 8, 12, 16 })
            {
                int d = 2 * order + 1; var gram = new double[d, d]; var rhs = new double[d]; var row = new double[d];
                // IV projects regressors and target onto the external SP instrument space.
                var xp = new double[q.GetLength(1), d]; var yp = new double[q.GetLength(1)];
                for (int i = start; i < fit; i++)
                {
                    budget.Check();
                    for (int j = 0; j < order; j++) { row[j] = (r.Y[i-1-j]-ym)/ys; row[order+j] = (r.U[i-1-j]-um)/us; }
                    row[d-1] = 1;
                    for (int k = 0; k < q.GetLength(1); k++)
                    {
                        double v = q[i-start,k]; yp[k] += v*(r.Y[i]-ym)/ys;
                        for (int j = 0; j < d; j++) xp[k,j] += v*row[j];
                    }
                }
                for (int k = 0; k < q.GetLength(1); k++)
                {
                    budget.Check();
                    for (int j = 0; j < d; j++)
                    { rhs[j] += xp[k,j]*yp[k]/(fit-start); for (int h = 0; h <= j; h++) gram[j,h] += xp[k,j]*xp[k,h]/(fit-start); }
                }
                for (int j = 0; j < d; j++) for (int k = 0; k < j; k++) gram[k,j] = gram[j,k];
                foreach (double ridge in new[] { 1e-10, 1e-8, 1e-6, 1e-4, .01, 1.0 })
                {
                    budget.Check(); var a = (double[,])gram.Clone();
                    for (int j = 0; j < d; j++) a[j, j] += ridge;
                    var coefficients = Solve(a, rhs, budget); if (coefficients == null) continue;
                    var model = new NativeLinear { Order = order, Coefficients = coefficients, Um = um, Us = us, Ym = ym, Ys = ys };
                    var errors = Errors(model, r, dt, fit, end, budget);
                    // Minimax prevents an excellent closed-loop fit hiding a poor free rollout (or vice versa).
                    double score = Math.Max(errors[0], Math.Max(errors[1], errors[2]));
                    AutoTuneDiagnostics.Event("linearCandidate", new { method = "sp-iv", order, ridge, free = errors[0], closedY = errors[1], closedU = errors[2], score });
                    if (Finite(score) && score < best.Score) { best.Model = model; best.Score = score; }
                }
                progress($"SP-IV 선형 {order}차 비교 · 최선 검증오차 {best.Score:0.000}");
            }
            if (best.Model == null) throw new InvalidOperationException("No finite linear model / 유효한 모델 없음");
            var final = Errors(best.Model, r, dt, end, n, budget);
            best.FinalFree = final[0]; best.FinalY = final[1]; best.FinalU = final[2];
            AutoTuneDiagnostics.Event("linearSelected", new { method = "sp-iv", instrumentRank = q.GetLength(1), instrumentLag = lag, singularCutoff = .01, order = best.Model.Order, coefficients = best.Model.Coefficients, best.Model.Um, best.Model.Us, best.Model.Ym, best.Model.Ys, selection = best.Score, finalFree = final[0], finalY = final[1], finalU = final[2] });
            return best;
        }
        // Training-only thin SVD via symmetric Jacobi on Z'Z. Cutoff is fixed at 1%.
        private static double[,] Instruments(double[] r, int start, int end, int lag, WorkBudget budget)
        {
            int n = end-start, d = lag+2;
            var z = new double[n,d]; var a = new double[d,d]; var v = new double[d,d];
            for (int j=0;j<d-1;j++)
            {
                budget.Check(); double mean=0, variance=0;
                for(int i=0;i<n;i++) mean+=r[start+i-j]/n;
                for(int i=0;i<n;i++) variance+=Math.Pow(r[start+i-j]-mean,2)/n;
                double scale=Math.Max(Math.Sqrt(variance),1e-10);
                for(int i=0;i<n;i++) z[i,j]=(r[start+i-j]-mean)/scale;
            }
            for(int i=0;i<n;i++) z[i,d-1]=1;
            for(int j=0;j<d;j++)
            {
                budget.Check(); v[j,j]=1;
                for(int k=0;k<=j;k++) { double sum=0; for(int i=0;i<n;i++) sum+=z[i,j]*z[i,k]; a[j,k]=a[k,j]=sum; }
            }
            bool converged=false;
            for(int sweep=0;sweep<80;sweep++)
            {
                double largest=0, diagonal=0;
                for(int j=0;j<d;j++) diagonal=Math.Max(diagonal,Math.Abs(a[j,j]));
                for(int j=0;j<d;j++)
                {
                    budget.Check();
                    for(int k=j+1;k<d;k++)
                    {
                        double off=a[j,k]; largest=Math.Max(largest,Math.Abs(off));
                        if(Math.Abs(off)<=1e-13*Math.Max(1,diagonal)) continue;
                        double tau=(a[k,k]-a[j,j])/(2*off);
                        double t=(tau>=0?1:-1)/(Math.Abs(tau)+Math.Sqrt(1+tau*tau));
                        double c=1/Math.Sqrt(1+t*t), sn=t*c;
                        a[j,j]-=t*off; a[k,k]+=t*off; a[j,k]=a[k,j]=0;
                        for(int h=0;h<d;h++)
                        {
                            if(h!=j && h!=k) { double x=a[h,j], y=a[h,k]; a[h,j]=a[j,h]=c*x-sn*y; a[h,k]=a[k,h]=sn*x+c*y; }
                            double vx=v[h,j], vy=v[h,k]; v[h,j]=c*vx-sn*vy; v[h,k]=sn*vx+c*vy;
                        }
                    }
                }
                if(largest<=1e-11*Math.Max(1,diagonal)) { converged=true; break; }
            }
            if(!converged) throw new InvalidOperationException("SP instrument decomposition did not converge");
            double max=0; for(int j=0;j<d;j++) max=Math.Max(max,a[j,j]);
            int rank=0; for(int j=0;j<d;j++) if(a[j,j]>max*1e-4) rank++;
            if(rank<2) throw new ArgumentException("Insufficient external SP excitation");
            var q=new double[n,rank]; int col=0;
            for(int j=0;j<d;j++) if(a[j,j]>max*1e-4)
            {
                budget.Check(); double scale=Math.Sqrt(a[j,j]);
                for(int i=0;i<n;i++) { for(int k=0;k<d;k++) q[i,col]+=z[i,k]*v[k,j]; q[i,col]/=scale; }
                col++;
            }
            AutoTuneDiagnostics.Event("linearInstruments",new { method="sp-iv",rank,lag,start,end,singularCutoff=.01 });
            return q;
        }
        private static double[]? Solve(double[,] a, double[] rhs, WorkBudget budget)
        {
            int n = rhs.Length; var b = (double[])rhs.Clone();
            // Cholesky on ridge-regularized normal equations; reject failed/nonfinite pivots.
            for (int i = 0; i < n; i++)
            {
                budget.Check();
                for (int j = 0; j <= i; j++)
                {
                    double v = a[i, j]; for (int k = 0; k < j; k++) v -= a[i, k] * a[j, k];
                    if (i == j) { if (!Finite(v) || v <= 0) return null; a[i, j] = Math.Sqrt(v); }
                    else a[i, j] = v / a[j, j];
                }
            }
            for (int i = 0; i < n; i++) { for (int j = 0; j < i; j++) b[i] -= a[i, j] * b[j]; b[i] /= a[i, i]; }
            for (int i = n - 1; i >= 0; i--) { for (int j = i + 1; j < n; j++) b[i] -= a[j, i] * b[j]; b[i] /= a[i, i]; if (!Finite(b[i])) return null; }
            return b;
        }
}
