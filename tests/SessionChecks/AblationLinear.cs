// Research-only source copy; never linked into game mod.
using System;

namespace PIDSupporter
{
    // Strictly proper, centered ARX. State contains newest-first output/input histories.
    // Only the initial history is observed; all later predicted outputs feed back freely.
    internal sealed partial class AblationLinear
    {
        internal int Order;
        internal double[] Coefficients = null!;
        internal double Um, Ym, Us, Ys;
        internal static bool Finite(double x) => !double.IsNaN(x) && !double.IsInfinity(x);
        internal double Next(double[] state)
        {
            double y = Coefficients[2 * Order];
            for (int j = 0; j < state.Length; j++) y += Coefficients[j] * state[j];
            return Ym + Ys * y;
        }
        internal void Push(double[] state, double y, double u)
        {
            for (int j = Order - 1; j > 0; j--) { state[j] = state[j - 1]; state[Order + j] = state[Order + j - 1]; }
            state[0] = (y - Ym) / Ys; state[Order] = (u - Um) / Us;
        }
        private double[] Initial(TuneRecord r)
        {
            var state = new double[2 * Order];
            for (int i = 0; i < Order; i++) Push(state, r.Y[i], r.U[i]);
            return state;
        }
        internal double[] RecordState(TuneRecord r, WorkBudget budget)
        {
            var state = Initial(r);
            for (int i = Order; i < r.U.Length; i++) { budget.Check(); Push(state, Next(state), r.U[i]); }
            return state;
        }
        internal double[] RecentState(TuneRecord r)
        {
            var state = new double[2 * Order];
            for (int i = r.U.Length - Order; i < r.U.Length; i++) Push(state, r.Y[i], r.U[i]);
            return state;
        }
        internal sealed class Selection
        {
            internal AblationLinear Model = null!;
            internal double Score = double.PositiveInfinity, FinalFree, FinalY, FinalU;
            internal double FinalWorst => Math.Max(FinalFree, Math.Max(FinalY, FinalU));
        }
        // Train 0-15s; select 15-20s; final diagnostics 20-30s. No refit using held-out data.
        internal static Selection Select(TuneRecord r, double dt, WorkBudget budget, Action<string> progress)
        {
            return SelectAccumulated(new[]{r},budget,progress);
        }
        // Direct SVD: do not square the condition number through Z'Z.
        internal static double[,] Instruments(double[] r, int start, int end, int lag, WorkBudget budget)
        {
            int n=end-start,d=lag+2;var z=new double[n,d];
            for(int j=0;j<d-1;j++) {
                double mean=0,ss=0;for(int i=0;i<n;i++)mean+=r[start+i-j]/n;
                for(int i=0;i<n;i++)ss+=Math.Pow(r[start+i-j]-mean,2)/n;
                double scale=Math.Max(1e-12,Math.Sqrt(ss));
                for(int i=0;i<n;i++)z[i,j]=(r[start+i-j]-mean)/scale;
            }
            for(int i=0;i<n;i++)z[i,d-1]=1;
            var svd=NumericalSvd.Decompose(z,budget);int rank=svd.Rank;
            if(rank<2)throw new ArgumentException("Insufficient external excitation");
            var q=new double[n,rank];int col=0;
            for(int j=0;j<d;j++)if(svd.S[j]>svd.Cutoff) {
                for(int i=0;i<n;i++)q[i,col]=svd.Rotated[i,j]/svd.S[j];col++;
            }
            AutoTuneDiagnostics.Event("linearInstruments",new {method="exogenous-carrier-iv",rank,columns=d,lag,start,end,
                singularValues=svd.S,singularCutoff=svd.Cutoff,relativeCutoff=svd.Cutoff/svd.Maximum});
            return q;
        }
        private double[] Errors(TuneRecord r, double dt, int start, int end, WorkBudget budget)
        {
            var free = Initial(r); var closed = Initial(r);
            int switchAt = Math.Max(Order, (int)Math.Round(4 / dt));
            double integral = r.Integrals[switchAt - 1], previous = r.R[switchAt - 1] - r.Y[switchAt - 1];
            var sums = new double[3];
            for (int i = Order; i < end; i++)
            {
                budget.Check(); double yf = Next(free), yc = Next(closed), uc = r.U[i];
                if (i >= switchAt)
                {
                    double error = r.R[i] - yc;
                    uc = NonlinearCauseChecks.Controller(r.Pid, error, previous, ref integral, r.T[i] - r.T[i - 1], NonlinearCauseChecks.CollectionClip, false); previous = error;
                }
                if (!Finite(yf) || !Finite(yc) || Math.Abs((yf - Ym) / Ys) > 1e6 || Math.Abs((yc - Ym) / Ys) > 1e6)
                    return new[] { double.PositiveInfinity, double.PositiveInfinity, double.PositiveInfinity };
                if (i >= start) { sums[0] += Math.Pow((yf - r.Y[i]) / Ys, 2); sums[1] += Math.Pow((yc - r.Y[i]) / Ys, 2); sums[2] += Math.Pow((uc - r.U[i]) / Us, 2); }
                Push(free, yf, r.U[i]); Push(closed, yc, uc);
            }
            for (int j = 0; j < 3; j++) sums[j] = Math.Sqrt(sums[j] / (end - start));
            return sums;
        }
    }
}
