using System;
using System.Collections.Generic;
using System.Diagnostics;
namespace PIDSupporter
{
    internal static class PidSearch
    {
        internal const double TrackingTolerance = .01;
        internal sealed class Motion
        {
            internal double Tracking, Slew, Effort;
        }
        internal sealed class Ranked
        {
            internal double[] Pid = null!;
            internal double Robust, Nominal, Slew, Effort;
        }
        // Fixed global floors avoid pairwise epsilon comparisons drifting through a chain
        // of "almost equal" candidates. The lower-priority objective cannot buy unlimited error.
        internal static Ranked ChooseHierarchy(List<Ranked> rows)
        {
            double floor = double.PositiveInfinity;
            foreach (var row in rows) floor = Math.Min(floor, row.Robust);
            double robustLimit = floor + Math.Max(1e-12, TrackingTolerance * Math.Abs(floor));
            double nominalFloor = double.PositiveInfinity;
            foreach (var row in rows) if (row.Robust <= robustLimit) nominalFloor = Math.Min(nominalFloor, row.Nominal);
            double nominalLimit = nominalFloor + Math.Max(1e-12, TrackingTolerance * Math.Abs(nominalFloor));
            Ranked? best = null;
            foreach (var row in rows)
                if (row.Robust <= robustLimit && row.Nominal <= nominalLimit &&
                    (best == null || row.Slew < best.Slew || (row.Slew == best.Slew && row.Effort < best.Effort))) best = row;
            return best ?? throw new InvalidOperationException("No finite hierarchical candidate");
        }
        internal sealed class Result
        {
            public double[] Pid { get; set; } = null!;
            public double RobustCost { get; set; }
            public bool Hierarchical { get; set; }
            public double TrackingTolerance { get; set; }
            public double PrimaryMinimum { get; set; }
            public double SelectedSlew { get; set; }
            public double SelectedEffort { get; set; }
            public int HierarchyCandidates { get; set; }
            public bool JointNeighborhood { get; set; }
            public double SelectedAxisCost { get; set; }
            public double SelectedJointCost { get; set; }
            public double BaselineJointCost { get; set; }
            public double NominalCost { get; set; }
            public double BaselineRobust { get; set; }
            public double BaselineNominal { get; set; }
            public double[] GridPid { get; set; } = null!;
            public double GridCost { get; set; }
            public double GridRobust { get; set; }
            public double[] DeNominalPid { get; set; } = null!;
            public double DeNominalCost { get; set; }
            public double DeNominalRobust { get; set; }
            public double GridSeconds { get; set; }
            public double NominalSeconds { get; set; }
            public double RobustSeconds { get; set; }
            public double Seconds { get; set; }
            public int Evaluations { get; set; }
            public bool BudgetReached { get; set; }
        }
        // Tensor grid: center and +/- max(10%, one slider tick) in each active axis.
        // Off modes stay off. Includes axes, edges and corners; not a continuum certificate.
        internal static double Neighborhood(double[] p, Func<double[],double> evaluate, bool joint)
        {
            double worst=evaluate(p);
            for(int a=-1;a<=1;a++) for(int b=-1;b<=1;b++) for(int c=-1;c<=1;c++)
            {
                if (!joint && Math.Abs(a)+Math.Abs(b)+Math.Abs(c)>1) continue;
                if ((p[1]>=250 && b!=0) || (p[2]==0 && c!=0)) continue;
                var q=(double[])p.Clone(); var signs=new[]{a,b,c};
                for(int j=0;j<3;j++) if(signs[j]!=0)
                    q[j]=Math.Round(NativeAutoTune.Clamp(p[j]+signs[j]*Math.Max(j==0?.001:.1,.1*p[j]),j==0?.001:(j==1?.1:0),j==0?1:(j==1?249.9:100)),j==0?3:1);
                worst=Math.Max(worst,evaluate(q));
            }
            return worst;
        }
        // Quantized physical coordinates; I-off and D-off are explicit separate DE populations.
        private static double[] Decode(double[] x, int mode) => new[] {
            Math.Round(Math.Exp(Math.Log(.001) + x[0] * Math.Log(1000)), 3),
            (mode & 1) != 0 ? 250 : Math.Min(249.9, Math.Round(Math.Exp(Math.Log(.1) + x[1] * Math.Log(2499)), 1)),
            (mode & 2) != 0 ? 0 : Math.Round(Math.Exp(Math.Log(.1) + x[2] * Math.Log(1000)), 1) };
        internal static Result Run(double[] original, Func<double[], double> cost, WorkBudget budget, Action<string> progress, bool joint = false, Func<double[], Motion>? motion = null)
        {
            var timer = Stopwatch.StartNew(); var result = new Result();
            var cache = new Dictionary<(double,double,double),double>();
            double Eval(double[] p)
            {
                budget.Check(); var key=(p[0],p[1],p[2]);
                if (!cache.TryGetValue(key,out double c)) { c=cost(p); if (!NativeLinear.Finite(c)) c=1e12; cache.Add(key,c); }
                return c;
            }
            var robustCache = new Dictionary<(double,double,double),double>();
            double Robust(double[] p)
            {
                var key=(p[0],p[1],p[2]);
                if (!robustCache.TryGetValue(key,out double value)) { value=Neighborhood(p,Eval,joint);robustCache.Add(key,value); }
                return value;
            }
            result.BaselineNominal=Eval(original); result.GridPid=(double[])original.Clone(); result.GridCost=result.BaselineNominal;
            foreach(var p in NativeAutoTune.Candidates(original)) { double c=Eval(p); if(c<result.GridCost) {result.GridCost=c;result.GridPid=p;} }
            result.GridSeconds=timer.Elapsed.TotalSeconds;
            result.BaselineRobust=Robust(original); result.GridRobust=Robust(result.GridPid);
            bool Limit(double deadline) => timer.Elapsed.TotalSeconds>=deadline || cache.Count>=18000;
            double[] Optimize(bool robust, double[] seed, double deDeadline, double finalDeadline)
            {
                Func<double[],double> score=robust ? new Func<double[],double>(Robust) : Eval;
                var best=(double[])seed.Clone(); double bestCost=score(best);
                double incumbent=score(original); if(incumbent<bestCost) {best=(double[])original.Clone();bestCost=incumbent;}
                var rng=new Random(robust?8257:8256);
                const int size=16;
                // Interleave generations across all modes so the time cap cannot starve a mode.
                var populations=new double[4][][]; var scores=new double[4][];
                for(int mode=0;mode<4;mode++)
                {
                    populations[mode]=new double[size][];scores[mode]=new double[size];
                    for(int i=0;i<size;i++)
                    {
                        var x=new[]{rng.NextDouble(),rng.NextDouble(),rng.NextDouble()};populations[mode][i]=x;
                        var p=Decode(x,mode);scores[mode][i]=score(p);
                        if(scores[mode][i]<bestCost){best=p;bestCost=scores[mode][i];}
                    }
                }
                for(int gen=0;gen<32&&!Limit(deDeadline);gen++) for(int mode=0;mode<4&&!Limit(deDeadline);mode++)
                {
                    var pop=populations[mode];var values=scores[mode];
                    for(int i=0;i<size&&!Limit(deDeadline);i++)
                    {
                        budget.Check();int a,b,c;do{a=rng.Next(size);}while(a==i);do{b=rng.Next(size);}while(b==i||b==a);do{c=rng.Next(size);}while(c==i||c==a||c==b);
                        int forced=rng.Next(3);var trial=(double[])pop[i].Clone();
                        for(int j=0;j<3;j++)if(j==forced||rng.NextDouble()<.85)trial[j]=NativeAutoTune.Clamp(pop[a][j]+.7*(pop[b][j]-pop[c][j]),0,1);
                        var p=Decode(trial,mode);double value=score(p);
                        if(value<values[i]){pop[i]=trial;values[i]=value;}if(value<bestCost){best=p;bestCost=value;}
                    }
                    if (mode == 3 && gen % 4 == 0) progress($"PID {(robust?"neighborhood":"DE")} search | {cache.Count} evaluations");
                }
                // Coordinate pattern search down to game ticks, explicitly check I/D-off switches.
                for(double fraction=.2;fraction>=.003&&!Limit(finalDeadline);fraction*=.5)
                    for(int sweep=0;sweep<12&&!Limit(finalDeadline);sweep++)
                    {
                        bool improved=false;
                        for(int j=0;j<3&&!Limit(finalDeadline);j++)for(int sign=-1;sign<=1;sign+=2)
                        {
                            var p=(double[])best.Clone();double tick=j==0?.001:.1;
                            p[j]=Math.Round(NativeAutoTune.Clamp(best[j]+sign*Math.Max(tick,fraction*best[j]),j==0?.001:(j==1?.1:0),j==0?1:(j==1?250:100)),j==0?3:1);
                            double value=score(p);if(value<bestCost){best=p;bestCost=value;improved=true;}
                        }
                        if(!improved)break;
                    }
                // Final one-tick neighborhood (including diagonal moves) at the returned point.
                for(int sweep=0;sweep<20&&!Limit(finalDeadline);sweep++)
                {
                    var center=(double[])best.Clone();bool improved=false;
                    for(int a=-1;a<=1;a++)for(int b=-1;b<=1;b++)for(int c=-1;c<=1;c++)
                    {
                        if(Limit(finalDeadline))break;
                        var p=new[]{Math.Round(NativeAutoTune.Clamp(center[0]+a*.001,.001,1),3),Math.Round(NativeAutoTune.Clamp(center[1]+b*.1,.1,250),1),Math.Round(NativeAutoTune.Clamp(center[2]+c*.1,0,100),1)};
                        double value=score(p);if(value<bestCost){best=p;bestCost=value;improved=true;}
                    }
                    if(!improved)break;
                }
                return best;
            }
            double started=timer.Elapsed.TotalSeconds;
            result.DeNominalPid=Optimize(false,result.GridPid,3,4);
            result.DeNominalCost=Eval(result.DeNominalPid); result.DeNominalRobust=Robust(result.DeNominalPid);
            result.NominalSeconds=timer.Elapsed.TotalSeconds-started;
            started=timer.Elapsed.TotalSeconds;
            var seed=result.DeNominalRobust<result.GridRobust?result.DeNominalPid:result.GridPid;
            result.Pid=Optimize(true,seed,8,10);
            result.PrimaryMinimum=Robust(result.Pid);
            if (motion != null)
            {
                var ranked=new List<Ranked>();
                foreach (var pair in robustCache)
                {
                    budget.Check();
                    if (pair.Value >= 1e10) continue;
                    var p=new[]{pair.Key.Item1,pair.Key.Item2,pair.Key.Item3};var m=motion(p);
                    if (!NativeLinear.Finite(m.Slew) || !NativeLinear.Finite(m.Effort)) continue;
                    ranked.Add(new Ranked { Pid=p, Robust=pair.Value, Nominal=Eval(p), Slew=m.Slew, Effort=m.Effort });
                }
                if (ranked.Count>0)
                {
                    var chosen=ChooseHierarchy(ranked);result.Pid=chosen.Pid;result.SelectedSlew=chosen.Slew;result.SelectedEffort=chosen.Effort;
                    result.Hierarchical=true;result.TrackingTolerance=TrackingTolerance;result.HierarchyCandidates=ranked.Count;
                    result.PrimaryMinimum=double.PositiveInfinity;
                    foreach (var row in ranked) result.PrimaryMinimum=Math.Min(result.PrimaryMinimum,row.Robust);
                }
            }
            result.RobustCost=Robust(result.Pid);result.NominalCost=Eval(result.Pid);
            result.RobustSeconds=timer.Elapsed.TotalSeconds-started;result.Seconds=timer.Elapsed.TotalSeconds;
            result.JointNeighborhood=joint;
            result.SelectedAxisCost=Neighborhood(result.Pid, Eval, false);
            result.SelectedJointCost=Neighborhood(result.Pid, Eval, true);
            result.BaselineJointCost=Neighborhood(original, Eval, true);
            result.Evaluations=cache.Count;result.BudgetReached=Limit(10);
            return result;
        }
    }
}
