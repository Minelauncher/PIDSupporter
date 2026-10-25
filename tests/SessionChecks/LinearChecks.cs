using System;
using System.Diagnostics;
using System.Threading;
using PIDSupporter;
internal static class LinearChecks
{
    internal static void Run()
    {
        var hierarchy = new System.Collections.Generic.List<PidSearch.Ranked> {
            new PidSearch.Ranked { Pid=new[]{.1,1.0,.1},Robust=1,Nominal=.8,Slew=10,Effort=1 },
            new PidSearch.Ranked { Pid=new[]{.2,1.0,.1},Robust=1.005,Nominal=.804,Slew=2,Effort=1 },
            new PidSearch.Ranked { Pid=new[]{.3,1.0,.1},Robust=1.014,Nominal=.8,Slew=0,Effort=0 },
            new PidSearch.Ranked { Pid=new[]{.4,1.0,.1},Robust=1.002,Nominal=.82,Slew=0,Effort=0 }
        };
        if(PidSearch.ChooseHierarchy(hierarchy).Pid[0]!=.2) throw new Exception("Hierarchy violated primary/secondary floors");
        hierarchy.Reverse();
        if(PidSearch.ChooseHierarchy(hierarchy).Pid[0]!=.2) throw new Exception("Tolerance chained or order-dependent");
        hierarchy.Add(new PidSearch.Ranked {Pid=new[]{.5,1.0,.1},Robust=1.004,Nominal=.803,Slew=2,Effort=.5});
        if(PidSearch.ChooseHierarchy(hierarchy).Pid[0]!=.5) throw new Exception("Effort tie-break failed");
        Console.WriteLine("PASS hierarchical floors, no epsilon chaining, effort tie-break");
        // Joint perturbations must include corners and subsume all axis probes.
        var point=new[]{.1,2.0,1.0};
        double Coupled(double[] v) => Math.Pow((v[0]-.1)/.01+(v[1]-2)/.2+(v[2]-1)/.1,2);
        double axis=PidSearch.Neighborhood(point,Coupled,false),joint=PidSearch.Neighborhood(point,Coupled,true);
        if(Math.Abs(axis-1)>1e-10 || Math.Abs(joint-9)>1e-10)throw new Exception("Joint neighborhood misses coupled corner");
        PidSearch.Neighborhood(new[]{.1,250.0,0.0}, v=> {if(v[1]!=250 || v[2]!=0)throw new Exception("Off mode perturbed");return 0;},true);
        Console.WriteLine("PASS joint corners, axis inclusion, off-mode preservation");
        // Deliberately optimistic isolated grid point versus a broad quadratic basin.
        double Landscape(double[] p) => p[0] == .04 && p[1] == 125 && p[2] == .1 ? 0 :
            1 + Math.Pow((p[0] - .15) / .1, 2) + Math.Pow(Math.Log(p[1] / 2.4), 2) + Math.Pow(p[2] - .06, 2);
        var searched = PidSearch.Run(new[] { .02, 250.0, 1.0 }, Landscape, new WorkBudget(CancellationToken.None), _ => {});
        if (searched.GridCost != 0 || searched.RobustCost >= searched.GridRobust || Math.Abs(searched.Pid[0]-.15) > .03)
            throw new Exception("Narrow false minimum was not avoided");
        if (searched.DeNominalCost > searched.GridCost || searched.RobustCost > searched.BaselineRobust)
            throw new Exception("Incumbent/grid retention failed");
        using (var cancel = new CancellationTokenSource())
        {
            int calls=0; bool stopped=false;
            try { PidSearch.Run(new[] {.02,250.0,1.0}, p => { if(++calls==20)cancel.Cancel();return 1; },new WorkBudget(cancel.Token), _=>{}); }
            catch(OperationCanceledException){stopped=true;}
            if(!stopped)throw new Exception("Search cancellation ignored");
        }
        foreach (var pid in new[] { searched.Pid, searched.DeNominalPid, searched.GridPid })
            if (Math.Abs(pid[0]*1000-Math.Round(pid[0]*1000))>1e-8 || Math.Abs(pid[1]*10-Math.Round(pid[1]*10))>1e-8 || Math.Abs(pid[2]*10-Math.Round(pid[2]*10))>1e-8)
                throw new Exception("Recommendation off slider grid");
        Console.WriteLine("PASS slider ticks; DE robust broad-basin preference, grid/incumbent retention, active search cancellation");
        var r = NativeChecks.Fixture(1200);
        var baseline = NativeLinear.Select(r, .025, new WorkBudget(CancellationToken.None), _ => {});
        if (baseline.Score > 1e-5 || baseline.FinalWorst > 1e-5) throw new Exception("Known first-order closed/free reproduction failed");
        for (int i = 800; i < 1200; i++) { r.Y[i] += 50; r.U[i] = -1; r.R[i] += 10; }
        var changed = NativeLinear.Select(r, .025, new WorkBudget(CancellationToken.None), _ => {});
        if (baseline.Score != changed.Score || baseline.Model.Order != changed.Model.Order) throw new Exception("Final data leaked into selection");
        for (int i = 0; i < baseline.Model.Coefficients.Length; i++)
            if (baseline.Model.Coefficients[i] != changed.Model.Coefficients[i]) throw new Exception("Final data leaked into coefficients");
        bool reachedSearch = false;
        NativeAutoTune.Run(r, CancellationToken.None, text => { if (text == "PID 후보 평가 중") reachedSearch = true; });
        if (!reachedSearch || changed.FinalWorst <= .5) throw new Exception("Finite holdout mismatch still blocks PID search");
        Console.WriteLine("PASS free + closed reproduction; final holdout cannot change order/coefficients; finite mismatch reaches PID search");
        foreach (int count in new[] { 1200, 6000 })
        {
            var timer = Stopwatch.StartNew(); long before = GC.GetTotalAllocatedBytes(true);
            var result = NativeAutoTune.Run(NativeChecks.Fixture(count), CancellationToken.None, _ => {});
            if (!NativeLinear.Finite(result.validation) || result.validation > 1e-5) throw new Exception("Large-record regression");
            Console.WriteLine($"PASS {count} samples: {timer.Elapsed.TotalSeconds:F3}s, {GC.GetTotalAllocatedBytes(true)-before} bytes, score {result.validation:E3}");
        }
    }
}
