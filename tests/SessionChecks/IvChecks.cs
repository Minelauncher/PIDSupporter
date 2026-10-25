using System;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Threading;
using System.Diagnostics;
using PIDSupporter;

internal static class IvChecks
{
    static double[] A(JsonElement e, string key) => e.GetProperty(key).EnumerateArray().Select(v => v.GetDouble()).ToArray();
    internal static void Run()
    {
        string src = "research_lti/results/negative_gain_diagnosis_20261020";
        var e = File.ReadLines(src + "/source.jsonl").Select(l => JsonDocument.Parse(l).RootElement.Clone())
            .First(x => x.GetProperty("kind").GetString() == "record").GetProperty("value");
        var r = new TuneRecord { U=A(e,"U"), Y=A(e,"Y"), R=A(e,"R"), T=A(e,"T"), Integrals=A(e,"Integrals"),
            Pid=A(e,"Pid"), Speed=.5, Amplitude=e.GetProperty("Amplitude").GetDouble(),
            Lower=e.GetProperty("Lower").GetDouble(), Upper=e.GetProperty("Upper").GetDouble(),
            InitialIntegral=e.GetProperty("InitialIntegral").GetDouble(), InitialError=e.GetProperty("InitialError").GetDouble() };
        var expected = JsonDocument.Parse(File.ReadAllText(src+"/results.json")).RootElement.EnumerateArray()
            .First(x => x.GetProperty("group").GetString()=="iv2_cut1e-2");
        var watch=Stopwatch.StartNew();
        var s=NativeLinear.Select(r,.025,new WorkBudget(CancellationToken.None),_=>{});
        double seconds=watch.Elapsed.TotalSeconds;
        if(s.Model.Order!=expected.GetProperty("order").GetInt32() || Math.Abs(s.Score-expected.GetProperty("score").GetDouble())>1e-6)
            throw new Exception("Independent NumPy SVD reference mismatch");
        var coeff=A(expected,"coeff");
        for(int i=0;i<coeff.Length;i++) if(Math.Abs(coeff[i]-s.Model.Coefficients[i])>1e-6) throw new Exception("IV coefficients mismatch");
        var result=NativeAutoTune.Run(r,CancellationToken.None,_=>{});
        if(!result.pid!.SequenceEqual(new[]{.065,.3,.2})) throw new Exception("Duster IV recommendation changed");
        Console.WriteLine(JsonSerializer.Serialize(new {seconds,order=s.Model.Order,score=s.Score,result}));
        // Instrument scaling and offset must not change the projector.
        var original=(double[])r.R.Clone();
        for(int i=0;i<r.R.Length;i++) r.R[i]=original[i]*3+20;
        // Model selection uses SP in the controller too; compare projection directly instead.
        var method=typeof(NativeLinear).GetMethod("Instruments",System.Reflection.BindingFlags.NonPublic|System.Reflection.BindingFlags.Static)!;
        var q=(double[,])method.Invoke(null,new object[]{original,80,600,80,new WorkBudget(CancellationToken.None)})!;
        var qs=(double[,])method.Invoke(null,new object[]{r.R,80,600,80,new WorkBudget(CancellationToken.None)})!;
        for(int i=0;i<520;i+=17) for(int j=0;j<520;j+=19)
        {
            double a=0,b=0; for(int k=0;k<q.GetLength(1);k++) a+=q[i,k]*q[j,k];
            for(int k=0;k<qs.GetLength(1);k++) b+=qs[i,k]*qs[j,k];
            if(Math.Abs(a-b)>1e-8) throw new Exception("Instrument scaling changes projector");
        }
        bool rejected=false;
        try { method.Invoke(null,new object[]{new double[1200],80,600,80,new WorkBudget(CancellationToken.None)}); }
        catch(System.Reflection.TargetInvocationException x) when(x.InnerException is ArgumentException) { rejected=true; }
        if(!rejected) throw new Exception("Constant SP accepted");
        Console.WriteLine("PASS independent NumPy IV coefficients/PID, projector scale invariance, constant SP rejection");
    }
}
