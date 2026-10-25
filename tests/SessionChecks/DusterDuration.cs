using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text.Json;
using System.Threading;
using PIDSupporter;

// Offline duration ablation. Same production REN, normalization, Adam and seeds.
internal static class DusterDuration
{
    internal static void Run()
    {
        string folder = "tests/output/duster-diagnostic";
        var events = File.ReadLines(folder + "/source.jsonl").Select(l => JsonDocument.Parse(l).RootElement.Clone()).ToArray();
        var record = events.Last(e => e.GetProperty("kind").GetString() == "record").GetProperty("value");
        double[] ArrayOf(string key) => record.GetProperty(key).EnumerateArray().Select(x => x.GetDouble()).ToArray();
        double[] actual = ArrayOf("Y"), input = ArrayOf("U"); int n = actual.Length, fit = record.GetProperty("fit").GetInt32();
        double um = record.GetProperty("um").GetDouble(), us = record.GetProperty("us").GetDouble();
        double ym = record.GetProperty("ym").GetDouble(), ys = record.GetProperty("ys").GetDouble();
        var u = new double[n]; var y = new double[n];
        for (int i=0;i<n;i++) { u[i]=i==0?0:(input[i-1]-um)/us; y[i]=(actual[i]-ym)/ys; }
        int[] limits = {2000,5000,10000};
        var records = new List<object>(); var checkpoints = new List<object>();
        for(int seed=0;seed<2;seed++)
        {
            var net=new NativeRen(n); net.Initialize(9701+seed);
            var m=new double[NativeRen.ParameterCount]; var v=new double[m.Length];
            var bestParameters=new double[m.Length]; double best=double.PositiveInfinity;int bestStep=0;
            double b1Power=1,b2Power=1;var timer=Stopwatch.StartNew();var budget=new WorkBudget(CancellationToken.None);
            for(int step=0;step<=10000;step++)
            {
                budget.Check();
                if(step%100==0)
                {
                    double validation=net.Validation(u,y,fit,CancellationToken.None);
                    var pred=net.Predictions(n);double train=Math.Sqrt(pred.Take(fit).Select((a,i)=>(a-y[i])*(a-y[i])).Average());
                    checkpoints.Add(new {seed,step,train,validation,seconds=timer.Elapsed.TotalSeconds});
                    if(validation<best){best=validation;bestStep=step;Array.Copy(net.Parameters,bestParameters,m.Length);}
                    if(limits.Contains(step))
                    {
                        var current=(double[])net.Parameters.Clone();Array.Copy(bestParameters,net.Parameters,m.Length);
                        net.Validation(u,y,fit,CancellationToken.None);
                        var prediction=net.Predictions(n).Select(a=>a*ys+ym).ToArray();
                        records.Add(new {seed,updates=step,bestStep,validation=best,seconds=timer.Elapsed.TotalSeconds,predictedY=prediction});
                        Array.Copy(current,net.Parameters,m.Length);
                        Console.WriteLine($"seed={seed} updates={step} bestStep={bestStep} validation={best:F6} time={timer.Elapsed.TotalSeconds:F2}s");
                    }
                }
                if(step==10000)break;
                double loss=net.Loss(u,y,fit,true,CancellationToken.None);
                double norm=net.Gradient.Sum(g=>g*g);
                if(!NativeRen.Finite(loss)||!NativeRen.Finite(norm))throw new Exception("Nonfinite training");
                double scale=norm>100?10/Math.Sqrt(norm):1;b1Power*=.9;b2Power*=.999;
                for(int j=0;j<m.Length;j++)
                {
                    double g=net.Gradient[j]*scale;m[j]=.9*m[j]+.1*g;v[j]=.999*v[j]+.001*g*g;
                    net.Parameters[j]-=.003*(m[j]/(1-b1Power))/(Math.Sqrt(v[j]/(1-b2Power))+1e-8);
                }
            }
        }
        File.WriteAllText(folder+"/duration.json",JsonSerializer.Serialize(new {records,checkpoints}));
    }
}
