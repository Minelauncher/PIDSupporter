using System;
using System.Diagnostics;
using System.IO;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using PIDSupporter;
using BrilliantSkies.Ai.Control.Pids;

internal static class ExcitationChecks
{
    internal static void Run()
    {
        var rows = new List<object>();
        foreach (var pid in new[] { new[]{.02,250.0,1.0},new[]{.111,.5,.3},new[]{.051,250.0,1.0},new[]{.001,.1,100.0},new[]{1.0,250.0,0.0} })
        {
            var c=new VariableControllerMaster();c.Pid.kP.Us=(float)pid[0];c.Pid.kI.Us=(float)pid[1];c.Pid.kD.Us=(float)pid[2];
            var timer=Stopwatch.StartNew();var s=new AutoTuneSession(c,.5f);double startMs=timer.Elapsed.TotalMilliseconds;
            var rng=new Random(826);var phases=Enumerable.Range(0,60).Select(_=>rng.NextDouble()*2*Math.PI).ToArray();
            var before=new double[1200];var after=new double[1200];timer.Restart();
            for(int k=0;k<1200;k++)
            {
                double t=k*.025,sum=0;
                for(int j=0;j<60;j++) {double w=2*Math.PI*(j+1)/30,reactive=w*s.Original[2]-(s.Original[1]>=250?0:1/(w*Math.Max(.01,s.Original[1])));sum+=Math.Sin(w*t+phases[j])/Math.Sqrt(1+reactive*reactive);}
                before[k]=s.Amplitude*Math.Max(-1,Math.Min(1,sum/20));after[k]=s.Wave(t);
            }
            double loopMs=timer.Elapsed.TotalMilliseconds;
            if(after[0]!=0 || s.Wave(30)!=0 || after.Any(v=>Math.Abs(v)>s.Amplitude) || Math.Abs(after.Average())>1e-5)
                throw new Exception("Waveform endpoint, bound or zero mean regression");
            // Check actual sampled low-band SP content, not just internal weights.
            double LowRms(double[] a) {double power=0;for(int j=1;j<=6;j++){double re=0,im=0;for(int k=0;k<a.Length;k++){double w=2*Math.PI*j*k/a.Length;re+=a[k]*Math.Cos(w);im+=a[k]*Math.Sin(w);}power+=2*(re*re+im*im)/(a.Length*a.Length);}return Math.Sqrt(power);}
            double oldLow=LowRms(before),newLow=LowRms(after);
            if(pid[0]==.111 && newLow<=oldLow)throw new Exception($"Integral-control low-band excitation not improved: {oldLow} -> {newLow}");
            rows.Add(new{pid,oldLow,newLow,oldStart=before[0],newStart=after[0],newEnd=after[1199],mean=after.Average(),peak=after.Max(v=>Math.Abs(v)),startMs,oldAndNew1200SamplesMs=loopMs});
            s.Cancel();
        }
        string root="tests/output/excitation-wave-v2-20261020";Directory.CreateDirectory(root);
        var json=JsonSerializer.Serialize(rows,new JsonSerializerOptions{WriteIndented=true});File.WriteAllText(root+"/checks.json",json);Console.WriteLine(json);
    }
}
