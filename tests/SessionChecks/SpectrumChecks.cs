using System;
using System.Linq;
using PIDSupporter;
internal static class SpectrumChecks
{
 static void Check(bool ok,string message){if(!ok)throw new Exception(message);}
 static TuneRecord Record()=>new TuneRecord{U=new double[1200],Y=new double[1200],InjectedSP=new double[1200],T=Enumerable.Range(0,1200).Select(i=>(i+1)*.025).ToArray(),Pid=new[]{.1,250.0,0.0},Speed=.5,Amplitude=1,Lower=-1,Upper=1};
 internal static void Run()
 {
  var r=Record();for(int i=0;i<1200;i++){double v=Math.Sin(2*Math.PI*.5*r.T[i]);r.InjectedSP[i]=v;r.U[i]=.5*v;r.Y[i]=3*v+10;}
  var s=AutoTuneSpectrum.Analyze(r);int peak=Array.IndexOf(s.Power[0],s.Power[0].Max());Check(Math.Abs(s.Frequency[peak]-.5)<1e-8,"tone location");
  double rms=Math.Sqrt(s.BandRms[0].Sum(x=>x*x));Check(Math.Abs(rms-1/Math.Sqrt(2))<.002,"PSD RMS normalization");Check(Math.Abs(s.BandRms[2][2]/s.BandRms[0][2]-3)<.001,"amplitude and offset handling");
  Check(s.Windows==5&&s.Coherence[0][4]>.999&&s.Coherence[1][4]>.999,"coherent response");Check(double.IsNaN(s.Coherence[0][19]),"unexcited bins hidden");
  foreach(double baseTime in new[]{500.0,1500.0})
  {
   var boundary=Record();for(int i=0;i<1200;i++){double v=Math.Sin(2*Math.PI*.1*(i+1)*.025)+Math.Sin(2*Math.PI*.3*(i+1)*.025)+Math.Sin(2*Math.PI*(i+1)*.025);boundary.InjectedSP[i]=v;boundary.U[i]=v/3;boundary.Y[i]=v;}
   var reference=AutoTuneSpectrum.Analyze(boundary);
   for(int i=0;i<1200;i++)boundary.T[i]=(double)(float)(baseTime+(i+1)*.025)-(double)(float)baseTime;
   var quantized=AutoTuneSpectrum.Analyze(boundary);
   for(int b=0;b<4;b++)Check(Math.Abs(reference.BandRms[0][b]-quantized.BandRms[0][b])<1e-7,"float timestamp band boundaries");
  }
  var empty=AutoTuneSpectrum.Analyze(Record());Check(empty.Coherence.All(a=>a.All(double.IsNaN)),"zero signal is unknown, not high quality");
  var noise=Record();var random=new Random(5721);for(int i=0;i<1200;i++){noise.InjectedSP[i]=random.NextDouble()*2-1;noise.U[i]=random.NextDouble()*2-1;noise.Y[i]=random.NextDouble()*2-1;}
  var ns=AutoTuneSpectrum.Analyze(noise);double mean=ns.Coherence[1].Where(x=>!double.IsNaN(x)).Average();Check(mean<.65,"independent noise coherence");
  Console.WriteLine($"Spectrum checks passed: peak .5Hz, RMS {rms:F6}, coherent transfer, missing-data masking, independent-noise coherence {mean:F3}.");
 }
}
