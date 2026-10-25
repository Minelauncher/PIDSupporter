using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Reflection;
using System.Threading;
using HarmonyLib;
using Newtonsoft.Json;
using UnityEngine;

namespace PIDSupporter
{
    // Bounded, read-only telemetry. No JSON, filesystem or Unity calls on the writer thread.
    internal static class PhysicsTrace
    {
        private const int Width = 40, Capacity = 100000;
        private static readonly object Gate = new object();
        private static readonly Dictionary<(Type,string), MemberInfo?> Members = new Dictionary<(Type,string), MemberInfo?>();
        private static Run? current;
        private static object? timer;
        private static bool ready;
        private static readonly string[] PowerFields={"IdealPower","PowerAllocated","PowerUsed","FractionOfPowerRequestedThatWasProvided","PowerUsePerFixedStep"};
        private static double[]? spare;
        private static void WarmBuffer()=>ThreadPool.QueueUserWorkItem(_=>
        {
            var data=new double[Width*Capacity];
            // Fault pages before collection rather than on the physics thread.
            for(int i=0;i<data.Length;i+=512)data[i]=double.NaN;
            Interlocked.CompareExchange(ref spare,data,null);
        });
        private sealed class Run
        {
            internal string Id = "";
            internal object Construct = null!, Platform = null!, Forces = null!;
            internal readonly double[] Data;
            internal Run(double[] data){Data=data;}
            internal readonly Dictionary<object,int> Objects = new Dictionary<object,int>(ReferenceComparer.Instance);
            internal readonly List<object> Metadata = new List<object>();
            internal readonly Queue<int> Pending = new Queue<int>();
            internal readonly Dictionary<int,double> ClearanceFrames = new Dictionary<int,double>();
            internal int Count, Dropped, Errors, Sequence, ApplyCount;
            internal long CaptureTicks, MaxCaptureTicks;
            internal double Dt;
        }
        private sealed class ReferenceComparer : IEqualityComparer<object>
        {
            internal static readonly ReferenceComparer Instance = new ReferenceComparer();
            public new bool Equals(object? a,object? b) => ReferenceEquals(a,b);
            public int GetHashCode(object x) => System.Runtime.CompilerServices.RuntimeHelpers.GetHashCode(x);
        }
        private static object? Read(object? value,string name)
        {
            if(value==null)return null;
            var key=(value.GetType(),name);
            if(!Members.TryGetValue(key,out var member))
            {
                for(var t=key.Item1;t!=null && member==null;t=t.BaseType)
                    member=(MemberInfo?)t.GetProperty(name,BindingFlags.Instance|BindingFlags.Public|BindingFlags.NonPublic|BindingFlags.DeclaredOnly)
                        ?? t.GetField(name,BindingFlags.Instance|BindingFlags.Public|BindingFlags.NonPublic|BindingFlags.DeclaredOnly);
                Members[key]=member;
            }
            try{return member is PropertyInfo p?p.GetValue(value,null):(member as FieldInfo)?.GetValue(value);}catch{return null;}
        }
        private static double N(object? v)
        { try { if(v==null)return double.NaN; if(v is Enum)return Convert.ToInt32(v); return Convert.ToDouble(v); }catch{return double.NaN;} }
        private static double N(object? v,string key)=>N(Read(v,key));
        private static Type? TypeNamed(string name)=>AccessTools.TypeByName(name);
        private static void Patch(Harmony h,Type? t,string method,string? pre,string? post)
        {
            var m=t==null?null:AccessTools.Method(t,method);
            if(m==null)throw new MissingMethodException(t?.FullName,method);
            h.Patch(m,pre==null?null:new HarmonyMethod(typeof(PhysicsTrace),pre),post==null?null:new HarmonyMethod(typeof(PhysicsTrace),post));
        }
        internal static void Install(Harmony h)
        {
            try
            {
                var platform=TypeNamed("BrilliantSkies.Ftd.Constructs.Modules.Main.PlatformPhysics.PlatformPhysics");
                var forces=TypeNamed("BrilliantSkies.Common.Forces.VehicleForces");
                Patch(h,platform,"Perform",nameof(BeforeStep),nameof(AfterStep));
                Patch(h,forces,"CalculateRequestedForces",null,nameof(ForcesCalculated));
                Patch(h,forces,"AddDragForceToCalculation",nameof(BeforeDrag),null);
                Patch(h,forces,"ApplyRequestedForce",nameof(Applied),null);
                // Capture the actual responder argument, not the PID proxy. Includes separate tailplane axes.
                var game=platform!.Assembly;
                int responses=0;
                foreach(var t in game.GetTypes())
                    foreach(var m in t.GetMethods(BindingFlags.Instance|BindingFlags.Public|BindingFlags.NonPublic|BindingFlags.DeclaredOnly))
                    {
                        var p=m.GetParameters();
                        if(!m.IsAbstract && m.Name.StartsWith("AssessResponse",StringComparison.Ordinal) && p.Length==2 && p[1].ParameterType==typeof(float))
                        { h.Patch(m,postfix:new HarmonyMethod(typeof(PhysicsTrace),nameof(Response))); responses++; }
                    }
                var tt=TypeNamed("BrilliantSkies.Core.Timing.GameTimer");
                timer=tt?.GetProperty("Instance",BindingFlags.Public|BindingFlags.Static|BindingFlags.FlattenHierarchy)?.GetValue(null);
                ready=true; AutoTuneContext.BeginPhysics=Begin; AutoTuneContext.EndPhysics=End;
                WarmBuffer();
                AutoTuneDiagnostics.Event("physicsTraceHooks",new { ready, responses, capacity=Capacity });
            }
            catch(Exception e){AutoTuneDiagnostics.Event("physicsTraceHooks",new {ready=false,error=e.ToString()});}
        }
        private static void Begin(object construct,string id)
        {
            if(!ready)return;
            var data=Interlocked.Exchange(ref spare,null);
            if(data==null){WarmBuffer();AutoTuneDiagnostics.Event("physicsTraceSkipped",new {runId=id,reason="Buffer not ready"});return;}
            var run=new Run(data) {Id=id,Construct=construct};
            lock(Gate)
            {
                run.Platform=Read(construct,"PlatformPhysics")!;run.Forces=Read(run.Platform,"Forces")!;
                if(run.Platform==null || run.Forces==null)return;
                // A previous run must never be silently merged with another axis.
                if(current!=null)Finish(current);
                current=run;
            }
        }
        private static int Row(Run r,int kind,int sequence)
        {
            if(r.Count==Capacity){r.Dropped++;return -1;}
            int start=r.Count++*Width;
            for(int i=0;i<Width;i++)r.Data[start+i]=double.NaN;
            r.Data[start]=kind;r.Data[start+1]=sequence;r.Data[start+2]=N(timer,"FrameCounter");
            r.Data[start+3]=Stopwatch.GetTimestamp();r.Data[start+4]=r.Dt;
            return start;
        }
        private static void V(Run r,int index,object? v)
        { r.Data[index]=N(v,"x");r.Data[index+1]=N(v,"y");r.Data[index+2]=N(v,"z"); }
        private static int Identify(Run r,object obj,string? method=null)
        {
            if(r.Objects.TryGetValue(obj,out int id))return id;
            id=r.Objects.Count;r.Objects.Add(obj,id);
            var p=Read(obj,"LocalPositionInMainConstruct")??Read(obj,"LocalPosition");
            r.Metadata.Add(new {id,type=obj.GetType().FullName,firstMethod=method,position=new[]{N(p,"x"),N(p,"y"),N(p,"z")}});
            return id;
        }
        private static void Guard(Action<Run> action)
        {
            lock(Gate)
            {
                var r=current;if(r==null)return;long start=Stopwatch.GetTimestamp();
                try{action(r);}catch{r.Errors++;}
                long ticks=Stopwatch.GetTimestamp()-start;r.CaptureTicks+=ticks;r.MaxCaptureTicks=Math.Max(r.MaxCaptureTicks,ticks);
            }
        }
        private static void BeforeStep(object __instance,object __0)=>Guard(r=>
        {
            if(!ReferenceEquals(__instance,r.Platform) || !(Read(r.Platform,"Active") is bool active) || !active)return;
            r.Sequence++;r.Dt=N(__0,"DeltaTime");int s=Row(r,1,r.Sequence);if(s<0)return;
            var part=Read(r.Platform,"OurPhysicsModule");var v=Read(part,"iVelocities");
            V(r,s+5,Read(v,"VelocityVector"));V(r,s+8,Read(v,"LocalAngularVelocityVector"));
            var q=Read(r.Construct,"SafeRotation");V(r,s+11,q);r.Data[s+14]=N(q,"w");
            var mass=Read(part,"TotalMass");V(r,s+15,Read(mass,"LCOM"));V(r,s+18,Read(mass,"GCOM"));
            r.Data[s+21]=N(Read(mass,"TotalMass"),"UnitySimulationMass");
        });
        private static void ForcesCalculated(object __instance,Vector3 __0,Vector3 __1,Quaternion __2)=>Guard(r=>
        {
            if(!ReferenceEquals(__instance,r.Forces))return;
            var all=Read(__instance,"LastForces") as IList;int count=(int)N(__instance,"LastForceCount");
            if(all==null)return;
            for(int i=0;i<count;i++)
            {
                var force=all[i];if(force==null)continue;int s=Row(r,2,r.Sequence);if(s<0)break;
                r.Data[s+5]=N(force,"Type");V(r,s+6,Read(force,"Force"));V(r,s+9,Read(force,"_localPosition"));
                r.Data[s+19]=N(force,"Frame");
                V(r,s+12,Read(force,"ExcessTorque"));r.Data[s+15]=N(force,"ControlSurfaceForceMagnitude");
                // This read-only game method computes torque into a local ref value.
                var method=AccessTools.Method(force.GetType(),"AddTorque");
                if(method!=null){object[] args={__1,__0,__2,Vector3.zero};method.Invoke(force,args);V(r,s+16,args[3]);}
            }
        });
        private static void BeforeDrag(object __instance,object __0)=>Guard(r=>
        {
            if(!ReferenceEquals(__instance,r.Forces))return;
            r.Pending.Enqueue(r.Sequence);
            int s=Row(r,3,r.Sequence);if(s<0)return;
            var total=Read(r.Forces,"CalculatedForce");V(r,s+5,Read(total,"Force"));V(r,s+8,Read(total,"Torque"));
            V(r,s+11,Read(__0,"GlobalDragForces"));r.Data[s+14]=N(total,"ControlSurfaceForceMagnitude");
        });
        private static void AfterStep(object __instance)=>Guard(r=>
        {
            if(!ReferenceEquals(__instance,r.Platform))return;int s=Row(r,4,r.Sequence);
            if(s>=0){var total=Read(r.Forces,"CalculatedForce");V(r,s+5,Read(total,"Force"));V(r,s+8,Read(total,"Torque"));r.Data[s+11]=N(r.Platform,"AltitudeThrustModifier");}
            if(Read(r.Platform,"ForceCreators") is IEnumerable creators)
            foreach(var obj in creators)
            {
                if(obj==null)continue;s=Row(r,5,r.Sequence);if(s<0)break;
                r.Data[s+5]=Identify(r,obj);r.Data[s+6]=N(obj,"_rawControlInput");r.Data[s+7]=N(obj,"SpinSpeed");r.Data[s+8]=N(obj,"Direction");
                r.Data[s+9]=N(Read(obj,"PowerScaleLerp"),"Current");r.Data[s+10]=N(Read(obj,"LerpController"),"Current");
                r.Data[s+11]=N(obj,"MaxThrust");r.Data[s+12]=N(obj,"ClearanceFactor");r.Data[s+13]=N(obj,"TotalArea");r.Data[s+14]=N(obj,"EffectiveArea");
                var prop=Read(obj,"Propulsion");r.Data[s+15]=N(prop,"PlacementForceFraction");
                var power=Read(obj,"Power");
                for(int i=0;i<PowerFields.Length;i++)r.Data[s+16+i]=N(power,PowerFields[i]);
                var swept=Read(obj,"SweptArea");r.Data[s+21]=N(swept,"LastClearanceChangeFrame");r.Data[s+22]=N(swept,"ClearanceCheckCounter");
                V(r,s+23,Read(obj,"GameWorldForwards"));r.Data[s+26]=N(obj,"IsRotationBlocked");r.Data[s+27]=N(obj,"IsFeelerDone");
                r.Data[s+28]=N(obj,"_clearanceCheckIteration");r.Data[s+29]=N(swept,"_currentCell");r.Data[s+30]=N(obj,"_repeatLastCell");
                int id=(int)r.Data[s+5];double change=r.Data[s+21];
                if(swept!=null && (!r.ClearanceFrames.TryGetValue(id,out double previous) || change!=previous))
                {
                    r.ClearanceFrames[id]=change;
                    if(Read(swept,"Cells") is IList cells)for(int i=0;i<cells.Count;i++)
                    {
                        var cell=cells[i];if(cell==null)continue;int row=Row(r,8,r.Sequence);if(row<0)break;
                        r.Data[row+5]=id;r.Data[row+6]=i;V(r,row+7,Read(cell,"Position"));r.Data[row+10]=N(cell,"ClearanceFactor");
                        r.Data[row+11]=N(cell,"TopClearance");r.Data[row+12]=N(cell,"BottomClearance");
                        r.Data[row+13]=N(cell,"TopCounter");r.Data[row+14]=N(cell,"BottomCounter");r.Data[row+15]=N(cell,"OverlapLoss");
                    }
                }
            }
        });
        private static void Response(object __instance,float __1,MethodBase __originalMethod)=>Guard(r=>
        {
            if(!ReferenceEquals(Read(__instance,"MainConstruct"),r.Construct))return;
            int s=Row(r,6,r.Sequence);if(s<0)return;r.Data[s+5]=Identify(r,__instance,__originalMethod.Name);r.Data[s+6]=__1;
            r.Data[s+7]=__originalMethod.Name.EndsWith("Pitch",StringComparison.Ordinal)?1:__originalMethod.Name.EndsWith("Yaw",StringComparison.Ordinal)?2:0;
        });
        private static void Applied(object __instance,Rigidbody __0,object __1,bool __2)=>Guard(r=>
        {
            if(!ReferenceEquals(__instance,r.Forces))return;int seq=r.Pending.Count>0?r.Pending.Dequeue():-1;r.ApplyCount++;
            int s=Row(r,7,seq);if(s<0)return;V(r,s+5,Read(__1,"Force"));V(r,s+8,Read(__1,"Torque"));
            V(r,s+11,__0.velocity);V(r,s+14,__0.angularVelocity);V(r,s+17,__0.rotation);r.Data[s+20]=__0.rotation.w;
            V(r,s+21,__0.inertiaTensor);V(r,s+24,__0.inertiaTensorRotation);r.Data[s+27]=__0.inertiaTensorRotation.w;
            r.Data[s+28]=__0.mass;r.Data[s+29]=__0.angularDrag;V(r,s+30,__0.centerOfMass);
            r.Data[s+33]=__2?1:0;var client=Read(r.Forces,"ClientInterpolatedForce");V(r,s+34,Read(client,"Force"));V(r,s+37,Read(client,"Torque"));
        });
        private static void End(string id){lock(Gate){if(current?.Id!=id)return;var r=current;current=null;Finish(r);}}
        private static void Finish(Run r)
        {
            string directory=Path.Combine(Path.GetDirectoryName(typeof(PhysicsTrace).Assembly.Location)!,"Diagnostics");
            new Thread(()=>
            {
                try
                {
                    Directory.CreateDirectory(directory);
                    string path=Path.Combine(directory,"physics-"+string.Join("_",r.Id.Split(Path.GetInvalidFileNameChars()))+".jsonl");
                    using(var sw=new StreamWriter(path,false))
                    {
                        sw.WriteLine(JsonConvert.SerializeObject(new {kind="header",schema=1,runId=r.Id,rows=r.Count,dropped=r.Dropped,errors=r.Errors,steps=r.Sequence,applications=r.ApplyCount,unmatched=r.Pending.Count,objects=r.Metadata,
                            captureTotalMs=r.CaptureTicks*1000.0/Stopwatch.Frequency,captureMaxHookMs=r.MaxCaptureTicks*1000.0/Stopwatch.Frequency,stopwatchFrequency=Stopwatch.Frequency,
                            common="kind,calculationSequence,gameFrame,stopwatchTicks,dt; missing fields are NaN strings",
                            kinds=new {step="1: worldVelocity[5:8],localOmega[8:11],quaternionXYZW[11:15],LCOM[15:18],GCOM[18:21],mass[21]",force="2: type[5],worldForce[6:9],localPoint[9:12],excessTorque[12:15],surfaceMagnitude[15],worldTorque[16:19]",beforeDrag="3: worldForce[5:8],worldTorque[8:11],worldSkinDrag[11:14],surfaceMagnitude[14]",calculated="4: worldForce[5:8],worldTorque[8:11],airFactor[11]",actuator="5: id[5],request[6],spin[7],direction[8],powerScaleRamp[9],commandRamp[10],maxThrust[11],clearance[12],area[13],effectiveArea[14],placement[15],ideal/allocated/used/fraction/maxPower[16:21],clearanceFrame[21],scanBudget[22],worldForward[23:26],blocked[26],feelerDone[27]",response="6: id[5],deliveredRequest[6],axisSuffix0generic1pitch2yaw[7]; sequence is latest calculation at callback, not assumed consumption",application="7: queuedForce[5:8],queuedTorque[8:11],preIntegrationWorldVelocity[11:14],preIntegrationWorldOmega[14:17],rotation[17:21],inertia[21:24],inertiaRotation[24:28],mass[28],angularDrag[29],LCOM[30:33]"},
                            notes="Read-only snapshots. Application pre-state becomes next-step evidence; no claim of post-integration sampling. Buffered during collection, written after stop. Limits/errors explicit. Force positions may be missing for force subclasses; actual AddTorque result is recorded."}));
                        for(int i=0;i<r.Count;i++)sw.WriteLine(JsonConvert.SerializeObject(new ArraySegment<double>(r.Data,i*Width,Width)));
                    }
                    AutoTuneDiagnostics.Event("physicsTraceSaved",new {runId=r.Id,path,rows=r.Count,dropped=r.Dropped,errors=r.Errors});
                }
                catch(Exception e){AutoTuneDiagnostics.Event("physicsTraceWriteFailed",new {runId=r.Id,error=e.Message});}
                finally { Interlocked.CompareExchange(ref spare,r.Data,null); }
            }) { IsBackground=true,Priority=System.Threading.ThreadPriority.BelowNormal,Name="PID physics diagnostic writer" }.Start();
        }
    }
}
