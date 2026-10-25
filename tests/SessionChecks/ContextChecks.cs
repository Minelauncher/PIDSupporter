using System;
using System.Collections.Generic;
using System.Text.Json;
using PIDSupporter;
using BrilliantSkies.Ai.Control.Pids;
internal static class ContextChecks
{
 static void Check(bool ok,string msg){if(!ok)throw new Exception(msg);}
 internal static void Run()
 {
  var roll=new VariableControllerMaster{LastControlVariable=2.5f};
  var pitch=new VariableControllerMaster{LastControlVariable=-.7f};
  var vector=new{x=1.0,y=2.0,z=3.0};
  object Vehicle(int id,object common)=>new{
   UniqueId=id,blueprintName="context-test",AltitudeOfComAboveMeanSeaLevel=1000.0,
   Pitch=2.0,Yaw=3.0,Roll=4.0,
   NodeSets=new{AiNodes=new{Nodes=new[]{new{Master=new{Common=common}}}}},
   PlatformPhysics=new{AltitudeThrustModifier=.125,LocalDragForce=vector},
   PartPhysics=new{iVelocities=new{VelocityMagnitude=20.0,LocalVelocityVector=vector,LocalAngularVelocityVector=vector}}
  };
  var vehicle=Vehicle(42,new{RollControl=roll,PitchControl=pitch});
  var wrong=Vehicle(99,new{RollControl=new VariableControllerMaster()});
  var events=new List<JsonElement>();
  AutoTuneDiagnostics.Sink=(k,v)=>{if(k=="operatingContext")events.Add(JsonSerializer.SerializeToElement(v));};
  var context=new AutoTuneContext(roll,"run-roll");context.Resolve(new[]{wrong,vehicle});
  for(int i=1;i<=1200;i++)context.Capture(i*.025);
  context.Finish("complete");context.Finish("again");
  Check(events.Count==1,"idempotent finish");
  var e=events[0];Check(e.GetProperty("axis").GetString()=="Roll","matched controller identity");
  Check(e.GetProperty("constructId").GetInt32()==42,"correct vehicle");
  Check(e.GetProperty("rawCommands").GetArrayLength()==1200,"full rate raw samples");
  Check(e.GetProperty("rawCommands")[0].GetDouble()==2.5,"preserve preclamp request");
  Check(e.GetProperty("physics").GetArrayLength()==150,"5 Hz physical samples");
  Check(e.GetProperty("physics")[0].GetProperty("airDensityModifier").GetDouble()==.125,"density");
  Check(roll.LastControlVariable==2.5f,"read only");
  var cp=new AutoTuneContext(pitch,"run-pitch");cp.Resolve(new[]{vehicle});cp.Capture(.025);cp.Finish("cancel");
  Check(events[1].GetProperty("axis").GetString()=="Pitch","pitch not confused with roll");
  var missing=new AutoTuneContext(new VariableControllerMaster(),"missing");missing.Capture(.025);missing.Finish("cancel");
  Check(!events[2].GetProperty("ownerResolved").GetBoolean(),"unresolved explicit");
  Check(events[2].GetProperty("physics")[0].GetProperty("airDensityModifier").ValueKind==JsonValueKind.Null,"missing not zero");
  AutoTuneDiagnostics.Sink=null;
  Console.WriteLine("Context checks passed: axis/vehicle matching, pre-clamp output, sampling, cancellation, missing data, no mutation.");
 }
}
