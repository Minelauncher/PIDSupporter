using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using System.Text.Json;
using PIDSupporter;

internal static class DusterLandscape
{
    static double[] A(JsonElement e,string key)=>e.GetProperty(key).EnumerateArray().Select(x=>x.GetDouble()).ToArray();
    sealed class Model
    {
        internal string Name=""; internal double Validation;
        internal Func<Func<double,double>> Reset=null!;
    }
    internal static void Run(bool includeEncoder = false, bool includeIndirect = false, bool includeClosedLoop = false, bool includeClosedRen = false)
    {
        string folder="tests/output/duster-diagnostic";
        var events=File.ReadLines(folder+"/source.jsonl").Select(l=>JsonDocument.Parse(l).RootElement.Clone()).ToArray();
        var r=events.Last(e=>e.GetProperty("kind").GetString()=="record").GetProperty("value");
        double[] u=A(r,"U"),y=A(r,"Y"),pid=A(r,"Pid");int n=u.Length;
        double um=r.GetProperty("um").GetDouble(),us=r.GetProperty("us").GetDouble(),ym=r.GetProperty("ym").GetDouble(),ys=r.GetProperty("ys").GetDouble();
        double dt=r.GetProperty("dt").GetDouble(),lo=r.GetProperty("Lower").GetDouble(),hi=r.GetProperty("Upper").GetDouble();
        double initialI=r.GetProperty("InitialIntegral").GetDouble(),initialE=r.GetProperty("InitialError").GetDouble();
        double bias=u.Skip(Math.Max(0,n-(int)Math.Round(1/dt))).Average();
        var models=new List<Model>();
        void AddRen(string name,double[] parameters,double[]? explicitValues,double validation)
        {
            var net=new NativeRen(n);Array.Copy(parameters,net.Parameters,parameters.Length);
            if(explicitValues==null)net.Refresh();else Array.Copy(explicitValues,net.Explicit,explicitValues.Length);
            var warm=parameters.Skip(690).ToArray();var next=new double[8];var w=new double[8];
            for(int k=0;k<n;k++){net.Step(warm,0,k==0?0:(u[k-1]-um)/us,next,0,w,0);var swap=warm;warm=next;next=swap;}
            models.Add(new Model {Name=name,Validation=validation,Reset=()=>{
                var x=(double[])warm.Clone();var xn=new double[8];var features=new double[8];
                return action=>{double value=net.Step(x,0,(action-um)/us,xn,0,features,0)*ys+ym;var swap=x;x=xn;xn=swap;return value;};}});
        }
        var game=events.Last(e=>e.GetProperty("kind").GetString()=="response").GetProperty("value");
        AddRen("REN_game_2000",A(game,"parameters"),null,game.GetProperty("validation").GetDouble());
        var structure=JsonDocument.Parse(File.ReadAllText(folder+"/structure.json")).RootElement;
        foreach(string variant in new[]{"baseline","combined"})
        {
            var best=structure.GetProperty("records").EnumerateArray().Where(e=>e.GetProperty("variant").GetString()==variant).OrderBy(e=>e.GetProperty("validation").GetDouble()).First();
            AddRen("REN_"+variant+"_5000",A(best,"parameters"),A(best,"explicitParameters"),best.GetProperty("validation").GetDouble());
        }
        if(includeEncoder)
        {
            var encoder=JsonDocument.Parse(File.ReadAllText(folder+"/encoder.json")).RootElement;
            foreach(var e in encoder.GetProperty("results").EnumerateArray())
            {
                var net=new NativeRen(n);var values=A(e,"explicitParameters");Array.Copy(values,net.Explicit,values.Length);
                var warm=A(e,"warmState");string name="Encoder_sigma"+e.GetProperty("sigma").GetDouble().ToString(System.Globalization.CultureInfo.InvariantCulture)+"_seed"+e.GetProperty("seed").GetInt32();
                models.Add(new Model {Name=name,Validation=e.GetProperty("validation").GetDouble(),Reset=()=>{
                    var x=(double[])warm.Clone();var xn=new double[8];var features=new double[8];
                    return action=>{double value=net.Step(x,0,(action-um)/us,xn,0,features,0)*ys+ym;var swap=x;x=xn;xn=swap;return value;};}});
            }
        }
        if(includeClosedRen)
        {
            var data=JsonDocument.Parse(File.ReadAllText(folder+"/closedren.json")).RootElement;
            var reference=JsonDocument.Parse(File.ReadAllText(folder+"/reference.json")).RootElement;var wave=A(reference,"wave");var times=A(r,"T");double maxDifference=0;var nativeResults=new List<object>();
            foreach(var item in data.GetProperty("results").EnumerateArray())
            {
                var net=new NativeRen(n);Array.Copy(A(item,"explicitParameters"),net.Explicit,305);
                var parameters=A(item,"parameters");var x=parameters.Skip(690).ToArray();var xn=new double[8];var w=new double[8];var cy=A(item,"closedY");var cu=A(item,"closedU");
                double last=u[159],prev=wave[159]-y[159],acc=0,sy=0,su=0,modelDifference=0;var nativeY=new List<double>();var nativeU=new List<double>();
                for(int k=160;k<n;k++){
                    double yp=net.Step(x,0,(last-um)/us,xn,0,w,0)*ys+ym;var swap=x;x=xn;xn=swap;
                    double error=wave[k]-yp;last=NativeAutoTune.PidStep(pid,error,prev,ref acc,times[k]-times[k-1],lo,hi,false);prev=error;
                    maxDifference=Math.Max(maxDifference,Math.Max(Math.Abs(yp-cy[k-160]),Math.Abs(last-cu[k-160])));
                    modelDifference=Math.Max(modelDifference,Math.Max(Math.Abs(yp-cy[k-160]),Math.Abs(last-cu[k-160])));nativeY.Add(yp);nativeU.Add(last);
                    if(k>=800){sy+=Math.Pow((yp-y[k])/ys,2);su+=Math.Pow((last-u[k])/us,2);}
                }
                nativeResults.Add(new{mode=item.GetProperty("mode").GetString(),seed=item.GetProperty("seed").GetInt32(),closedYError=Math.Sqrt(sy/400),closedUError=Math.Sqrt(su/400),maxDifference=modelDifference,closedY=nativeY,closedU=nativeU});
                var warm=A(item,"warmState");string name="ClosedREN_"+item.GetProperty("mode").GetString()+"_seed"+item.GetProperty("seed").GetInt32();
                models.Add(new Model{Name=name,Validation=item.GetProperty("validation").GetDouble(),Reset=()=>{
                    var xx=(double[])warm.Clone();var nn=new double[8];var ww=new double[8];return action=>{double value=net.Step(xx,0,(action-um)/us,nn,0,ww,0)*ys+ym;var sw=xx;xx=nn;nn=sw;return value;};}});
            }
            Console.WriteLine("REN double vs production float closed-loop max discrepancy="+maxDifference);
            File.WriteAllText(folder+"/closedren-native.json",JsonSerializer.Serialize(nativeResults));
        }
        var info=JsonDocument.Parse(File.ReadAllText(folder+(includeClosedLoop?"/closedloop.json":includeIndirect?"/indirect.json":"/information.json"))).RootElement;
        if(includeClosedLoop)
        {
            var reference=JsonDocument.Parse(File.ReadAllText(folder+"/reference.json")).RootElement;var wave=A(reference,"wave");var time=A(r,"T");
            double maxError=0;
            foreach(var m in info.GetProperty("models").EnumerateArray())
            {
                var py=A(m,"closedY");var pu=A(m,"closedU");var b=A(m,"coefficients");var z=A(m,"initialState");
                double prev=info.GetProperty("previousError").GetDouble(),acc=0,last=u[159];
                for(int k=160;k<n;k++)
                {
                    double un=(last-um)/us,v=z[0]+b[3]*un+b[6];
                    z=new[]{z[1]+b[0]*v+b[4]*un,z[2]+b[1]*v+b[5]*un,b[2]*v};
                    double yp=v*ys+ym,error=wave[k]+info.GetProperty("baseSP").GetDouble()-yp;
                    double action=NativeAutoTune.PidStep(pid,error,prev,ref acc,time[k]-time[k-1],lo,hi,false);prev=error;last=action;
                    maxError=Math.Max(maxError,Math.Max(Math.Abs(yp-py[k-160]),Math.Abs(action-pu[k-160])));
                }
            }
            if(maxError>1e-6)throw new Exception("Closed-loop Python/native mismatch "+maxError);
            Console.WriteLine("Closed-loop replay vs production PidStep PASS, max error="+maxError);
        }
        foreach(var model in info.GetProperty("models").EnumerateArray().Where(e=>e.GetProperty("family").GetString()=="ARX" && e.GetProperty("order").GetInt32()<=3))
        {
            int order=model.GetProperty("order").GetInt32();var beta=A(model,"coefficients");
            foreach(bool measured in new[]{true,false})
            {
                var history=y.Select(v=>(v-ym)/ys).ToArray();
                if(!measured && includeClosedLoop){var pred=A(model,"predictedY");for(int k=160;k<n;k++)history[k]=(pred[k-160]-ym)/ys;}
                else if(!measured)for(int k=includeIndirect?160:order;k<n;k++){double value=beta[2*order];for(int j=0;j<order;j++)value+=beta[j]*history[k-1-j]+beta[order+j]*(u[k-1-j]-um)/us;history[k]=value;}
                var yh=history.Reverse().Take(order).ToArray();var uh=u.Reverse().Skip(1).Take(order-1).Select(v=>(v-um)/us).ToArray();
                models.Add(new Model {Name=(includeIndirect?model.GetProperty("name").GetString():"ARX"+order)+(measured?"_observed_state":"_free_state"),Validation=model.GetProperty("full_free_validation").GetDouble(),Reset=()=>{
                    var yy=(double[])yh.Clone();var uu=(double[])uh.Clone();return action=>{
                        double normalized=(action-um)/us;double value=beta[2*order]+beta[order]*normalized;
                        for(int j=0;j<order;j++)value+=beta[j]*yy[j];for(int j=1;j<order;j++)value+=beta[order+j]*uu[j-1];
                        for(int j=order-1;j>0;j--)yy[j]=yy[j-1];yy[0]=value;
                        for(int j=uu.Length-1;j>0;j--)uu[j]=uu[j-1];if(uu.Length>0)uu[0]=normalized;
                        return value*ys+ym;};}});
            }
        }
        var grid=NativeAutoTune.Candidates(pid);var results=new List<object>();
        double minU=u.Min(),maxU=u.Max(),minY=y.Min(),maxY=y.Max();
        // Primary reproduces the installed objective. Smaller steps probe extrapolation sensitivity.
        foreach(double delta in new[]{Math.Max(.5*r.GetProperty("Amplitude").GetDouble(),.1*ys),.5})
        foreach(var model in models)
        {
            var candidates=new List<object>();double bestCost=double.PositiveInfinity,baseCost=0;int chosen=0;
            for(int i=0;i<grid.Count;i++)
            {
                double cost=0;var signs=new List<object>();
                foreach(int sign in new[]{-1,1})
                {
                    var step=model.Reset();double lastU=u[n-1],integral=initialI,prev=initialE;
                    int count=(int)Math.Round(15/dt),sat=0,outU=0,outY=0;double total=0;
                    var yy=new double[count];var uu=new double[count];bool diverged=false;
                    for(int k=0;k<count;k++)
                    {
                        double yp=step(lastU),error=y[n-1]+sign*delta-yp;
                        double action=NativeAutoTune.PidStep(grid[i],error,prev,ref integral,dt,lo,hi,false);prev=error;
                        double over=Math.Max(-sign*error/delta,0),effort=(action-bias)/(hi-lo),slew=(action-lastU)/((hi-lo)*dt);
                        double c=2*Math.Pow(error/delta,2)+2.5*over*over+.085*effort*effort+.0105*slew*slew;
                        yy[k]=yp;uu[k]=action;
                        if(!NativeRen.Finite(c)||Math.Abs(yp-y[n-1])>20*delta){total=1e12*count;diverged=true;break;}
                        total+=c;lastU=action;if(action<=lo+1e-5||action>=hi-1e-5)sat++;
                        if(action<minU||action>maxU)outU++;if(yp<minY||yp>maxY)outY++;
                    }
                    cost=Math.Max(cost,total/count);
                    signs.Add(new {sign,cost=total/count,saturation=(double)sat/count,outsideInput=(double)outU/count,outsideOutput=(double)outY/count,diverged,predictedY=yy,predictedU=uu});
                }
                if(i==0)baseCost=cost;if(cost<bestCost){bestCost=cost;chosen=i;}
                candidates.Add(new {index=i,pid=grid[i],cost,signs});
            }
            results.Add(new {model=model.Name,validation=model.Validation,delta,bestIndex=chosen,bestCost,baselineCost=baseCost,ratio=bestCost/baseCost,candidates});
            Console.WriteLine($"{model.Name} delta={delta} best={chosen} PID={string.Join(",",grid[chosen])} cost={bestCost:F4} ratio={bestCost/baseCost:F4}");
        }
        File.WriteAllText(folder+(includeClosedRen?"/closedren-landscape.json":includeClosedLoop?"/closedloop-landscape.json":includeIndirect?"/indirect-landscape.json":includeEncoder?"/encoder-landscape.json":"/landscape.json"),JsonSerializer.Serialize(new {dt,initialI,initialE,bias,grid,results}));
    }
}
