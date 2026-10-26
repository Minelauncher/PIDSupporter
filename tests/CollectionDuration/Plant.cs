using System;using System.Text.Json;partial class Run {
sealed class Plant {
 internal double[] X,Q;internal double Cross,Time;readonly JsonElement spec;readonly double gain;readonly bool coupled;
 internal Plant(JsonElement c){spec=c.GetProperty("plant");X=new double[spec.GetProperty("n").GetInt32()];X[0]=c.GetProperty("initial").GetDouble();if(X.Length>1)X[1]=-.5*X[0];Q=new double[c.GetProperty("delay").GetInt32()];gain=c.GetProperty("gain").GetDouble();coupled=c.GetProperty("coupled").GetBoolean();}
 internal double Step(double u){Array.Copy(Q,1,Q,0,Q.Length-1);Q[Q.Length-1]=u;double act=gain*Q[0],y;
 if(spec.GetProperty("type").GetString()=="nonlinear"){double p1=spec.GetProperty("p1").GetDouble(),p2=spec.GetProperty("p2").GetDouble();X[0]=p1*X[0]+(1-p1)*(act-.2*X[0]*Math.Abs(X[0]));X[1]=p2*X[1]+(1-p2)*X[0];y=X[1];}
 else {var A=spec.GetProperty("A");var B=spec.GetProperty("B");var C=spec.GetProperty("C");var xn=new double[X.Length];for(int i=0;i<X.Length;i++){xn[i]=B[i].GetDouble()*act;for(int j=0;j<X.Length;j++)xn[i]+=A[i][j].GetDouble()*X[j];}X=xn;y=0;for(int i=0;i<X.Length;i++)y+=C[i].GetDouble()*X[i];}
 Time+=.05;if(coupled)Cross+=.05*(-.7*Cross+.4*y+.8*Math.Sin(2*Math.PI*.11*Time)+.4*Math.Sin(2*Math.PI*.23*Time));return y-(coupled?.8*Cross:0);}
}

}