using System;
using BrilliantSkies.Ui.Consoles;
using BrilliantSkies.Ui.Consoles.Segments;
using UnityEngine;
namespace PIDSupporter
{
    // Bounded IMGUI drawing; spectra are cached by the background acquisition worker.
    internal sealed class AutoTuneVisual : ScreenSegmentAbstract
    {
        internal enum View { Progress, Spectrum, Coherence, Records, Bands }
        private readonly AutoTuneTab owner;private readonly View view;
        private static readonly Color Sp=new Color(.35f,.85f,.6f),Input=new Color(.3f,.75f,1),Output=new Color(1,.65f,.3f),Grid=new Color(.2f,.25f,.3f);
        internal AutoTuneVisual(ConsoleUiScreen screen,AutoTuneTab owner,View view):base(screen){this.owner=owner;this.view=view;SpaceAbove=8;SpaceBelow=8;}
        private static void Box(Rect r,Color c){var previous=GUI.color;GUI.color=c;GUI.DrawTexture(r,Texture2D.whiteTexture);GUI.color=previous;}
        private static GUIStyle? textStyle,buttonStyle;
        private static void Styles(){if(textStyle!=null)return;textStyle=new GUIStyle{font=GUI.skin.label.font,fontSize=18,wordWrap=false,clipping=TextClipping.Clip,alignment=TextAnchor.MiddleLeft};textStyle.normal.textColor=Color.white;buttonStyle=new GUIStyle(GUI.skin.button){fontSize=18,wordWrap=false,clipping=TextClipping.Clip};}
        private static void Label(Rect r,string text,Color c){var previous=GUI.color;GUI.color=c;GUI.Label(r,text,textStyle!);GUI.color=previous;}
        private static void Line(Vector2 a,Vector2 b,Color c)
        {
            var matrix=GUI.matrix;var color=GUI.color;
            try{GUI.color=c;GUIUtility.RotateAroundPivot(Mathf.Atan2(b.y-a.y,b.x-a.x)*Mathf.Rad2Deg,a);GUI.DrawTexture(new Rect(a.x,a.y,Vector2.Distance(a,b),2),Texture2D.whiteTexture);}
            finally{GUI.matrix=matrix;GUI.color=color;}
        }
        protected override void UiEntry(ScreenSegmentDisplayOptions options)
        {
            owner.KeepWindowAlive();Styles();
            bool table=view==View.Records||view==View.Bands;
            int rows=view==View.Records?Math.Max(1,owner.RecordCount):4;
            float height=table?58+rows*44:view==View.Progress?132:340;
            Rect r=GUILayoutUtility.GetRect(100,height,GUILayout.Height(height),GUILayout.ExpandWidth(true));
            // Explicit shared row rectangles keep buttons and labels aligned. Native
            // column tables measure them independently, causing row drift with skins.
            if(table){DrawTable(r,rows);return;}
            if(Event.current.type!=EventType.Repaint)return;
            Box(r,new Color(.075f,.095f,.12f));Box(new Rect(r.x,r.y,r.width,1),Grid);
            if(view==View.Progress)
            {
                Label(new Rect(r.x+10,r.y+5,r.width-20,30),owner.ProgressCaption,Color.white);
                var bar=new Rect(r.x+10,r.y+40,r.width-20,9);Box(bar,Grid);bar.width*=(float)owner.CollectionProgress;Box(bar,Input);
                Label(new Rect(r.x+10,r.y+60,r.width-20,30),$"Window data  {owner.RecordCount} / 5 runs",Color.white);
                for(int i=0;i<5;i++)Box(new Rect(r.x+10+i*26,r.y+102,20,15),i<owner.RecordCount?Sp:Grid);return;
            }
            bool coherence=view==View.Coherence;
            Label(new Rect(r.x+10,r.y+3,r.width-20,30),coherence?"Connection to injected SP":"Spectral coverage",Color.white);
            Label(new Rect(r.x+10,r.y+34,r.width-20,30),coherence?"Coherence 0-1":"dB below each signal's own peak",Color.gray);
            var plot=new Rect(r.x+62,r.y+78,Math.Max(30,r.width-96),185);
            for(int i=0;i<=4;i++){float yy=plot.y+i*plot.height/4;Box(new Rect(plot.x,yy,plot.width,1),Grid);Label(new Rect(r.x+2,yy-14,56,28),coherence?(1-i*.25).ToString("0.00"):(-i*15).ToString(),Color.gray);}
            double[] ticks=coherence?new[]{.1,.3,1,2}:new[]{1.0/30,.1,.3,1,2};double min=coherence?.1:1.0/30;
            Func<double,float> px=f=>plot.x+(float)(Math.Log(f/min)/Math.Log(2/min))*plot.width;
            foreach(double f in ticks){float xx=px(f);Box(new Rect(xx,plot.y,1,plot.height),Grid);Label(new Rect(xx-22,plot.yMax+2,60,28),f.ToString("0.##"),Color.gray);}
            var s=owner.Spectrum;if(s==null){Label(new Rect(plot.x+8,plot.y+55,plot.width-10,50),"Complete a 30 s test to view bandwidth.\nNo quality estimate yet.",Color.white);return;}
            var series=coherence?s.Coherence:s.Power;var freq=coherence?s.CoherenceFrequency:s.Frequency;var colors=coherence?new[]{Input,Output}:new[]{Sp,Input,Output};
            for(int a=0;a<series.Length;a++)
            {
                double peak=0;foreach(double value in series[a])if(!double.IsNaN(value))peak=Math.Max(peak,value);
                Vector2? previous=null;
                for(int k=0;k<freq.Length;k++)
                {
                    double value=series[a][k];if(double.IsNaN(value)||(!coherence&&peak<=1e-20)){previous=null;continue;}
                    double fraction=coherence?value:1+Math.Max(-60,10*Math.Log10(Math.Max(value/peak,1e-6)))/60;
                    var point=new Vector2(px(freq[k]),plot.yMax-(float)fraction*plot.height);if(previous.HasValue)Line(previous.Value,point,colors[a]);previous=point;
                }
            }
            float cell=(r.width-24)/(coherence?2:3);
            Label(new Rect(r.x+12,r.yMax-36,cell,30),coherence?"SP -> u":"Injected SP",coherence?Input:Sp);
            Label(new Rect(r.x+12+cell,r.yMax-36,cell,30),coherence?"SP -> y":"Control u",coherence?Output:Input);
            if(!coherence)Label(new Rect(r.x+12+2*cell,r.yMax-36,cell,30),"Output y",Output);
        }
        private void DrawTable(Rect r,int rows)
        {
            bool records=view==View.Records;string[] headings=records?new[]{"Run","P","Ti (s)","Td (s)","Samples"}:new[]{"Band Hz","SP RMS","u RMS","y RMS"};
            int count=headings.Length;float cell=(r.width-20)/count;
            if(Event.current.type==EventType.Repaint){Box(r,new Color(.075f,.095f,.12f));Box(new Rect(r.x,r.y,r.width,1),Grid);for(int c=0;c<count;c++)Label(new Rect(r.x+10+c*cell,r.y+5,cell-4,30),headings[c],Color.gray);}
            for(int row=0;row<rows;row++)
            {
                float y=r.y+50+row*44;
                if(Event.current.type==EventType.Repaint)Box(new Rect(r.x+8,y-3,r.width-16,1),Grid);
                if(records&&row>=owner.RecordCount){if(Event.current.type==EventType.Repaint)Label(new Rect(r.x+10,y,r.width-20,28),"No completed runs",Color.gray);continue;}
                for(int c=0;c<count;c++)
                {
                    var rect=new Rect(r.x+10+c*cell,y,cell-6,36);
                    if(records&&c==0){if(GUI.Button(rect,(row+1).ToString(),buttonStyle!))owner.SelectSpectrum(row);continue;}
                    if(Event.current.type!=EventType.Repaint)continue;
                    var s=owner.Spectrum;string value=records?owner.RecordCell(row,c):c==0?AutoTuneSpectrum.Bands[row]:s==null?"-":s.BandRms[c-1][row].ToString("0.0E+0");
                    Label(rect,value,Color.white);
                }
            }
        }
    }
}
