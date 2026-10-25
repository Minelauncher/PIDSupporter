using System;
using System.Collections.Generic;
using BrilliantSkies.Ui.Consoles;
using BrilliantSkies.Ui.Consoles.Segments;
using UnityEngine;
namespace PIDSupporter
{
    // Text has no skin background or hover highlight. Layout computes wrapped height.
    internal sealed class AutoTuneText : ScreenSegmentAbstract
    {
        private readonly AutoTuneTab owner;
        private readonly string title;
        private readonly List<Func<string>> content=new List<Func<string>>();
        private readonly List<string> tips=new List<string>();
        private GUIStyle? body,heading;
        internal AutoTuneText(ConsoleUiScreen screen,AutoTuneTab owner,string title):base(screen)
        {this.owner=owner;this.title=title;SpaceAbove=12;SpaceBelow=8;}
        internal void Add(Func<string> text,string tip){content.Add(text);tips.Add(tip);}
        protected override void UiEntry(ScreenSegmentDisplayOptions options)
        {
            owner.KeepWindowAlive();
            if(body==null){
                body=new GUIStyle{font=GUI.skin.label.font,fontSize=18,wordWrap=true,richText=false,padding=new RectOffset(8,8,4,4)};
                body.normal.textColor=Color.white;
                heading=new GUIStyle(body){fontStyle=FontStyle.Bold,fontSize=20};
            }
            Rect line=GUILayoutUtility.GetRect(1,1,GUILayout.Height(1),GUILayout.ExpandWidth(true));
            if(Event.current.type==EventType.Repaint){var color=GUI.color;GUI.color=new Color(.3f,.36f,.42f);GUI.DrawTexture(line,Texture2D.whiteTexture);GUI.color=color;}
            GUILayout.Label(title,heading!,GUILayout.ExpandWidth(true));
            for(int i=0;i<content.Count;i++)GUILayout.Label(new GUIContent(content[i](),tips[i]),body,GUILayout.ExpandWidth(true));
        }
    }
}
