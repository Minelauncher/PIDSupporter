using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using BrilliantSkies.Ui.Consoles;
using HarmonyLib;
namespace PIDSupporter
{
    internal static class AutoTuneWindowLifetime
    {
        private static readonly ConditionalWeakTable<ConsoleWindow,AutoTuneTab> tabs=new ConditionalWeakTable<ConsoleWindow,AutoTuneTab>();
        private static readonly List<WeakReference<AutoTuneTab>> live=new List<WeakReference<AutoTuneTab>>();
        internal static void Register(ConsoleWindow parent,ConsoleWindow tuner,AutoTuneTab tab)
        {
            tabs.Remove(parent);tabs.Add(parent,tab);tabs.Remove(tuner);tabs.Add(tuner,tab);
            live.RemoveAll(w=>!w.TryGetTarget(out var t)||t.IsClosed);live.Add(new WeakReference<AutoTuneTab>(tab));
        }
        internal static void Close(ConsoleWindow window) { if(tabs.TryGetValue(window,out var tab))tab.CloseHistory(); }
        internal static void CloseAll() { foreach(var w in live)if(w.TryGetTarget(out var tab))tab.CloseHistory();live.Clear(); }
    }
    [HarmonyPatch(typeof(ConsoleWindow),"CloseWindow")]
    internal static class AutoTuneWindowClosePatch
    { private static void Prefix(ConsoleWindow __instance)=>AutoTuneWindowLifetime.Close(__instance); }
    [HarmonyPatch(typeof(ConsoleWindow),"OnDeactivateGui")]
    internal static class AutoTuneWindowDeactivatePatch
    { private static void Prefix(ConsoleWindow __instance)=>AutoTuneWindowLifetime.Close(__instance); }
    [HarmonyPatch(typeof(ConsoleWindow),"CloseAllWindows")]
    internal static class AutoTuneWindowsClosePatch
    { private static void Prefix()=>AutoTuneWindowLifetime.CloseAll(); }
}
