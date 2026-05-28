// ============================================================================
// AiVariableControllerUiPatch.cs — AI-PID 편집 UI용 Harmony 패치
//
// ■ VariableControllerUiPatch.cs와 거의 동일하지만 대상이 다름:
//   GP-PID = BrilliantSkies.Blocks.Ai.VariableControllerUi (블록에 붙는 PID)
//   AI-PID = 글로벌 VariableControllerUi (AI Mainframe에서 여는 PID)
//
// ■ 메서드/타입 출처 (VariableControllerUiPatch.cs와 동일 패턴)
//   [자체]    TryGetTab / Postfix / CreateFritWindowViaNewWindow
//   [자체]    GetMethodInHierarchy / GetFieldInHierarchy
//   [Harmony] TargetMethod / AccessTools.*
//   [FTD]     ConsoleWindow / VariableControllerMaster / AdvLogger / ITimeStep
//
// ■ GP-PID와 다른 점:
//   - 타입명: "VariableControllerUi" (글로벌, BrilliantSkies 접두사 없음)
//   - _focus가 이미 VariableControllerMaster (GP-PID는 IPidBlock→.Controller 거침)
// ============================================================================

using HarmonyLib;
using System;
using System.Collections.Generic;
using System.Reflection;
using System.Runtime.CompilerServices;
using BrilliantSkies.Core.Logger;
using BrilliantSkies.Core.Timing;
using BrilliantSkies.Ui.Consoles;
using BrilliantSkies.Ai.Control.Pids;

namespace PIDSupporter
{
    /// <summary>
    /// AI Mainframe에서 여는 PID 편집창 패치.
    ///
    /// GP-PID와 다른 점:
    /// - 클래스: 전역 네임스페이스의 "VariableControllerUi" (Ai.dll)
    /// - 베이스: ConsoleUi&lt;VariableControllerMaster&gt; (GP-PID는 ConsoleUi&lt;IPidBlock&gt;)
    /// - _focus가 이미 VariableControllerMaster (GP-PID는 IPidBlock → .Controller 거쳐야 함)
    /// </summary>
    [HarmonyPatch]
    public static class AiVariableControllerUiPatch
    {
        private static readonly ConditionalWeakTable<object, FritTuningTab> _tabsByUiInstance
            = new ConditionalWeakTable<object, FritTuningTab>();

        public static bool TryGetTab(object uiInstance, out FritTuningTab tab)
            => _tabsByUiInstance.TryGetValue(uiInstance, out tab);

        static MethodBase TargetMethod()
        {
            // 글로벌 네임스페이스의 VariableControllerUi (Ai.dll)
            Type t = AccessTools.TypeByName("VariableControllerUi");
            if (t == null) return null;
            return AccessTools.Method(t, "BuildInterface", new[] { typeof(string) });
        }

        static void Postfix(object __instance, ref ConsoleWindow __result)
        {
            try
            {
                if (__instance == null || __result == null) return;
                if (_tabsByUiInstance.TryGetValue(__instance, out _)) return;

                // _focus는 이미 VariableControllerMaster
                object focusObj = GetFieldInHierarchy(__instance.GetType(), __instance, "_focus");
                VariableControllerMaster controller = focusObj as VariableControllerMaster;
                if (controller == null) return;

                ConsoleWindow fritWindow = CreateFritWindowViaNewWindow(__instance, "FRIT Tuner Supporter", 700f, 10f, 520f, 720f);
                if (fritWindow == null) return;

                fritWindow.MinimumWindowWidth = new ScaledSizing(420f, Dimension.Width);
                fritWindow.MinimumWindowHeight = new ScaledSizing(600f, Dimension.Height);
                fritWindow.BackgroundType = BackgroundType.Normal;
                fritWindow.DisplayTextPrompt = false;

                FritTuningTab fritTab = new FritTuningTab(fritWindow, controller);
                GuideTab guideTab = new GuideTab(fritWindow, controller);
                fritWindow.SetMultipleTabs(new SuperScreen[] { fritTab, guideTab });

                _tabsByUiInstance.Add(__instance, fritTab);

                AdvLogger.LogInfo("[PIDSupporter] AI-PID용 FRIT 창 생성됨.", LogOptions.None);
            }
            catch (Exception e)
            {
                AdvLogger.LogInfo("[PIDSupporter] AiVariableControllerUiPatch 실패: " + e, LogOptions.None);
            }
        }

        private static ConsoleWindow CreateFritWindowViaNewWindow(object uiInstance, string title, float x, float y, float w, float h)
        {
            try
            {
                ScaledRectangle rect = new ScaledRectangle(x, y, w, h);
                MethodInfo mi = GetMethodInHierarchy(
                    uiInstance.GetType(),
                    "NewWindow",
                    new[] { typeof(int), typeof(string), typeof(ScaledRectangle) }
                );
                if (mi == null) return null;
                return mi.Invoke(uiInstance, new object[] { 1, title, rect }) as ConsoleWindow;
            }
            catch { return null; }
        }

        private static MethodInfo GetMethodInHierarchy(Type t, string name, Type[] sig)
        {
            while (t != null)
            {
                MethodInfo mi = t.GetMethod(name, BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.Public, null, sig, null);
                if (mi != null) return mi;
                t = t.BaseType;
            }
            return null;
        }

        private static object GetFieldInHierarchy(Type t, object obj, string fieldName)
        {
            while (t != null)
            {
                FieldInfo f = t.GetField(fieldName, BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.Public);
                if (f != null) return f.GetValue(obj);
                t = t.BaseType;
            }
            return null;
        }
    }

    /// <summary>
    /// AI-PID용 FixedUpdate 패치 (GP-PID와 별개의 클래스이므로 별도 패치 필요)
    /// </summary>
    [HarmonyPatch]
    public static class AiVariableControllerUiFixedUpdatePatch
    {
        static MethodBase TargetMethod()
        {
            Type t = AccessTools.TypeByName("VariableControllerUi");
            if (t == null) return null;
            return AccessTools.Method(t, "FixedUpdateWhenActive", new[] { typeof(ITimeStep) });
        }

        static void Postfix(object __instance, ITimeStep t)
        {
            try
            {
                if (__instance == null) return;
                if (AiVariableControllerUiPatch.TryGetTab(__instance, out FritTuningTab tab) && tab != null)
                    tab.OnUiFixed();
            }
            catch (Exception e)
            {
                AdvLogger.LogInfo("[PIDSupporter] AiVariableControllerUiFixedUpdatePatch 실패: " + e, LogOptions.None);
            }
        }
    }
}
