// ============================================================================
// VariableControllerUiPatch.cs — GP-PID 편집 UI에 FRIT 창을 끼워넣는 Harmony 패치
//
// ■ 메서드/타입 출처
//   [자체]    TryGetTab(instance, out tab)         UI인스턴스→FRIT탭 조회
//   [Harmony] TargetMethod()                       패치 대상 메서드 지정
//   [Harmony] Postfix(__instance, __result)         원래 메서드 실행 후 호출
//   [자체]    CreateFritWindowViaNewWindow(...)     리플렉션으로 FTD 새 창 생성
//   [자체]    GetMethodInHierarchy(type,name,sig)   상속 계층에서 메서드 찾기
//   [자체]    GetFieldInHierarchy(type,obj,name)    상속 계층에서 필드 찾기
//
//   [Harmony] AccessTools.TypeByName(name)          [Harmony] 타입 이름으로 찾기
//   [Harmony] AccessTools.Method(type,name,sig)     [Harmony] 메서드 찾기
//   [C#]      MethodInfo / FieldInfo / PropertyInfo [C#] 리플렉션 타입
//   [C#]      Type.GetMethod / GetField / GetProperty [C#] 리플렉션 조회
//   [C#]      MethodInfo.Invoke(obj, args)          [C#] 메서드 동적 호출
//   [C#]      FieldInfo.GetValue(obj)               [C#] 필드 값 읽기
//   [C#]      PropertyInfo.GetValue(obj, index)     [C#] 프로퍼티 값 읽기
//   [C#]      BindingFlags.NonPublic|Instance|Public [C#] 접근 범위 지정
//   [C#]      Dictionary<K,V>                       [C#] 해시맵
//   [FTD]     ConsoleWindow                         [FTD] UI 창
//   [FTD]     ScaledRectangle(x,y,w,h)              [FTD] UI 위치/크기
//   [FTD]     ScaledSizing(value, Dimension)         [FTD] UI 최소 크기
//   [FTD]     window.SetMultipleTabs(screens[])      [FTD] 창에 탭 설정
//   [FTD]     AdvLogger.LogInfo(msg, opts)           [FTD] 게임 로그 출력
//
// ■ Harmony란?
//   게임 코드를 직접 수정하지 않고, 런타임에 메서드 실행 전/후에 우리 코드를 끼워넣는 라이브러리.
//   [HarmonyPatch] 속성을 붙이면 Harmony가 자동으로 패치 대상을 찾아 적용.
//
// ■ 이 파일이 하는 일:
//   FTD에서 GP-PID 블록의 편집 UI(VariableControllerUi.BuildInterface)가 호출될 때,
//   그 직후(Postfix)에 FRIT 튜닝 창을 별도로 생성해서 띄움.
//
// ■ GP-PID vs AI-PID:
//   GP-PID = 블록에 직접 붙는 PID (BrilliantSkies.Blocks.Ai.VariableControllerUi)
//   AI-PID = AI Mainframe에서 여는 PID (글로벌 VariableControllerUi) → AiVariableControllerUiPatch.cs
// ============================================================================

using HarmonyLib;
using System;
using System.Collections.Generic;
using System.Reflection;            // MethodBase, FieldInfo, PropertyInfo 등 (리플렉션)
using System.Runtime.CompilerServices; // ConditionalWeakTable (GC 연동 약참조 테이블)
using BrilliantSkies.Core.Logger;   // AdvLogger (FTD 로그 시스템)
using BrilliantSkies.Ui.Consoles;   // ConsoleWindow 등 (FTD UI)
using BrilliantSkies.Ai.Control.Pids; // VariableControllerMaster (PID 제어기)

namespace PIDSupporter
{
    // [HarmonyPatch] = "이 클래스는 Harmony 패치입니다"라는 표시.
    // static class = 인스턴스를 만들지 않는 클래스 (모든 멤버가 static).
    [HarmonyPatch]
    public static class VariableControllerUiPatch
    {
        // UI 인스턴스 → FRIT 탭 매핑. PID 편집 창 하나당 FRIT 탭 하나.
        // ConditionalWeakTable = key가 GC되면 엔트리도 자동 제거 → 메모리 누수 방지.
        // Dictionary는 key를 강참조해서 GC를 막지만, 이건 약참조라 UI 닫히면 정리됨.
        private static readonly ConditionalWeakTable<object, FritTuningTab> _tabsByUiInstance
            = new ConditionalWeakTable<object, FritTuningTab>();

        // FixedUpdate 패치에서 "이 UI에 FRIT 탭이 있나?" 확인할 때 사용.
        public static bool TryGetTab(object uiInstance, out FritTuningTab tab)
            => _tabsByUiInstance.TryGetValue(uiInstance, out tab);

        /// <summary>
        /// Harmony에게 "어떤 메서드를 패치할 건지" 알려주는 함수.
        /// FTD의 BrilliantSkies.Blocks.Ai.VariableControllerUi.BuildInterface(string)를 대상으로 함.
        /// AccessTools = Harmony 유틸. private/internal 타입이나 메서드도 찾을 수 있음.
        /// </summary>
        static MethodBase TargetMethod()
        {
            Type t = AccessTools.TypeByName("BrilliantSkies.Blocks.Ai.VariableControllerUi");
            if (t == null) return null;
            return AccessTools.Method(t, "BuildInterface", new[] { typeof(string) });
        }

        /// <summary>
        /// Postfix = 원래 메서드(BuildInterface) 실행이 끝난 직후에 호출됨.
        /// __instance = 원래 메서드의 this (VariableControllerUi 인스턴스).
        /// __result = 원래 메서드의 반환값 (ConsoleWindow). ref = 수정 가능하다는 표시.
        ///
        /// 여기서 하는 일:
        /// 1) __instance에서 리플렉션으로 PID 제어기(VariableControllerMaster) 추출
        /// 2) 새 ConsoleWindow(FRIT 전용 창) 생성
        /// 3) FritTuningTab을 만들어서 창에 넣기
        /// </summary>
        static void Postfix(object __instance, ref ConsoleWindow __result)
        {
            try
            {
                if (__instance == null || __result == null) return;

                // 같은 UI 인스턴스에 대해 중복 생성 방지
                if (_tabsByUiInstance.TryGetValue(__instance, out _)) return;

                // ── 리플렉션으로 PID 제어기 꺼내기 ──
                // GP-PID의 구조: VariableControllerUi._focus = IPidBlock
                //                IPidBlock.Controller = VariableControllerMaster
                // _focus는 private 필드라서 일반적으로 접근 불가 → 리플렉션으로 강제 접근.
                object focusObj = GetFieldInHierarchy(__instance.GetType(), __instance, "_focus");
                if (focusObj == null) return;

                // IPidBlock에서 Controller 프로퍼티(속성)를 찾아 VariableControllerMaster 얻기
                PropertyInfo controllerProp = focusObj.GetType().GetProperty("Controller", BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
                VariableControllerMaster controller = controllerProp != null ? controllerProp.GetValue(focusObj, null) as VariableControllerMaster : null;
                if (controller == null) return;

                // FRIT 전용 별도 창 생성 (기존 PID 창에 탭으로 넣지 않고 새 창)
                // 700, 10, 520, 720 = 화면상 x, y, 폭, 높이
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

                AdvLogger.LogInfo("[PIDSupporter] FRIT를 별도 창으로 열었습니다.", LogOptions.None);
            }
            catch (Exception e)
            {
                AdvLogger.LogInfo("[PIDSupporter] VariableControllerUiPatch 실패: " + e, LogOptions.None);
            }
        }

        /// <summary>
        /// FTD UI 시스템의 NewWindow 메서드를 리플렉션으로 호출해서 새 창을 만듦.
        /// NewWindow는 protected(상속받은 클래스만 접근 가능)라서 직접 호출 불가 → 리플렉션 사용.
        /// </summary>
        private static ConsoleWindow CreateFritWindowViaNewWindow(object variableControllerUiInstance, string title, float x, float y, float w, float h)
        {
            try
            {
                ScaledRectangle rect = new ScaledRectangle(x, y, w, h);

                MethodInfo mi = GetMethodInHierarchy(
                    variableControllerUiInstance.GetType(),
                    "NewWindow",
                    new[] { typeof(int), typeof(string), typeof(ScaledRectangle) }
                );

                if (mi == null)
                {
                    AdvLogger.LogInfo("[PIDSupporter] NewWindow 메서드를 찾지 못했습니다.", LogOptions.None);
                    return null;
                }

                // index는 1로(원본 PID 창이 0을 쓰는 경우가 많아서 충돌 줄이기)
                ConsoleWindow win = mi.Invoke(variableControllerUiInstance, new object[] { 1, title, rect }) as ConsoleWindow;
                return win;
            }
            catch (Exception e)
            {
                AdvLogger.LogInfo("[PIDSupporter] CreateFritWindow 실패: " + e, LogOptions.None);
                return null;
            }
        }

        /// <summary>
        /// 클래스 상속 계층을 따라 올라가며 메서드를 찾는 유틸.
        /// 왜 필요한가? FTD UI 클래스는 여러 단계 상속(A→B→C→...)되어 있어서
        /// 찾고자 하는 메서드가 부모의 부모에 있을 수 있음.
        /// GetMethod는 현재 타입에서만 찾으므로, BaseType을 타고 올라가며 탐색.
        /// </summary>
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

        /// <summary>
        /// 클래스 상속 계층을 따라 올라가며 필드(변수)를 찾아 값을 읽는 유틸.
        /// private 필드도 BindingFlags.NonPublic으로 접근 가능 (리플렉션의 힘).
        /// </summary>
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
}
