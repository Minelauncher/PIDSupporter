// ============================================================================
// VariableControllerUiFixedUpdatePatch.cs — 매 물리 틱마다 FRIT 탭에 알려주는 패치
//
// ■ 메서드/타입 출처
//   [Harmony] TargetMethod()              패치 대상 (FixedUpdateWhenActive) 지정
//   [Harmony] Postfix(__instance, t)       원래 메서드 실행 후 호출
//   [자체]    tab.OnUiFixed()             FRIT 탭의 매 틱 처리 호출
//   [Harmony] AccessTools.TypeByName()    [Harmony] 타입 이름으로 찾기
//   [Harmony] AccessTools.Method()        [Harmony] 메서드 찾기
//   [FTD]     ITimeStep                   [FTD] 시간 정보 인터페이스
//   [FTD]     AdvLogger.LogInfo()         [FTD] 게임 로그
//
// ■ FixedUpdate란?
//   Unity(FTD의 게임 엔진)에서 물리 시뮬레이션은 고정 간격(보통 0.02초=50Hz)으로 돌아감.
//   FixedUpdate = 이 고정 간격마다 호출되는 함수.
//   FTD의 PID UI는 FixedUpdateWhenActive라는 메서드가 있어서, UI가 열려있을 때마다 호출됨.
//
// ■ 이 파일이 하는 일:
//   FTD의 FixedUpdateWhenActive 실행 직후(Postfix)에 우리 FRIT 탭의 OnUiFixed()를 호출.
//   → 가진 신호 적용, u/y 데이터 수집, 적응형 진폭 등이 여기서 돌아감.
// ============================================================================

using HarmonyLib;
using System;
using System.Reflection;
using BrilliantSkies.Core.Logger;
using BrilliantSkies.Core.Timing;   // ITimeStep (FTD의 시간 정보 인터페이스)

namespace PIDSupporter
{
    [HarmonyPatch]
    public static class VariableControllerUiFixedUpdatePatch
    {
        /// <summary>패치 대상: GP-PID UI의 FixedUpdateWhenActive 메서드</summary>
        static MethodBase? TargetMethod()
        {
            Type? t = AccessTools.TypeByName("BrilliantSkies.Blocks.Ai.VariableControllerUi");
            if (t == null) return null;
            return AccessTools.Method(t, "FixedUpdateWhenActive", new[] { typeof(ITimeStep) });
        }

        /// <summary>
        /// 원래 FixedUpdateWhenActive 실행 후 호출.
        /// 이 UI 인스턴스에 연결된 FRIT 탭이 있으면 OnUiFixed() 호출.
        /// </summary>
        static void Postfix(object __instance, ITimeStep t)
        {
            try
            {
                if (__instance == null) return;

                // 이 UI에 FRIT 탭이 등록되어 있는지 확인 → 있으면 틱 전달
                if (VariableControllerUiPatch.TryGetTab(__instance, out FritTuningTab tab) && tab != null)
                {
                    tab.OnUiFixed();
                }
            }
            catch (Exception e)
            {
                AdvLogger.LogInfo("[PIDSupporter] FixedUpdatePatch 실패: " + e, LogOptions.None);
            }
        }
    }
}
