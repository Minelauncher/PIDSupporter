// ============================================================================
// VariableControllerOutputPatch.cs — u-direct 가진 신호 주입 (additive perturbation)
//
// ■ 왜 필요한가
//   SP-가진 방식 (SetPointAdjust 변경) 은 PID 가 약하면 (Kp ≈ 0) u 도 거의 안 움직여서
//   plant 가 자극받지 않음. 강하면 closed-loop 이 가진을 너무 빨리 reject 해서 plant
//   응답이 안 보임. 어느 쪽이든 PID 강도에 데이터 품질이 의존하는 closed-loop ID 의
//   고전적 문제.
//
// ■ 해결: u-direct 가진
//   FTD 의 PID 출력 (LastControlVariable) 에 직접 가진 신호 더함.
//   plant 는 PID 강도와 무관하게 일정한 자극 받음 → 데이터 품질이 PID 에 안 의존.
//   학계 용어: "additive perturbation" 또는 "external excitation".
//
// ■ 구현
//   1. VariableControllerMaster.NewMeasurement(sp, pv, dt) → Single 을 Harmony postfix 로 후킹.
//   2. 데이터 수집 모드에서만 가진값을 더해서 __result 와 LastControlVariable 둘 다 갱신.
//   3. LastControlVariable 는 ControlBase 의 private backing field 라 reflection 으로 set.
//
// ■ 사용 흐름 (FritTuningTab 쪽)
//   - 수집 시작: 매 틱 FritExcitationInjector.Set(focus, x) 로 현재 가진값 설정
//   - 수집 종료: FritExcitationInjector.Clear(focus) 로 인젝션 해제
//   - 인젝터가 등록 안 된 컨트롤러는 patch 가 no-op → 다른 PID 영향 없음
// ============================================================================

using System;
using System.Collections.Generic;
using System.Reflection;
using BrilliantSkies.Ai.Control.Pids;
using BrilliantSkies.Core.Logger;
using HarmonyLib;

namespace PIDSupporter
{
    /// <summary>
    /// 데이터 수집 중인 컨트롤러별 현재 가진값을 보관.
    /// FritTuningTab 이 매 틱 Set, 종료 시 Clear.
    /// VariableControllerOutputPatch 가 NewMeasurement postfix 에서 읽어 u 에 더함.
    /// </summary>
    internal static class FritExcitationInjector
    {
        // ConcurrentDictionary 안 쓰는 이유: 게임 루프가 단일 스레드 (FixedUpdate).
        private static readonly Dictionary<VariableControllerMaster, float> _active
            = new Dictionary<VariableControllerMaster, float>();

        public static void Set(VariableControllerMaster controller, float excitation)
        {
            if (controller == null) return;
            _active[controller] = excitation;
        }

        public static void Clear(VariableControllerMaster controller)
        {
            if (controller == null) return;
            _active.Remove(controller);
        }

        public static bool TryGet(VariableControllerMaster controller, out float excitation)
        {
            return _active.TryGetValue(controller, out excitation);
        }
    }

    /// <summary>
    /// VariableControllerMaster.NewMeasurement postfix 로 PID 출력에 가진 신호 추가.
    /// 액추에이터가 NewMeasurement 의 반환값을 사용하므로 __result 수정으로 u 가 바뀜.
    /// LastControlVariable (데이터 수집에서 우리가 읽는 값) 도 동일하게 동기화.
    /// </summary>
    [HarmonyPatch(typeof(VariableControllerMaster), "NewMeasurement")]
    internal static class NewMeasurementOutputPatch
    {
        // ControlBase.<LastControlVariable>k__BackingField — private, 한 번만 lookup 후 캐시.
        private static FieldInfo? _lcvBackingField;
        private static bool _backingLookupDone;

        private static FieldInfo? GetLcvBackingField()
        {
            if (_backingLookupDone) return _lcvBackingField;
            _backingLookupDone = true;
            try
            {
                Type controlBase = typeof(ControlBase);
                _lcvBackingField = AccessTools.Field(controlBase, "<LastControlVariable>k__BackingField");
                if (_lcvBackingField == null)
                {
                    AdvLogger.LogInfo("[PIDSupporter] LastControlVariable backing field not found", LogOptions.None);
                }
            }
            catch (Exception ex)
            {
                AdvLogger.LogInfo("[PIDSupporter] backing field lookup failed: " + ex.Message, LogOptions.None);
            }
            return _lcvBackingField;
        }

        static void Postfix(VariableControllerMaster __instance, ref float __result)
        {
            try
            {
                if (__instance == null) return;
                if (!FritExcitationInjector.TryGet(__instance, out float excite)) return;
                if (excite == 0f) return;

                // FTD 의 PID 출력은 [-1, 1] 범위. 가진 추가 후 클램프.
                float modified = __result + excite;
                if (modified > 1f) modified = 1f;
                else if (modified < -1f) modified = -1f;

                __result = modified;

                // LastControlVariable 동기화: 데이터 수집 시 c.LastControlVariable 을 읽으니
                // 가진이 포함된 값이어야 (u, y) pair 가 일관됨.
                var ctrl = __instance.GetCurrentController();
                if (ctrl == null) return;

                FieldInfo? fld = GetLcvBackingField();
                fld?.SetValue(ctrl, modified);
            }
            catch (Exception ex)
            {
                AdvLogger.LogInfo("[PIDSupporter] NewMeasurementOutputPatch failed: " + ex.Message, LogOptions.None);
            }
        }
    }
}
