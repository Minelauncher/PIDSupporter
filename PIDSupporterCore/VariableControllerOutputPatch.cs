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
using UnityEngine;

namespace PIDSupporter
{
    // ─────────────────────────────────────────────────────────────────────
    // TTL (time-to-live) 자동 만료:
    //   FritTuningTab 의 OnUiFixed 가 매 틱 Set() 호출 → LastTime 갱신
    //   사용자가 창 닫으면 OnUiFixed 안 호출됨 → Set() 중단
    //   TryGet() 이 Time.fixedTime - LastTime > TTL_SECONDS 면 자동 Clear + 거부
    //   → 창 닫힘으로 인한 "PID 영구 stuck" 버그 차단
    //   TTL = 0.2s (8 틱 @ 40Hz) — 정상 한 틱 간격 대비 충분히 크고,
    //                              비정상 중단 후 회복 시간 충분히 짧음
    // ─────────────────────────────────────────────────────────────────────
    internal static class InjectorTtl
    {
        public const float TTL_SECONDS = 0.2f;
    }

    /// <summary>
    /// 데이터 수집 중인 컨트롤러별 현재 가진값을 보관.
    /// FritTuningTab 이 매 틱 Set, 종료 시 Clear.
    /// VariableControllerOutputPatch 가 NewMeasurement postfix 에서 읽어 u 에 더함.
    /// </summary>
    internal static class FritExcitationInjector
    {
        private struct Entry { public float Value; public float LastTime; }
        // ConcurrentDictionary 안 쓰는 이유: 게임 루프가 단일 스레드 (FixedUpdate).
        private static readonly Dictionary<VariableControllerMaster, Entry> _active
            = new Dictionary<VariableControllerMaster, Entry>();

        public static void Set(VariableControllerMaster controller, float excitation)
        {
            if (controller == null) return;
            _active[controller] = new Entry { Value = excitation, LastTime = Time.fixedTime };
        }

        public static void Clear(VariableControllerMaster controller)
        {
            if (controller == null) return;
            _active.Remove(controller);
        }

        public static bool TryGet(VariableControllerMaster controller, out float excitation)
        {
            excitation = 0f;
            if (controller == null) return false;
            if (!_active.TryGetValue(controller, out Entry entry)) return false;
            if (Time.fixedTime - entry.LastTime > InjectorTtl.TTL_SECONDS)
            {
                // TTL 만료 — UI 가 창 닫혀 더 이상 refresh 안 함 → stale, 자동 정리
                _active.Remove(controller);
                return false;
            }
            excitation = entry.Value;
            return true;
        }
    }

    /// <summary>
    /// Relay feedback test 중 PID 출력을 **완전 교체** (additive 가 아님).
    /// Åström-Hägglund 1984 표준: u = ±h based on sign of error.
    /// 활성화된 컨트롤러는 PID 의 계산값 무시, relay 출력만 사용.
    /// </summary>
    internal static class RelayOutputInjector
    {
        private struct Entry { public float Value; public float LastTime; }
        private static readonly Dictionary<VariableControllerMaster, Entry> _active
            = new Dictionary<VariableControllerMaster, Entry>();

        public static void Set(VariableControllerMaster controller, float relayOutput)
        {
            if (controller == null) return;
            _active[controller] = new Entry { Value = relayOutput, LastTime = Time.fixedTime };
        }

        public static void Clear(VariableControllerMaster controller)
        {
            if (controller == null) return;
            _active.Remove(controller);
        }

        public static bool TryGet(VariableControllerMaster controller, out float relayOutput)
        {
            relayOutput = 0f;
            if (controller == null) return false;
            if (!_active.TryGetValue(controller, out Entry entry)) return false;
            if (Time.fixedTime - entry.LastTime > InjectorTtl.TTL_SECONDS)
            {
                // TTL 만료 — UI 종료/창 닫힘으로 relay refresh 중단 → PID 정상 복귀
                _active.Remove(controller);
                return false;
            }
            relayOutput = entry.Value;
            return true;
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

                float modified;
                bool changed = false;

                // 우선순위 1: Relay (replace mode) — PID 출력 완전 교체
                if (RelayOutputInjector.TryGet(__instance, out float relayU))
                {
                    modified = relayU;
                    changed = true;
                }
                // 우선순위 2: 가진 (additive) — PID 출력에 더함
                else if (FritExcitationInjector.TryGet(__instance, out float excite) && excite != 0f)
                {
                    modified = __result + excite;
                    changed = true;
                }
                else
                {
                    return;
                }

                // FTD 의 PID 출력은 [-1, 1] 범위. 클램프.
                if (modified > 1f) modified = 1f;
                else if (modified < -1f) modified = -1f;

                __result = modified;

                if (!changed) return;

                // LastControlVariable 동기화 — c.LastControlVariable 을 읽는 데이터 수집에서 일관.
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
