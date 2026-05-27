// ============================================================================
// CoreEntry.cs
// 역할: Selector가 호출하는 Core 측 엔트리 포인트.
// - OnLoad/OnStart/OnSave 라이프사이클 진입점 제공
// - Harmony PatchAll을 딱 1번만 수행
//
// 보통은 고정. 아래 상황이면 수정:
//  - 패치 타이밍을 바꿔야 할 때(OnLoad vs OnStart)
//  - 설정/리소스 로드/GUI 초기화 등 Core 초기화 단계가 늘어날 때
// ============================================================================

using BrilliantSkies.Core.Logger;
using HarmonyLib;
using System.Reflection;

namespace PIDSupporter
{
    internal static class CoreEntry
    {
        private static bool _patched;

        public static void OnLoad()
        {
            // 로드 확인용 UI 표시(원하면)
            ModUiNotice.ShowActive("PIDSupporter", "Activate!");

            PatchAllOnce();
        }

        public static void OnStart()
        {
            PatchAllOnce();
        }

        public static void OnSave()
        {
        }

        private static void PatchAllOnce()
        {
            if (_patched) return;
            _patched = true;

            // Harmony ID는 모드 고유 문자열로 고정 권장
            Harmony harmony = new Harmony("PIDSupporter");
            harmony.PatchAll(Assembly.GetExecutingAssembly());

            AdvLogger.LogInfo("[PIDSupporter] Harmony PatchAll done (Core)", LogOptions.None);
        }
    }
}
