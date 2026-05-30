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

using System;
using BrilliantSkies.Core.Logger;
using HarmonyLib;
using System.Reflection;
using UnityEngine.SceneManagement;

namespace PIDSupporter
{
    internal static class CoreEntry
    {
        private static bool _patched;
        private static bool _versionShown;

        public static void OnLoad()
        {
            PatchAllOnce();

            // FTD 의 GamePlugin 라이프사이클은 실제로 OnLoad 만 안정적으로 호출됨
            // (로그 확인: OnStart 자국 없음). 따라서 VersionConfirmation 은 OnStart 에
            // 두지 않고 SceneManager.sceneLoaded 첫 발생 시 호출 — BreadThing 도 동일 패턴.
            // 이유: ModProblemOverwrite 가 GuiDisplayer 싱글톤을 건드리는데, OnLoad 시점엔
            // 아직 GuiDisplayer / ProfileManager 가 준비 안 됨.
            SceneManager.sceneLoaded += OnSceneLoadedOnce;
        }

        public static void OnStart()
        {
            // FTD 가 실제로는 호출하지 않지만 인터페이스 계약상 남겨둠.
            PatchAllOnce();
        }

        public static void OnSave()
        {
        }

        private static void OnSceneLoadedOnce(Scene scene, LoadSceneMode mode)
        {
            if (_versionShown) return;
            _versionShown = true;
            SceneManager.sceneLoaded -= OnSceneLoadedOnce;
            try
            {
                ModInformation.VersionConfirmation();
            }
            catch (Exception e)
            {
                AdvLogger.LogInfo("[PIDSupporter] VersionConfirmation failed: " + e, LogOptions.None);
            }
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
