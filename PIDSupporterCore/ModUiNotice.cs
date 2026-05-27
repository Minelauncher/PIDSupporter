// ============================================================================
// ModUiNotice.cs
// 역할: 모드 로드/활성 상태를 FTD의 ModProblems(ActiveText)에 띄우는 유틸.
// 특징: 기능이 단순하고 다른 코드가 의존하기 쉬워서 "가급적 수정하지 않는" 파일.
// 언제 수정하나?
//  - 게임 업데이트로 ModProblems API/동작이 바뀐 경우
//  - ActiveText 경로/표시 정책을 바꾸고 싶은 경우(예: key 규칙 변경)
// 주의:
//  - key 규칙(modRoot + "\\ActiveText")을 바꾸면 중복 표시에 영향 가능.
// ============================================================================

using System.IO;
using System.Reflection;
using BrilliantSkies.Core.Logger;
using BrilliantSkies.Modding;

namespace PIDSupporter
{
    internal static class ModUiNotice
    {
        public static string GetModRoot()
        {
            string path1 = Assembly.GetExecutingAssembly().Location;
            string path2 = Path.GetDirectoryName(path1);

            // Mods 폴더를 만날 때까지 상위로 올라가며 루트를 찾는다
            while (!string.IsNullOrEmpty(path2) && Path.GetFileName(path2) != "Mods")
            {
                path1 = path2;
                path2 = Path.GetDirectoryName(path1);
            }

            // Mods를 못 찾으면(특수 로딩 경로) 현재 어셈블리 폴더를 루트로 간주
            if (string.IsNullOrEmpty(path2))
                return Path.GetDirectoryName(Assembly.GetExecutingAssembly().Location);

            return path1;
        }

        public static void ShowActive(string modName, string details = "")
        {
            string modRoot = GetModRoot();

            // "이 모드의 ActiveText"를 식별하는 고유 키
            // (같은 키로 Remove 후 Add 하므로 중복 표시 방지)
            string key = modRoot + "\\ActiveText";

            ModProblems.AllModProblems.Remove(key);
            ModProblems.AddModProblem($"{modName}", key, details ?? "", false);

            AdvLogger.LogInfo($"[{modName}] ModProblems posted: key={key}", LogOptions.None);
        }
    }
}
