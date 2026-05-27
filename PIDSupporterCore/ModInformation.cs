// ============================================================================
// ModInformation.cs
// 역할: 모드 로드 시 ModProblems에 "{ModName} v{ver} Active!" 표시,
//       그리고 Steam Workshop description의 "Mod latest version X.Y.Z" 줄을
//       조회해서 새 버전 알림을 띄운다.
//
// 출처: AdvancedMimicUi(ModInformation.cs)의 로직을 거의 그대로 포팅.
// 차이점:
//  - 네임스페이스: ModManagement -> PIDSupporter
//  - Preparation()의 경로 탐색에 안전 가드(루트 도달/Mods 미발견) 추가
//
// 동작 흐름:
//  1) plugin.json 에서 version, workshop_id 읽음 (없으면 0.0.0 / 0)
//  2) "{name}  v{ver}  Active!" 를 ModProblems 에 등록
//  3) workshop_id != 0 이면 Twice_Second 이벤트로 SteamUGC 요청
//     -> callback 에서 description 의 "Mod latest version X.Y.Z" 파싱
//     -> 로컬 < 최신 이면 "New version released! v..." 를 추가 표시
//  4) workshop_id == 0 이면 SteamUGC 호출 없이 즉시 unregister
// ============================================================================

using BrilliantSkies.Core.Timing;
using BrilliantSkies.Modding;
using BrilliantSkies.Ui.Displayer;
using BrilliantSkies.Ui.Displayer.Types;
using Newtonsoft.Json.Linq;
using Steamworks;
using System;
using System.IO;
using System.Reflection;

namespace PIDSupporter
{
    internal static class ModInformation
    {
        private static string _name;
        private static string _myModDirPath;

        public static string MyModFolderPath
        {
            get
            {
                if (string.IsNullOrEmpty(_myModDirPath)) Preparation();
                return _myModDirPath;
            }
        }

        public static string MyModName
        {
            get
            {
                if (string.IsNullOrEmpty(_name)) Preparation();
                return _name;
            }
        }

        private static void Preparation()
        {
            string path1 = Assembly.GetExecutingAssembly().Location;
            string path2 = Path.GetDirectoryName(path1);

            // Mods 폴더를 만날 때까지 상위로 올라간다.
            while (!string.IsNullOrEmpty(path2) && Path.GetFileName(path2) != "Mods")
            {
                path1 = path2;
                path2 = Path.GetDirectoryName(path1);
            }

            // Mods 를 못 찾은 경우(특수 로딩 경로) 현재 어셈블리 폴더를 모드 루트로 간주.
            if (string.IsNullOrEmpty(path2))
            {
                _myModDirPath = Path.GetDirectoryName(Assembly.GetExecutingAssembly().Location);
                _name = Path.GetFileName(_myModDirPath);
                return;
            }

            _myModDirPath = path1;
            _name = Path.GetFileName(path1);
        }


        private static System.Version _modVersion = new System.Version(0, 0, 0);
        private static ulong _workshopID;
        private static int _requestCount;
        private static CallResult<SteamUGCRequestUGCDetailsResult_t> _steamCall;

        public static void VersionConfirmation()
        {
            GameEvents.Twice_Second.RegWithEvent(SteamUGCRequest);

            string pluginPath = Path.Combine(MyModFolderPath, "plugin.json");

            if (File.Exists(pluginPath))
            {
                JObject jObject = JObject.Parse(File.ReadAllText(pluginPath));

                JToken jobj1 = jObject["version"];
                JToken jobj2 = jObject["workshop_id"];

                if (jobj1 != null)
                {
                    _modVersion = System.Version.Parse(jobj1.ToString());
                }

                if (jobj2 != null)
                {
                    _workshopID = ulong.Parse(jobj2.ToString());
                }
            }

            ModProblemOverwrite($"{MyModName}  v{_modVersion}  Active!", MyModFolderPath, string.Empty, false);
        }

        private static void ModProblemOverwrite(string initModName, string initModPath, string initDescription, bool initIsError)
        {
            ModProblems.AllModProblems.Remove(initModPath);
            ModProblems.AddModProblem(initModName, initModPath, initDescription, initIsError);

            foreach (IGui_GuiSystem guiSystem in GuiDisplayer.GetSingleton().ActiveGuis)
            {
                guiSystem.OnActivateGui();
            }
        }

        private static void SteamUGCRequest(ITimeStep t)
        {
            if (_workshopID != 0 && ++_requestCount <= 5)
            {
                SteamAPICall_t ugcDetails = SteamUGC.RequestUGCDetails(new PublishedFileId_t(_workshopID), 0);
                _steamCall = new CallResult<SteamUGCRequestUGCDetailsResult_t>(Callback);
                _steamCall.Set(ugcDetails);
            }
            else
            {
                GameEvents.Twice_Second.UnregWithEvent(SteamUGCRequest);
            }
        }

        private static void Callback(SteamUGCRequestUGCDetailsResult_t param, bool bIOFailure)
        {
            GameEvents.Twice_Second.UnregWithEvent(SteamUGCRequest);

            string description = param.m_details.m_rgchDescription;
            if (string.IsNullOrEmpty(description)) return;

            using (StringReader reader = new StringReader(description))
            {
                string inputLine;
                System.Version latestVersion = null;

                while ((inputLine = reader.ReadLine()) != null)
                {
                    if (inputLine.StartsWith("Mod latest version "))
                    {
                        // prefix 길이로 정확히 잘라 leading space 가 남지 않게 한다.
                        latestVersion = System.Version.Parse(inputLine.Substring("Mod latest version ".Length));
                        break;
                    }
                }

                if (latestVersion != null && _modVersion.CompareTo(latestVersion) < 0)
                {
                    ModProblemOverwrite(MyModName, MyModFolderPath + "UpdateText", "New version released! v" + latestVersion, false);
                }
            }
        }
    }
}
