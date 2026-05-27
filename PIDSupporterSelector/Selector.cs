using BrilliantSkies.Core.Logger;
using BrilliantSkies.Modding;
using System;
using System.IO;
using System.Reflection;

/// <summary>
/// "Selector(부트스트랩)" 플러그인.
/// - 게임이 이 DLL을 먼저 로드/실행한다.
/// - 이 플러그인은 실제 기능(하모니 패치 등)이 들어있는 "Core DLL"을 런타임에 로드하고,
///   그 Core DLL 안의 엔트리 메서드(CoreEntry.OnLoad/OnStart/OnSave)를 호출한다.
/// 
/// 목적:
/// 1) FTD 모드 로더가 요구하는 GamePlugin 진입점은 가볍게 유지
/// 2) 의존 DLL(0Harmony.dll 등)을 확실히 같은 폴더에서 해결
/// 3) 메인 기능 DLL(PIDSupporterCore.dll)을 나중에 로드하여 안정적으로 패치 적용
/// </summary>
public class PIDSupporterSelectorPlugin : GamePlugin
{
    // ----------------------------
    // 1) GamePlugin 메타데이터
    // ----------------------------

    /// <summary>모드 이름(표시/식별용)</summary>
    public string Name => "PIDSupporter";

    /// <summary>모드 버전(표시/식별용)</summary>
    public Version Version => new Version(1, 0, 0);

    // (FTD 쪽 GamePlugin 인터페이스/관례에 따라
    //  name/version 소문자 프로퍼티도 요구될 수 있어서 같이 둔 것으로 보임)
    public string name => Name;
    public Version version => Version;


    // ----------------------------
    // 2) 전역 상태(한 번만 초기화하기 위한 플래그/캐시)
    // ----------------------------

    /// <summary>BootOnce가 이미 실행됐는지 여부</summary>
    private static bool _booted;

    /// <summary>현재 Selector DLL이 들어있는 모드 폴더 경로</summary>
    private static string _modDir;

    /// <summary>Core DLL(= HarmonyPatch 들어있는 DLL)을 로드한 Assembly 핸들</summary>
    private static Assembly _coreAsm;


    // ----------------------------
    // 3) FTD 모드 로더가 호출하는 라이프사이클
    // ----------------------------

    /// <summary>
    /// 게임이 모드를 "로드"할 때 호출되는 훅.
    /// - 1회 초기화(BootOnce) 보장
    /// - Core DLL의 PIDSupporter.CoreEntry.OnLoad() 호출 시도
    /// </summary>
    public void OnLoad()
    {
        BootOnce("OnLoad");
        InvokeCore("PIDSupporter.CoreEntry", "OnLoad");
    }

    /// <summary>
    /// 게임이 본격적으로 "시작"될 때 호출되는 훅.
    /// - 1회 초기화(BootOnce) 보장
    /// - Core DLL의 PIDSupporter.CoreEntry.OnStart() 호출 시도
    /// </summary>
    public void OnStart()
    {
        BootOnce("OnStart");
        InvokeCore("PIDSupporter.CoreEntry", "OnStart");
    }

    /// <summary>
    /// 게임 저장 시 호출되는 훅.
    /// - 1회 초기화(BootOnce) 보장
    /// - Core DLL의 PIDSupporter.CoreEntry.OnSave() 호출 시도
    /// </summary>
    public void OnSave()
    {
        BootOnce("OnSave");
        InvokeCore("PIDSupporter.CoreEntry", "OnSave");
    }


    // ----------------------------
    // 4) 부트스트랩(딱 1번만 실행)
    // ----------------------------

    /// <summary>
    /// Selector 플러그인의 핵심 초기화.
    /// - 모드 디렉토리 계산
    /// - AssemblyResolve 핸들러 설치 (의존 DLL을 모드 폴더에서 찾게 함)
    /// - 0Harmony.dll 선 로드
    /// - PIDSupporterCore.dll 로드 (실제 패치/로직 포함)
    /// </summary>
    private static void BootOnce(string phase)
    {
        // 이미 한 번 부팅했으면 다시 하지 않음
        if (_booted) return;
        _booted = true;

        // 현재 실행 중인(=Selector) 어셈블리의 실제 경로에서 폴더를 추출
        // 예: ...\From The Depths\Mods\PIDSupporter\
        _modDir = Path.GetDirectoryName(Assembly.GetExecutingAssembly().Location);

        // 로그로 부트 단계와 경로를 찍어서 "정말 여기서 로드되고 있나" 확인 가능
        AdvLogger.LogInfo($"[PIDSupporterSelector] Boot phase={phase} dir={_modDir}", LogOptions.None);

        // 같은 폴더에서 의존 DLL을 자동으로 찾아 로드하게 해주는 리졸버 설치
        InstallResolver(_modDir);

        // 1) Harmony를 먼저 로드 (Core DLL이 Harmony 타입을 참조할 가능성이 크기 때문)
        LoadDllIfExists("0Harmony.dll");

        LoadDllIfExists("MathNet.Numerics.dll");

        // 2) 실제 기능/패치가 들어있는 Core DLL 로드
        LoadCoreIfExists("PIDSupporterCore.dll");
    }


    // ----------------------------
    // 5) 의존 DLL 로드 실패 방지용 리졸버
    // ----------------------------

    /// <summary>
    /// .NET이 어떤 어셈블리를 못 찾았을 때(AssemblyResolve),
    /// "모드 폴더(dir)" 안에서 동일한 이름의 DLL을 찾아서 직접 로드해 주는 장치.
    ///
    /// 예)
    /// - Core DLL이 0Harmony.dll을 필요로 한다
    /// - 근데 로드 경로가 꼬이면 못 찾는 경우가 생길 수 있음
    /// - 그 때 여기서 dir\0Harmony.dll 찾아서 Assembly.LoadFrom로 해결
    /// </summary>
    private static void InstallResolver(string dir)
    {
        AppDomain.CurrentDomain.AssemblyResolve += (sender, args) =>
        {
            try
            {
                // args.Name은 보통 "어셈블리이름, Version=..., Culture=..., PublicKeyToken=..."
                // 여기서 "어셈블리이름"만 뽑아 "어셈블리이름.dll"로 만든다
                string asmName = new AssemblyName(args.Name).Name + ".dll";

                // 모드 폴더에 동일 이름 DLL이 있으면 그걸 로드
                string candidate = Path.Combine(dir, asmName);
                if (File.Exists(candidate))
                {
                    return Assembly.LoadFrom(candidate);
                }
            }
            catch
            {
                // 리졸버에서는 예외를 밖으로 던지면 로딩 흐름이 더 꼬일 수 있어서
                // 그냥 조용히 null 리턴(=다른 리졸브 경로 시도)하게 둠
            }

            // null을 반환하면 .NET은 "해결 실패"로 보고 다음 리졸버/기본 로딩을 시도함
            return null;
        };
    }


    // ----------------------------
    // 6) 파일 존재하면 Assembly.LoadFrom로 로드
    // ----------------------------

    /// <summary>
    /// 모드 폴더에 fileName이 있으면 로드하고 로그를 남김.
    /// 없으면 "Missing" 로그만 찍고 끝.
    /// </summary>
    private static void LoadDllIfExists(string fileName)
    {
        string path = Path.Combine(_modDir, fileName);

        // 파일이 없다면 로드 시도하지 않음(=크래시 방지)
        if (!File.Exists(path))
        {
            AdvLogger.LogInfo($"[PIDSupporterSelector] Missing {fileName} at {path}", LogOptions.None);
            return;
        }

        // 해당 DLL을 명시적으로 로드
        Assembly.LoadFrom(path);
        AdvLogger.LogInfo($"[PIDSupporterSelector] Loaded {fileName}", LogOptions.None);
    }


    // ----------------------------
    // 7) Core DLL 로드 + Assembly 핸들 보관
    // ----------------------------

    /// <summary>
    /// Core DLL(PIDSupporterCore.dll)을 로드하고 _coreAsm에 저장.
    /// 이후 InvokeCore가 이 Assembly 안에서 타입/메서드를 찾아 실행한다.
    /// </summary>
    private static void LoadCoreIfExists(string coreFileName)
    {
        string path = Path.Combine(_modDir, coreFileName);

        if (!File.Exists(path))
        {
            AdvLogger.LogInfo($"[PIDSupporterSelector] Missing core at {path}", LogOptions.None);
            return;
        }

        // Core DLL 로드 후 Assembly 참조 저장
        _coreAsm = Assembly.LoadFrom(path);
        AdvLogger.LogInfo($"[PIDSupporterSelector] Loaded core: {coreFileName}", LogOptions.None);
    }


    // ----------------------------
    // 8) Core DLL의 엔트리 메서드 호출(Reflection)
    // ----------------------------

    /// <summary>
    /// Core DLL 내부의 특정 타입(typeName)에서 정적 메서드(methodName)를 찾아 호출한다.
    ///
    /// 기대 구조 예:
    /// namespace PIDSupporter {
    ///   public static class CoreEntry {
    ///     public static void OnStart() { ... Harmony.PatchAll ... }
    ///   }
    /// }
    ///
    /// 여기서는 "PIDSupporter.CoreEntry" 타입의
    /// "OnLoad/OnStart/OnSave" 같은 메서드를 호출하려고 한다.
    /// </summary>
    private static void InvokeCore(string typeName, string methodName)
    {
        try
        {
            // Core DLL이 아직 로드 안됐으면 호출 불가
            if (_coreAsm == null)
            {
                AdvLogger.LogInfo("[PIDSupporterSelector] Core assembly not loaded. Skip invoke.", LogOptions.None);
                return;
            }

            // 타입 찾기 (throwOnError:false => 못 찾아도 예외 안 던지고 null)
            var t = _coreAsm.GetType(typeName, throwOnError: false);

            // 정적 메서드 찾기 (public/private 모두 탐색)
            var m = t?.GetMethod(
                methodName,
                BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Static
            );

            // 타입 또는 메서드가 없으면 로그만 남기고 종료
            if (t == null || m == null)
            {
                AdvLogger.LogInfo($"[PIDSupporterSelector] Core method not found: {typeName}.{methodName}", LogOptions.None);
                return;
            }

            // 정적 메서드라서 instance=null, 파라미터도 없으니 args=null
            m.Invoke(null, null);

            // 호출 성공 로그
            AdvLogger.LogInfo($"[PIDSupporterSelector] Invoked: {typeName}.{methodName}", LogOptions.None);
        }
        catch (Exception e)
        {
            // 리플렉션 호출은 예외가 잦을 수 있으니 전체 예외를 로그로 남김
            AdvLogger.LogInfo("[PIDSupporterSelector] InvokeCore failed: " + e, LogOptions.None);
        }
    }
}
