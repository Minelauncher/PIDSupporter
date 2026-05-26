// ============================================================================
// FritTuningTab.cs — FRIT 기반 PID 자동 튜닝 UI 탭 (전체 핵심 로직)
//
// ■ using 설명 (C# 기본):
//   using = "이 네임스페이스의 클래스를 쓰겠다"는 선언.
//   Java의 import, Python의 from X import * 와 비슷.
//
// ============================================================================
// ■ 메서드/타입 출처 레퍼런스 (코드 읽을 때 참고)
//
// ── 출처 범례 ──
//   [C#]     = C# / .NET 기본 라이브러리
//   [FTD]    = FTD 게임 DLL (BrilliantSkies.*)
//   [Unity]  = Unity 엔진 (UnityEngine.*)
//   [MathNet]= MathNet.Numerics 라이브러리
//   [자체]   = 이 모드에서 직접 만든 코드
//
// ── 이 파일의 메서드들 ──
//
//   [자체] FritTuningTab(window, focus)     생성자. FTD SuperScreen 상속
//   [자체] Build()                          override. UI 요소 배치 (FTD가 호출)
//   [자체] OnUiFixed()                      매 물리 틱 호출. 데이터 수집/적응형 진폭
//   [자체] BuildStatus()                    UI: 상태 표시 영역 생성
//   [자체] BuildSettingsSliders()            UI: 설정 슬라이더들 생성
//   [자체] BuildExcitationControls()         UI: 가진 설정 영역 생성
//   [자체] BuildActionButtons()              UI: 버튼들 (자동튜닝/녹화/계산/적용)
//   [자체] BuildResult()                    UI: 결과 표시 영역
//   [자체] StartRecording()                 녹화 시작 (세션 초기화)
//   [자체] StopRecording()                  녹화 중지 (SP 복원)
//   [자체] AutoTuneNow()                    [자동 튜닝] 버튼 → 가진 설정 + 녹화 시작
//   [자체] AutoTuneCompute()                녹화 완료 후 → 추정 + FRIT 계산
//   [자체] CaptureSetPointAdjustBase()      현재 SP 백업
//   [자체] RestoreSetPointAdjustIfNeeded()   SP를 원래 값으로 복원
//   [자체] ApplyExcitation(dt)              매 틱 SP에 가진 신호 더하기
//   [자체] ComputeNow()                     [계산] 버튼 → FRIT 계산 (수동)
//   [자체] ApplyToPid()                     [적용] 버튼 → 결과를 게임 PID에 쓰기
//   [자체] ComputeFritPid(u,y,dt,s)         ★ FRIT 핵심 계산 (static)
//   [자체] MakeButton(...)                  UI 헬퍼: 버튼 생성
//   [자체] MakeToggle(...)                  UI 헬퍼: 토글 생성
//   [자체] MakeCycleButton(...)             UI 헬퍼: 순환 버튼 생성
//   [자체] MakeSliderFloat(...)             UI 헬퍼: float 슬라이더 생성
//   [자체] MakeSliderInt(...)               UI 헬퍼: int 슬라이더 생성
//   [자체] WaveToKo(w)                      WaveType → 한국어 문자열
//   [자체] Clamp(v,lo,hi)                   값 범위 제한 (float)
//   [자체] ClampInt(v,lo,hi)                값 범위 제한 (int)
//   [자체] RoundToStep(v,step)              step 단위로 반올림
//   [자체] NextPow2(n)                      2의 거듭제곱으로 올림
//   [자체] Detrend(x)                       DC+선형추세 제거 (in-place)
//   [자체] StdDev(data)                     표준편차 계산
//   [자체] EstimateDelay(u,y,dt)            임펄스 응답 기반 지연 추정
//   [자체] EstimateSettlingTime(y,dt)       자기상관 기반 정착시간 추정
//
// ── 사용하는 외부 타입/메서드 ──
//
//   [FTD]    SuperScreen<T>                 UI 탭 기본 클래스 (상속)
//   [FTD]    VariableControllerMaster       PID 제어기 객체 (this._focus)
//   [FTD]    IVariableController            개별 제어 채널 인터페이스
//     .GetCurrentController()               [FTD] 현재 활성 컨트롤러 반환
//     .LastControlVariable                  [FTD] 마지막 제어 출력 (u)
//     .LastProcessVariable                  [FTD] 마지막 프로세스 변수 (y)
//   [FTD]    this._focus.SetPointAdjust.Us  SetPoint 오프셋 (읽기/쓰기)
//   [FTD]    this._focus.Pid.kP.Us          PID의 Kp 값 (읽기/쓰기)
//   [FTD]    this._focus.Pid.kI.Us          PID의 Ti 값 (읽기/쓰기)
//   [FTD]    this._focus.Pid.kD.Us          PID의 Td 값 (읽기/쓰기)
//   [FTD]    ConsoleWindow                  UI 창 객체
//   [FTD]    ScreenSegmentStandard          UI 구획 (세로)
//   [FTD]    ScreenSegmentTable             UI 구획 (테이블)
//   [FTD]    ScreenSegmentStandardHorizontal UI 구획 (가로)
//   [FTD]    SubjectiveDisplay<T>           읽기 전용 텍스트 표시
//   [FTD]    SubjectiveButton<T>            클릭 버튼
//   [FTD]    SubjectiveToggle<T>            ON/OFF 토글
//   [FTD]    SubjectiveFloatClampedWithBar<T> 슬라이더 (범위 제한 float)
//   [FTD]    M.m<T>(값)                     매 프레임 값을 갱신하는 래퍼 (UI용)
//   [FTD]    Content(텍스트, 툴팁, 태그)    탭/UI 이름 + 설명 묶음
//   [FTD]    ToolTip(텍스트, 폭)            마우스 올리면 나오는 설명
//   [FTD]    InsertPosition.OnCursor        UI 요소 삽입 위치
//   [FTD]    ConsoleStyles.Instance          UI 스타일 싱글톤
//   [FTD]    base.CreateStandardSegment()   부모(SuperScreen)의 UI 구획 생성
//   [FTD]    base.CreateTableSegment(열,행) 부모(SuperScreen)의 테이블 생성
//   [FTD]    base.CreateStandardHorizontalSegment() 가로 구획 생성
//   [FTD]    seg.AddInterpretter(...)       구획에 UI 요소 추가
//
//   [Unity]  Time.fixedDeltaTime            물리 틱 간격 (보통 0.02초)
//
//   [MathNet] Fourier.Forward(data, opt)    FFT (시간→주파수)
//   [MathNet] Fourier.Inverse(data, opt)    IFFT (주파수→시간)
//   [MathNet] FourierOptions.Matlab         FFT 스케일링 옵션 (MATLAB 호환)
//   [MathNet] Matrix<double>.Build.Dense()  행렬 생성
//   [MathNet] Vector<double>.Build.DenseOfArray() 벡터 생성
//   [MathNet] matrix.Svd()                  특이값 분해 (SVD)
//   [MathNet] matrix.QR().Solve(b)          QR 분해 → 최소자승 풀기
//   [MathNet] svd.S                         특이값 벡터
//   [MathNet] svd.VT                        V 전치 행렬
//
//   [C#]     Math.Sin/Cos/Sqrt/Abs/...      기본 수학 함수
//   [C#]     Math.Clamp(v,min,max)          범위 제한
//   [C#]     Complex                        복소수 (실수부 + 허수부)
//   [C#]     Complex.Exp/Pow/Conjugate      복소수 연산
//   [C#]     Array.Copy(src,dst,len)        배열 복사
//   [C#]     List<T>.Add/Clear/Count/CopyTo 리스트 조작
//   [C#]     string.IsNullOrEmpty(s)        null 또는 빈 문자열 체크
//   [C#]     $"...{변수}..."                문자열 보간 (f-string과 동일)
//   [C#]     () => 표현식                   람다 (Python의 lambda와 동일)
//   [C#]     Action<T> / Func<T>            함수를 변수로 전달하는 타입
//   [C#]     try/catch                      예외 처리
//   [C#]     double.IsNaN/IsInfinity        숫자 유효성 체크
// ============================================================================

using System;                          // 기본 타입 (Math, Array, Exception 등)
using System.Collections.Generic;      // List<T>, Dictionary 등 컬렉션
using System.Linq;                     // Enumerable (ERA에서 사용)
using System.Numerics;                 // Complex (복소수) — FFT 계산에 필수
using BrilliantSkies.Ai.Control.Pids;  // FTD PID 관련 (VariableControllerMaster, IVariableController 등)
// using BrilliantSkies.Core.Control.Tuning; // removed: OpenLoopCollector 제거됨
using BrilliantSkies.Ui.Consoles;      // FTD UI 시스템 (ConsoleWindow, SuperScreen 등)
using BrilliantSkies.Ui.Consoles.Getters;                          // M.m<T> — UI 값 갱신 래퍼
using BrilliantSkies.Ui.Consoles.Interpretters.Subjective;         // SubjectiveDisplay 등
using BrilliantSkies.Ui.Consoles.Interpretters.Subjective.Buttons; // SubjectiveButton
using BrilliantSkies.Ui.Consoles.Interpretters.Subjective.Choices; // SubjectiveToggle
using BrilliantSkies.Ui.Consoles.Interpretters.Subjective.Numbers; // SubjectiveFloatClampedWithBar (슬라이더)
using BrilliantSkies.Ui.Consoles.Segments;  // ScreenSegmentStandard 등 (UI 구획)
using BrilliantSkies.Ui.Consoles.Styles;    // ConsoleStyles (UI 스타일/테마)
using BrilliantSkies.Ui.Tips;               // ToolTip (마우스 올리면 나오는 설명)
using MathNet.Numerics.IntegralTransforms;  // Fourier (FFT/IFFT)
using MathNet.Numerics.LinearAlgebra;       // Matrix, Vector (선형대수 — SVD, QR 등)
using UnityEngine;                          // Time.fixedDeltaTime (Unity 물리 틱 간격)

namespace PIDAutoTuner
{
    /// <summary>
    /// FRIT(Fictitious Reference Iterative Tuning) 기반 PID 자동 튜닝 UI 탭.
    ///
    /// ■ FRIT가 뭔가?
    ///   초기 PID C₀ 로 수집한 폐루프 데이터 (u, y) 만으로,
    ///   원하는 폐루프 응답 M(s) 에 출력이 가장 가까워지는 새 PID 파라미터를 찾는 방법.
    ///   플랜트 모델 식별 불필요. 측정 노이즈에 상대적으로 강건 (vs VRFT 선형 회귀).
    ///
    /// ■ 핵심 수식:
    ///   M(jw) = exp(-jw·τM) / (1 + jw·0.2·Ts)^nM
    ///           → 목표 폐루프 응답. τM=지연, Ts=정착시간, nM=차수 (보통 2).
    ///
    ///   가상 레퍼런스:  r̃(θ)[k] = y[k] + (1/C(θ)) · u[k]
    ///   참조 모델 응답: ŷ(θ)[k] = M · r̃(θ)[k]
    ///   비용:           J(θ) = Σ (y[k] - ŷ(θ)[k])²    ← Levenberg-Marquardt 최적화
    ///
    ///   θ = (Kp, Ti, Td) 3개 파라미터 비선형 최적화 (MathNet LM).
    ///
    /// ■ 주파수 영역 구현:
    ///   C(z)^{-1} 시간 도메인 IIR 필터의 zero 안정성 문제 회피.
    ///   제로패딩 (Nfft = 2N) 으로 circular convolution wrap-around 회피.
    ///
    /// ■ C# 클래스 구조:
    ///   SuperScreen{T} = FTD UI 시스템의 "탭 화면" 기본 클래스.
    ///   T = VariableControllerMaster = FTD의 PID 제어기 객체.
    ///   이 클래스를 상속하면 FTD UI 창 안에 탭으로 들어갈 수 있다.
    ///   this._focus = 부모 클래스에서 물려받는 필드, 현재 편집 중인 PID 제어기.
    /// </summary>
    public class FritTuningTab : SuperScreen<VariableControllerMaster>
    {
        // ── MathNet 편의 팩토리 ──
        // C#에서 System.Numerics.Vector{T}와 MathNet의 Vector{T}가 이름이 겹쳐서
        // "어떤 Vector?"라는 모호성 오류가 남. 여기서 MathNet 것을 명시적으로 선언.
        // MB.Dense(행, 열) -> 행렬 생성,  VB.DenseOfArray(배열) -> 벡터 생성
        private static readonly MatrixBuilder<double> MB = Matrix<double>.Build;
        private static readonly VectorBuilder<double> VB = MathNet.Numerics.LinearAlgebra.Vector<double>.Build;

        // ■ enum = 이름 붙인 정수 상수 모음. Python의 Enum과 동일.
        //   private = 이 클래스 안에서만 사용 가능.

        /// <summary>가진(excitation) 파형 종류</summary>
        private enum WaveType
        {
            Off = 0,        // 가진 없음
            Sine = 1,       // 단일 사인파
            Chirp = 2,      // 주파수 스윕 (시간에 따라 주파수 증가)
            MultiSine = 3   // 여러 주파수 사인파 합성
        }

        /// <summary>자동 튜닝 상태 머신</summary>
        private enum AutoTuneState
        {
            Idle,       // 대기 중
            Recording,  // 데이터 수집 중 (폐루프)
            Computing,  // 수집 끝, FRIT 계산 중
            Done,       // 계산 완료 (결과 있음)
            Failed,     // 실패 (에러 메시지 있음)
            Validating, // 검증 모드: 전 축 y 수집 중 (5초)
        }

        /// <summary>축 타입 — 사용자가 각 tab 에서 지정. 피치 고도유지 로직 등에 사용.</summary>
        public enum AxisType
        {
            Unspecified,  // 미지정 (기본값)
            Yaw, Roll, Pitch, Hover, Forward, Strafe
        }

        // ■ sealed class = 상속 불가 클래스. "이 클래스를 더 확장하지 않겠다"는 의미.
        //   private = FritTuningTab 안에서만 사용.
        //   Settings는 UI 슬라이더와 연결되는 "설정값 묶음".

        /// <summary>FRIT 튜닝에 사용되는 모든 설정값</summary>
        private sealed class Settings
        {
            // ===== 참조모델 M: "폐루프가 이렇게 반응했으면 좋겠다" =====
            public float SettlingTimeTs = 2.0f;     // t_s: 목표 정착시간 (초). 작을수록 빠른 응답 요구
            public int ModelOrderNm = 2;            // n_M: 모델 차수. 높을수록 오버슈트 적지만 느림
            public float ModelDelayTau = 0.0f;      // tau_M: 목표 지연 (초). 자동 추정됨

            // ===== 가중 필터 W: 고주파 노이즈 억제 =====
            public float CutoffHz = 30.0f;          // f_W: 컷오프 주파수 (Hz). 이 위 주파수는 무시

            // ===== 녹화/전처리 =====
            public int MinSamples = 1024;           // 최소 수집 샘플 수 (FFT 해상도에 영향)
            public int DropEdgeSamples = 64;        // FFT 양끝 아티팩트 버릴 개수

            // ===== 포화 처리 =====
            public float SaturationThreshold = 0.98f;   // |u| >= 이 값이면 포화로 판정
            public int   TransientTailSamples = 100;    // 포화 이후 IIR 회복 transient 구간 (≈2초)
            public float MaxRecordingSec = 60.0f;       // 수집 안전 상한 (이 시간 넘으면 fail or 강제 종료)

            // ===== 가진(Excitation): 플랜트를 흔들어서 데이터를 만드는 신호 =====
            public bool ExciteEnabled = true;       // 가진 켤지
            public WaveType ExciteWave = WaveType.Sine; // 가진 파형 종류
            public float ExciteAmp = 0.5f;          // 가진 진폭 (SetPoint에 더해지는 크기)
            public float ExciteFreqHz = 0.6f;       // Sine/MultiSine 기본 주파수 (Hz)
            public float ChirpStartHz = 0.2f;       // Chirp 시작 주파수
            public float ChirpEndHz = 2.0f;         // Chirp 끝 주파수

            // ===== 저주파 square wave (적분 모드 식별용 DC 보강) =====
            // MultiSine 가진 위에 sign(sin(2π·f_sq·t)) 를 더해서 적분기에 sustained 오차 주입.
            // 각 half-period 동안 SP 일정 → Ti 식별이 강해짐. 평균 0 이라 자세 bias 없음.
            public float SquareAmpRatio = 0.5f;     // 0 = off, 멀티사인 대비 square 진폭 비율
            public float SquareFreqHz   = 0.1f;     // square 주파수 (주기 10초)

            // ===== 적응형 진폭: PID가 가진을 다 눌러버릴 때 자동으로 키움 =====
            public bool  AdaptiveAmp = true;        // 적응형 켤지
            public float AdaptiveAmpMax = 10.0f;    // 최대 허용 진폭 (안전 상한)

            // ===== 축 분리 (Axis Fixture) =====
            public bool FixOtherAxes = true;        // 튜닝 중 다른 축 SP 고정
            public AxisType AxisKind = AxisType.Unspecified;  // 이 탭의 축 타입
            public float PitchAltHoldGain = 0.01f;  // 고도 오차 (m) → 피치 SP 오프셋 스케일
            public float PitchAltHoldClamp = 0.3f;  // 피치 SP 오프셋 최대 크기
        }

        /// <summary>
        /// 현재 녹화 세션의 실시간 상태.
        /// 녹화 시작 시 Clear()로 초기화, 매 틱마다 U/Y에 데이터 추가.
        /// readonly = "리스트 객체 자체는 교체 불가, 안에 요소 추가/삭제는 가능"
        /// </summary>
        private sealed class Session
        {
            public bool Recording;                   // 지금 녹화 중인지
            public double T;                          // 경과 시간 (초)

            public readonly List<double> U = new List<double>();          // 제어 출력 기록 (전 샘플)
            public readonly List<double> Y = new List<double>();          // 프로세스 변수 기록 (전 샘플)
            public readonly List<bool>   Saturated = new List<bool>();    // 이 샘플이 포화 중인지 (가중치용)

            // ── 포화 회복 추적 ──
            // 포화 끝난 직후 1/C(z) IIR 역필터 state 가 회복하는 데 ~2초 (TransientTailSamples) 소요.
            // 그 동안 계산되는 e[k] 가 오염됨 → 해당 인덱스는 FRIT 가중치에서 down-weight.
            public int SaturatedCount;
            public int SamplesSinceLastSat;   // 마지막 포화 이후 경과 샘플 수
            public int EffectiveValidCount;   // transient tail 밖에 있는 깨끗한 샘플 누적

            // 적응형 진폭 상태 (saturation 기반 — u 만 봄)
            public double AdaptiveCurrentAmp;       // 현재 실제 적용 진폭
            public int    AdaptiveWindowSat;        // 윈도우 내 포화 카운트
            public double AdaptiveWindowUPeak;      // 윈도우 내 max |u|
            public int    AdaptiveWindowCount;      // 윈도우 누적 샘플 수
            public int    AdaptiveCheckInterval = 60;  // 윈도우 크기 (≈1.2초)
            public int    AdaptiveBoostCount;       // 진폭 ↑ 횟수 (표시용)
            public double AdaptiveLastChangeT;      // 마지막 변경 시각 (쿨다운)
            public double LastU;                    // 마지막 제어 출력 (가진 회피용)
            public double NaturalYStd;              // 가진 전 자연 변동 (시작 진폭 결정)

            public bool HasResult;
            public double Kp, Ti, Td;
            public double KpSE, TiSE, TdSE;     // Cramér-Rao 표준오차 (NaN if 계산 실패)
            public double FitRmse;

            public string LastMessage = "";

            // ── Validation (검증) ──
            public double ValidateStartT;                      // 검증 시작 시간
            public readonly List<List<double>> ValidateY = new List<List<double>>(); // 축별 y 기록

            public void Clear()
            {
                Recording = false;
                T = 0;
                U.Clear();
                Y.Clear();
                Saturated.Clear();
                SaturatedCount = 0;
                SamplesSinceLastSat = int.MaxValue / 2;   // 시작 시 "이미 충분히 오래 깨끗" 상태
                EffectiveValidCount = 0;
                AdaptiveCurrentAmp = 0;
                AdaptiveWindowSat = 0;
                AdaptiveWindowUPeak = 0;
                AdaptiveWindowCount = 0;
                AdaptiveBoostCount = 0;
                AdaptiveLastChangeT = 0;
                LastU = 0;
                NaturalYStd = 0;
                HasResult = false;
                Kp = Ti = Td = FitRmse = 0;
                KpSE = TiSE = TdSE = double.NaN;
                LastMessage = "";
                ValidateStartT = 0;
                ValidateY.Clear();
            }
        }

        // ── 인스턴스 필드 ──
        // _s: 설정값 (슬라이더와 연결)
        // _sess: 현재 녹화 세션 상태 (데이터, 결과 등)
        // _autoState: 자동 튜닝 상태 머신
        private readonly Settings _s = new Settings();
        private readonly Session _sess = new Session();
        private AutoTuneState _autoState = AutoTuneState.Idle;


        // ── 축 분리 모드 (Axis Fixture) ──
        // 방법 1: 리플렉션으로 _focus 의 부모 객체에서 형제 VariableControllerMaster 자동 발견.
        // 방법 2: 사용자가 각 축 PID UI 열면 자동 등록 (_tabsByAxis).
        // 두 방법 병합 — 리플렉션 성공하면 자동, 실패하면 수동 등록 폴백.
        private static readonly Dictionary<VariableControllerMaster, FritTuningTab> _tabsByAxis
            = new Dictionary<VariableControllerMaster, FritTuningTab>();
        private static bool _axisDiscoveryAttempted = false;
        private readonly Dictionary<VariableControllerMaster, float> _frozenOtherSPs
            = new Dictionary<VariableControllerMaster, float>();

        // ── 피치 고도 유지 (Pitch Altitude Hold) ──
        // 비행기형 기체는 피치를 고도 제어에 사용 → 튜닝 중 피치 SP 를 고정하면 고도 드리프트.
        // 해결: Hover 축이 등록되어 있으면 그 PV 를 고도 기준으로 사용, 피치 SP 에 실시간 offset 주입.
        //   pitchOffset = clamp(K_alt · (startAlt - currentAlt), ±clampMax)
        // 가진 0.05~2Hz vs 고도 루프 ~0.01Hz → 대역 분리 → 피치 SISO 데이터 깨끗.
        private VariableControllerMaster _altitudeSourceAxis;  // Hover 로 지정된 축 (PV=고도)
        private VariableControllerMaster _pitchTargetAxis;     // Pitch 로 지정된 축 (SP 받음)
        private bool _altHoldActive;
        private double _altHoldStartAltitude;

        // 가진 적용 시 원래 SetPoint를 백업해두고, 녹화 끝나면 복원하기 위한 변수.
        // SetPointAdjust = FTD에서 PID의 목표값을 외부에서 조절하는 파라미터.
        private bool _hasBaseSetPointAdjust;
        private float _baseSetPointAdjust;

        // 자연 변동 측정: 녹화 전 y를 링버퍼에 모아서 std 계산
        private const int NaturalBufSize = 60; // 약 1.2초 분량
        private readonly double[] _naturalYBuf = new double[NaturalBufSize];
        private int _naturalYIdx = 0;
        private int _naturalYCount = 0;

        /// <summary>
        /// 생성자. FTD가 PID 편집 UI를 열 때 패치에서 호출.
        /// : base(window, focus) = 부모 클래스(SuperScreen) 생성자에 window와 focus를 넘김.
        /// this._focus = focus (부모에서 설정됨) → 이후 this._focus로 PID 제어기에 접근.
        /// </summary>
        public FritTuningTab(ConsoleWindow window, VariableControllerMaster focus) : base(window, focus)
        {
            // Name = 탭 이름. Content(표시텍스트, 툴팁, 내부ID)
            this.Name = new Content("FRIT Tuning / FRIT 튜닝", new ToolTip("Auto-estimate PID (Kp, Ti, Td) via FRIT.\n---\nFRIT로 PID(Kp, Ti, Td)를 자동 추정합니다.", 220f), "frit");

            // 정적 registry 에 등록 — 다른 축 튜닝 시 이 축 SP 고정 대상으로 사용
            if (focus != null) _tabsByAxis[focus] = this;
        }

        /// <summary>
        /// FTD UI 시스템이 탭을 그릴 때 호출. UI 요소들을 여기서 생성/배치.
        /// override = 부모 클래스의 같은 이름 메서드를 덮어쓰기.
        /// </summary>
        public override void Build()
        {
            BuildStatus();              // 상태 표시 영역
            BuildSettingsSliders();     // 설정 슬라이더들
            BuildExcitationControls();  // 가진 설정 (파형/진폭/주파수)
            BuildActionButtons();       // 버튼들 (자동튜닝, 녹화, 계산, 적용)
            BuildResult();              // 결과 표시 (Kp, Ti, Td, RMSE)
        }

        // ============================================================
        // FixedUpdate tick — 매 물리 프레임(보통 0.02초=50Hz)마다 호출됨.
        // VariableControllerUiFixedUpdatePatch가 Harmony로 FTD 코드에 끼어들어서
        // 이 메서드를 호출해 줌. 이게 이 모드의 "심장박동".
        //
        // 하는 일:
        // 1) 가진 신호를 SetPoint에 더함
        // 2) u(제어 출력), y(프로세스 변수) 읽어서 저장
        // 3) 적응형 진폭 조절
        // 4) 포화 샘플 처리
        // 5) 자동 튜닝: 충분히 모이면 계산 단계로 전환
        // ============================================================
        public void OnUiFixed()
        {
            try
            {
                if (this._focus == null)
                {
                    _sess.LastMessage = "focus is null / focus가 null입니다.";
                    StopRecording();
                    return;
                }


                // 자동 튜닝 Computing 상태 처리 (녹화 중지 후 다음 틱)
                if (_autoState == AutoTuneState.Computing)
                {
                    try { AutoTuneCompute(); }
                    catch (Exception e)
                    {
                        _autoState = AutoTuneState.Failed;
                        _sess.LastMessage = "Auto-tune failed / 자동 튜닝 실패: " + e.Message;
                    }
                    return;
                }

                // 검증 모드 틱 처리 (가진 없이 전 축 y 수집)
                if (_autoState == AutoTuneState.Validating)
                {
                    double dtVal = Time.fixedDeltaTime;
                    if (dtVal <= 0) dtVal = 0.02;
                    OnValidateTick(dtVal);
                    return;
                }

                if (!_sess.Recording)
                {
                    RestoreSetPointAdjustIfNeeded();

                    // 녹화 전 자연 변동 측정: y를 링버퍼에 수집
                    IVariableController cIdle = this._focus.GetCurrentController();
                    if (cIdle != null)
                    {
                        _naturalYBuf[_naturalYIdx % NaturalBufSize] = cIdle.LastProcessVariable;
                        _naturalYIdx++;
                        if (_naturalYCount < NaturalBufSize) _naturalYCount++;
                    }
                    return;
                }

                double dt = Time.fixedDeltaTime;
                if (dt <= 0) dt = 0.02;

                ApplyExcitation((float)dt);
                ApplyOtherAxesFixture();  // 다른 축 SP 재적용 (매 틱) — 자세/고도 유지

                IVariableController c = this._focus.GetCurrentController();
                if (c == null) return;

                // u: 제어 출력(컨트롤 변수), y: 프로세스 변수, sp: 목표값
                double u = c.LastControlVariable;
                double y = c.LastProcessVariable;
                _sess.LastU = u;

                // 포화 추적 + transient tail 카운터
                bool saturated = Math.Abs(u) >= _s.SaturationThreshold;
                if (saturated)
                {
                    _sess.SaturatedCount++;
                    _sess.SamplesSinceLastSat = 0;
                }
                else
                {
                    _sess.SamplesSinceLastSat++;
                }

                // ── 적응형 진폭 — saturation 기반 binary 규칙 ──
                // 윈도우 통계 (uPeak, satCount) 만 보고 ↑/↓ 결정. y 는 안 봄 (FRIT 이론적으로 y info 는 amp ↑ 하면 자연 증가).
                // 규칙: satRate > 2% 또는 uPeak > 0.85 → amp ÷ 1.5. 그 외 → amp × 1.5. 쿨다운 3초.
                if (_s.AdaptiveAmp && _s.ExciteEnabled && _autoState == AutoTuneState.Recording)
                {
                    if (saturated) _sess.AdaptiveWindowSat++;
                    double absU = Math.Abs(u);
                    if (absU > _sess.AdaptiveWindowUPeak) _sess.AdaptiveWindowUPeak = absU;
                    _sess.AdaptiveWindowCount++;

                    if (_sess.AdaptiveWindowCount >= _sess.AdaptiveCheckInterval)
                    {
                        const double SAT_RATE_THRESHOLD = 0.02;   // 2%
                        const double U_PEAK_THRESHOLD = 0.85;
                        const double AMP_UP = 1.5;
                        const double AMP_DOWN = 1.0 / 1.5;        // ≈ 0.667
                        const double AMP_COOLDOWN = 3.0;
                        const double AMP_FLOOR = 0.05;

                        double satRate = (double)_sess.AdaptiveWindowSat / _sess.AdaptiveWindowCount;
                        double uPeak = _sess.AdaptiveWindowUPeak;
                        double amp = Math.Max(AMP_FLOOR, _sess.AdaptiveCurrentAmp);
                        bool cooledDown = (_sess.T - _sess.AdaptiveLastChangeT) >= AMP_COOLDOWN;

                        bool tooHigh = (satRate > SAT_RATE_THRESHOLD) || (uPeak > U_PEAK_THRESHOLD);

                        if (cooledDown)
                        {
                            if (tooHigh && amp > AMP_FLOOR)
                            {
                                double newAmp = Math.Max(AMP_FLOOR, amp * AMP_DOWN);
                                _sess.AdaptiveCurrentAmp = newAmp;
                                _s.ExciteAmp = (float)newAmp;
                                _sess.AdaptiveLastChangeT = _sess.T;
                                _sess.LastMessage = $"Adaptive ↓ amp {amp:0.00}→{newAmp:0.00} (uPeak={uPeak:0.00}, satRate={satRate:P0})";
                            }
                            else if (!tooHigh && amp < _s.AdaptiveAmpMax)
                            {
                                double newAmp = Math.Min(_s.AdaptiveAmpMax, amp * AMP_UP);
                                _sess.AdaptiveCurrentAmp = newAmp;
                                _s.ExciteAmp = (float)newAmp;
                                _sess.AdaptiveBoostCount++;
                                _sess.AdaptiveLastChangeT = _sess.T;
                                _sess.LastMessage = $"Adaptive ↑ amp {amp:0.00}→{newAmp:0.00} (uPeak={uPeak:0.00}, sat 없음)";
                            }
                            // else: 경계에 도달 (위 cap 또는 아래 floor). 유지.
                        }

                        _sess.AdaptiveWindowSat = 0;
                        _sess.AdaptiveWindowUPeak = 0;
                        _sess.AdaptiveWindowCount = 0;
                    }
                }

                _sess.U.Add(u);
                _sess.Y.Add(y);
                _sess.Saturated.Add(saturated);

                // EffectiveValid: transient tail 밖에 있는 깨끗한 샘플만 카운트
                if (_sess.SamplesSinceLastSat > _s.TransientTailSamples)
                    _sess.EffectiveValidCount++;

                _sess.T += dt;

                if (_sess.U.Count % 240 == 0)
                {
                    _sess.LastMessage = $"Collecting... valid {_sess.EffectiveValidCount}/{_s.MinSamples}  (total {_sess.U.Count}, sat {_sess.SaturatedCount}, boost {_sess.AdaptiveBoostCount}) / 수집중... 유효 {_sess.EffectiveValidCount}/{_s.MinSamples}";
                }

                // 자동 튜닝 종료 조건
                if (_autoState == AutoTuneState.Recording)
                {
                    if (_sess.EffectiveValidCount >= _s.MinSamples)
                    {
                        StopRecording();
                        _autoState = AutoTuneState.Computing;
                        _sess.LastMessage = "Auto-tune: analyzing... / 자동 튜닝: 데이터 분석 중...";
                    }
                    else if (_sess.T > _s.MaxRecordingSec)
                    {
                        // 안전 상한 초과 — 최소 유효 샘플 256 이상이면 그래도 진행, 아니면 실패.
                        if (_sess.EffectiveValidCount >= 256)
                        {
                            StopRecording();
                            _autoState = AutoTuneState.Computing;
                            _sess.LastMessage = $"Timeout {_s.MaxRecordingSec:0}s — proceeding with {_sess.EffectiveValidCount} valid samples / 타임아웃, {_sess.EffectiveValidCount}개로 진행";
                        }
                        else
                        {
                            StopRecording();
                            _autoState = AutoTuneState.Failed;
                            _sess.LastMessage = $"Auto-tune failed: only {_sess.EffectiveValidCount} valid samples in {_s.MaxRecordingSec:0}s. Check PID / 자동 튜닝 실패: {_s.MaxRecordingSec:0}초 내 유효 샘플 {_sess.EffectiveValidCount}개. PID 점검 필요.";
                        }
                    }
                }
            }
            catch (Exception e)
            {
                _sess.LastMessage = "OnUiFixed error / 오류: " + e.Message;
                StopRecording();
            }
        }

        // ============================================================
        // UI builders
        // ============================================================

        private void BuildStatus()
        {
            ScreenSegmentStandard seg = base.CreateStandardSegment(InsertPosition.OnCursor);
            seg.BackgroundStyleWhereApplicable = ConsoleStyles.Instance.Styles.Segments.OptionalSegmentDarkBackgroundWithHeader.Style;
            seg.NameWhereApplicable = "Status / 상태";
            seg.SpaceAbove = 10f;
            seg.SpaceBelow = 10f;

            seg.AddInterpretter(new SubjectiveDisplay<VariableControllerMaster>(
                this._focus,
                M.m<VariableControllerMaster>(_ =>
                {
                    string rec;
                    if (_autoState == AutoTuneState.Validating)
                        rec = "Validating / 검증 중";
                    else if (_autoState == AutoTuneState.Computing)
                        rec = "Computing / 계산 중";
                    else if (_sess.Recording)
                        rec = "Recording / 녹화중";
                    else if (_autoState == AutoTuneState.Done)
                        rec = "Done / 완료";
                    else if (_autoState == AutoTuneState.Failed)
                        rec = "Failed / 실패";
                    else
                        rec = "Idle / 대기";
                    double dt = Time.fixedDeltaTime;

                    return
                        $"Status: {rec}\n" +
                        $"Valid: {_sess.EffectiveValidCount} / {_s.MinSamples}  (total {_sess.U.Count}, elapsed {_sess.T:0.0}s)\n" +
                        $"Saturated: {_sess.SaturatedCount}\n" +
                        $"FixedDeltaTime: {dt:0.000}s\n" +
                        $"Msg: {_sess.LastMessage}";
                }),
                M.m<VariableControllerMaster>(new ToolTip(
                    "Shows current FRIT recording status, sample count, and saturated/rejected samples.\nSamples accumulate every FixedUpdate during recording.\n---\n" +
                    "현재 FRIT 기록 상태와 샘플 수, 포화/제외된 샘플 수를 표시합니다.\n" +
                    "샘플은 녹화 중 FixedUpdate마다 누적됩니다.",
                    260f
                ))
            ));
        }

        private void BuildSettingsSliders()
        {
            ScreenSegmentTable table = base.CreateTableSegment(1, 10);
            table.BackgroundStyleWhereApplicable = ConsoleStyles.Instance.Styles.Segments.OptionalSegmentDarkBackgroundWithHeader.Style;
            table.NameWhereApplicable = "FRIT Settings / FRIT 설정";
            table.SpaceAbove = 10f;
            table.SpaceBelow = 10f;
            table.SqueezeTable = false;

            // t_s
            table.AddInterpretter(MakeSliderFloat(
                "Settling time t_s (s) / 정착시간 t_s (초)",
                "Target settling time. Smaller = faster response.\nAuto-tuning estimates this automatically.\n---\n목표 정착시간. 작을수록 빠른 응답.\n자동 튜닝 시 자동 추정됩니다.",
                () => _s.SettlingTimeTs,
                f => _s.SettlingTimeTs = Clamp(f, 0.2f, 60f),
                0.2f, 60f, 0.1f, "0.0", "Ts"
            ));

            // tau_M
            table.AddInterpretter(MakeSliderFloat(
                "Delay τ_M (s) / 지연 τ_M (초)",
                "Plant delay (dead-time). 0 = no delay.\nAuto-tuning estimates this automatically.\n---\n플랜트 지연. 0이면 지연 없음.\n자동 튜닝 시 자동 추정됩니다.",
                () => _s.ModelDelayTau,
                f => _s.ModelDelayTau = Clamp(f, 0f, 5f),
                0f, 5f, 0.01f, "0.00", "tau"
            ));

            // min samples
            table.AddInterpretter(MakeSliderInt(
                "Min samples / 최소 샘플 수",
                "Minimum samples for data collection.\nMore samples = better accuracy but longer wait.\n---\n데이터 수집 최소 샘플 수.\n많을수록 정확하지만 대기 시간이 길어집니다.",
                () => _s.MinSamples,
                v => _s.MinSamples = ClampInt(v, 256, 200000),
                256, 32768, 256, "0", "N"
            ));
        }

        private void BuildExcitationControls()
        {
            ScreenSegmentStandard seg = base.CreateStandardSegment(InsertPosition.OnCursor);
            seg.BackgroundStyleWhereApplicable = ConsoleStyles.Instance.Styles.Segments.OptionalSegmentDarkBackgroundWithHeader.Style;
            seg.NameWhereApplicable = "Excitation / 자극";
            seg.SpaceAbove = 10f;
            seg.SpaceBelow = 10f;

            seg.AddInterpretter(MakeToggle(
                "Enable excitation / 자극 사용",
                "Adds excitation signal to SetPointAdjust during recording.\nAuto-tuning configures this automatically.\n---\n녹화 중 SetPointAdjust에 가진 신호를 더합니다.\n자동 튜닝 시 자동 설정됩니다.",
                () => _s.ExciteEnabled,
                b => _s.ExciteEnabled = b,
                "excite"
            ));

            // Axis type 선택 (cycle 버튼)
            seg.AddInterpretter(MakeCycleButton(
                "Axis type / 축 타입",
                "Mark this tab's axis so cross-axis features work correctly.\n" +
                "Hover axis's PV = altitude (used for pitch altitude-hold).\n" +
                "Pitch axis receives altitude-hold offset.\n" +
                "Open each axis's PID UI once and set its type.\n---\n" +
                "이 탭의 축 타입 지정. 축간 기능 (피치 고도유지) 에 필요.\n" +
                "Hover 축의 PV = 고도, Pitch 축 SP 에 고도 보정 주입.\n" +
                "튜닝 전 각 축 PID UI 열고 타입 설정.",
                () => _s.AxisKind.ToString(),
                () =>
                {
                    // 순환: Unspecified → Yaw → Roll → Pitch → Hover → Forward → Strafe → Unspecified
                    _s.AxisKind = (AxisType)(((int)_s.AxisKind + 1) % Enum.GetValues(typeof(AxisType)).Length);
                },
                "axistype"
            ));

            seg.AddInterpretter(MakeToggle(
                "Fix other axes / 다른 축 고정",
                "During tuning, other axes' SetPoints are frozen at captured values\n" +
                "so existing PIDs hold attitude/altitude. Open each axis's PID UI\n" +
                "once before tuning to register it.\n" +
                "If Hover + Pitch axes both tagged, Pitch SP receives altitude-hold\n" +
                "offset via Hover's PV (for airplane-style altitude control).\n---\n" +
                "튜닝 중 다른 축 SP를 캡처 값에 고정 → 기존 PID가 자세/고도 유지.\n" +
                "튜닝 전 각 축 PID UI를 한 번씩 열어 등록 필요.\n" +
                "Hover + Pitch 모두 태그되면 Hover PV로 피치 SP에 고도 보정 주입.",
                () => _s.FixOtherAxes,
                b => _s.FixOtherAxes = b,
                "fixaxes"
            ));

            ScreenSegmentTable excTable = base.CreateTableSegment(1, 5);
            excTable.SqueezeTable = false;

            excTable.AddInterpretter(MakeSliderFloat(
                "Amplitude A / 자극 진폭 A",
                "Excitation amplitude. Auto-tuning sets this automatically.\n---\n자극 진폭. 자동 튜닝 시 자동 설정됩니다.",
                () => _s.ExciteAmp,
                f => _s.ExciteAmp = Clamp(f, 0f, 10f),
                0f, 10f, 0.05f, "0.00", "A"
            ));

            excTable.AddInterpretter(MakeSliderFloat(
                "Freq base Hz / 기저 주파수",
                "Base frequency for Sine/MultiSine excitation.\n---\nSine/MultiSine 가진의 기저 주파수.",
                () => _s.ExciteFreqHz,
                f => _s.ExciteFreqHz = Clamp(f, 0.01f, 5.0f),
                0.01f, 5.0f, 0.01f, "0.00", "fBase"
            ));

            excTable.AddInterpretter(MakeSliderFloat(
                "Freq max Hz / 최대 주파수",
                "End frequency for Chirp excitation.\n---\nChirp 가진의 최대 주파수.",
                () => _s.ChirpEndHz,
                f => _s.ChirpEndHz = Clamp(f, 0.1f, 10.0f),
                0.1f, 10.0f, 0.1f, "0.0", "fMax"
            ));
        }

        private void BuildActionButtons()
        {
            ScreenSegmentStandardHorizontal seg = base.CreateStandardHorizontalSegment();
            seg.BackgroundStyleWhereApplicable = ConsoleStyles.Instance.Styles.Segments.OptionalSegmentDarkBackgroundWithHeader.Style;
            seg.NameWhereApplicable = "Actions / 동작";
            seg.SpaceAbove = 10f;
            seg.SpaceBelow = 10f;


            seg.AddInterpretter(new SubjectiveButton<VariableControllerMaster>(
                this._focus,
                M.m<VariableControllerMaster>(_ => _autoState == AutoTuneState.Recording ? "Auto-tuning... / 자동 튜닝 중..." : "Auto Tune / 자동 튜닝"),
                M.m<VariableControllerMaster>(new ToolTip(
                    "Closed-loop auto-tuning: excitation → record → FRIT → PID.\n---\n폐루프 자동 튜닝: 가진 → 녹화 → FRIT → PID.", 260f)),
                null,
                _ => AutoTuneNow()
            ));

            seg.AddInterpretter(MakeButton(
                "Record start/stop / 녹화 시작/중지",
                "Start/stop sample collection.\nDuring recording, u (output) and y (process variable) are saved every FixedUpdate.\n---\n샘플 수집을 시작/중지합니다.\n" +
                "녹화 중에는 FixedUpdate마다 u(출력), y(과정변수) 샘플을 저장합니다.",
                _ =>
                {
                    if (_sess.Recording) StopRecording();
                    else StartRecording();
                }
            ));

            seg.AddInterpretter(MakeButton(
                "Reset / 초기화",
                "Clear all saved samples and results.\n---\n저장된 샘플/결과를 모두 지웁니다.",
                _ =>
                {
                    RestoreSetPointAdjustIfNeeded();
                    _sess.Clear();
                    _autoState = AutoTuneState.Idle;
                    _sess.LastMessage = "Reset complete / 초기화 완료";
                }
            ));

            seg.AddInterpretter(MakeButton(
                "Compute (FRIT) / 계산(FRIT)",
                "Compute FRIT: minimize ||y - M·r̃(θ)||² over (Kp,Ti,Td) via Levenberg-Marquardt.\n" +
                "Seeds from current PID values.\n---\n" +
                "FRIT 계산: 현재 PID를 시드로 (Kp,Ti,Td)를 LM 으로 비선형 최적화.\n" +
                "비용: ||y - M·r̃(θ)||² (r̃ = y + u/C(θ))",
                _ => ComputeNow()
            ));

            seg.AddInterpretter(MakeButton(
                "Apply / 적용",
                "Apply Kp/Ti/Td to PID. (Kp: 0.001, Ti/Td: 0.1 step)\n---\nKp/Ti/Td를 PID에 적용. (Kp: 0.001, Ti/Td: 0.1 단위)",
                _ => ApplyToPid()
            ));

            seg.AddInterpretter(MakeButton(
                "Validate / 검증",
                "Health check: record y on all registered axes for 5 seconds (no excitation),\n" +
                "compute std(y) per axis, flag any with yStd > 2× median as HIGH.\n---\n" +
                "검증: 전 축 y를 5초간 수집 (가진 없음), 축별 std(y) 계산,\n" +
                "중앙값의 2배 초과 시 HIGH 경고.",
                _ => ValidateAxes()
            ));
        }

        private void BuildResult()
        {
            ScreenSegmentStandard seg = base.CreateStandardSegment(InsertPosition.OnCursor);
            seg.BackgroundStyleWhereApplicable = ConsoleStyles.Instance.Styles.Segments.OptionalSegmentDarkBackgroundWithHeader.Style;
            seg.NameWhereApplicable = "Result / 결과";
            seg.SpaceAbove = 10f;
            seg.SpaceBelow = 10f;

            seg.AddInterpretter(new SubjectiveDisplay<VariableControllerMaster>(
                this._focus,
                M.m<VariableControllerMaster>(_ =>
                {
                    if (!_sess.HasResult)
                        return "No result yet. Press Compute. / 아직 결과가 없습니다.";

                    // 표준오차 표시 (CRLB, NaN 이면 생략)
                    string fmtSE(double v, double se, string vFmt) {
                        if (double.IsNaN(se) || double.IsInfinity(se) || v == 0) return v.ToString(vFmt);
                        double pct = 100.0 * se / Math.Abs(v);
                        return v.ToString(vFmt) + $"  ±{se.ToString(vFmt)}  ({pct:0}%)";
                    }
                    string result =
                        $"── Single PID ──\n" +
                        $"Kp = {fmtSE(_sess.Kp, _sess.KpSE, "0.0000")}\n" +
                        $"Ti = {fmtSE(_sess.Ti, _sess.TiSE, "0.00")} s\n" +
                        $"Td = {fmtSE(_sess.Td, _sess.TdSE, "0.0000")} s\n" +
                        $"Fit (RMSE) = {_sess.FitRmse:0.0000}";

                    // ── PI(외부) × PD(내부) 분해 ──
                    // Ti_o + Td_i = Ti,  Ti_o * Td_i = Ti * Td
                    // 판별식: Ti² - 4*Ti*Td >= 0 이어야 실수 해 존재
                    double Ti = _sess.Ti;
                    double Td = _sess.Td;
                    double Kp = _sess.Kp;
                    double disc = Ti * Ti - 4.0 * Ti * Td;

                    if (Kp > 1e-6 && Ti > 0.1 && Td > 1e-6 && disc >= 0)
                    {
                        double sqrtDisc = Math.Sqrt(disc);
                        double Ti_o = (Ti + sqrtDisc) / 2.0;  // 외부 (느린 쪽)
                        double Td_i = (Ti - sqrtDisc) / 2.0;  // 내부 (빠른 쪽)

                        if (Ti_o > 1e-6 && Td_i > 1e-6)
                        {
                            double alpha = Ti_o / Td_i;        // 대역폭 비율
                            double product = Kp * Ti_o / Ti;   // Kp_o * Kp_i
                            double sqrtAlpha = Math.Sqrt(alpha);
                            double Kp_o = Math.Sqrt(product / sqrtAlpha);
                            double Kp_i = product / Kp_o;

                            result += $"\n\n── Dual PID (PI×PD, α={alpha:0.0}) ──\n" +
                                      $"Outer PI:  Kp={Kp_o:0.000},  Ti={Ti_o:0.0} s\n" +
                                      $"Inner PD:  Kp={Kp_i:0.000},  Td={Td_i:0.00} s";
                        }
                    }
                    else if (Kp > 1e-6 && Td > 1e-6)
                    {
                        result += $"\n\n── Dual PID: decomposition not possible (Ti < 4·Td required) ──";
                    }

                    return result;
                }),
                M.m<VariableControllerMaster>(new ToolTip(
                    "PID parameters estimated by FRIT (Fictitious Reference Iterative Tuning).\n" +
                    "RMSE = ||y - M·r̃(θ)||² residual after LM convergence. Lower is better.\n" +
                    "Dual PID: PI(outer)×PD(inner) decomposition equivalent to single PID.\n" +
                    "α = inner/outer bandwidth ratio. Larger means inner is faster.\n---\n" +
                    "FRIT (가상 레퍼런스 반복 튜닝) 으로 추정된 PID 파라미터입니다.\n" +
                    "RMSE 는 LM 수렴 후 ||y - M·r̃(θ)|| 잔차 크기입니다. 작을수록 좋습니다.\n\n" +
                    "이중 PID: 단일 PID와 동치인 PI(외부)×PD(내부) 분해입니다.\n" +
                    "α = 내부/외부 대역폭 비율. 클수록 내부가 외부보다 빠릅니다.\n" +
                    "중간에 속도 클램프를 넣으면 캐스케이드 제어에 사용 가능합니다.",
                    260f
                ))
            ));
        }

        // ============================================================
        // Open-Loop Tune
        // ============================================================


        // ============================================================
        // Recording control
        // ============================================================

        private void StartRecording()
        {
            _sess.Clear();
            _sess.Recording = true;
            _sess.LastMessage = "Recording started / 녹화 시작";

            CaptureSetPointAdjustBase();
            CaptureOtherAxesFixture();
        }

        private void StopRecording()
        {
            _sess.Recording = false;
            RestoreSetPointAdjustIfNeeded();
            ReleaseOtherAxesFixture();

            if (_autoState == AutoTuneState.Recording)
                _autoState = AutoTuneState.Idle;
            _sess.LastMessage = "Recording stopped / 녹화 중지";
        }

        // ============================================================
        // Axis Fixture — 다른 축 SP 고정 (기존 PID 가 alt/attitude 유지하게)
        // ============================================================

        /// <summary>
        /// 튜닝 시작 시 호출. 현재 등록된 모든 다른 축의 SP 를 캡처.
        /// Recording 중 매 틱 ApplyOtherAxesFixture() 가 이 값으로 재적용.
        /// 동시에 피치 고도 유지용 Hover/Pitch 축 식별 + 시작 고도 캡처.
        /// </summary>
        private void CaptureOtherAxesFixture()
        {
            _frozenOtherSPs.Clear();
            _altitudeSourceAxis = null;
            _pitchTargetAxis = null;
            _altHoldActive = false;

            if (!_s.FixOtherAxes) return;

            // 리플렉션으로 형제 축 자동 발견 시도 (1회만)
            DiscoverSiblingAxes();

            // 등록된 축 중에서 Hover / Pitch 식별 (AxisKind 로)
            foreach (var kv in _tabsByAxis)
            {
                VariableControllerMaster axis = kv.Key;
                if (axis == null) continue;
                try
                {
                    var axisTab = kv.Value;
                    if (axisTab != null && axisTab._s != null)
                    {
                        if (axisTab._s.AxisKind == AxisType.Hover) _altitudeSourceAxis = axis;
                        else if (axisTab._s.AxisKind == AxisType.Pitch) _pitchTargetAxis = axis;
                    }
                }
                catch { }

                // 다른 축 SP 고정 (현재 축 제외)
                // FakeSetPoint = 현재 PV → FakeSetPointInUse = true → AI SP 덮어쓰기
                // 이전: SetPointAdjust freeze → AI가 자체 SP 계속 업데이트라 효과 없음.
                // 개선: FakeSetPoint 로 AI SP를 현재 PV 값으로 고정.
                if (axis == this._focus) continue;
                try
                {
                    var ctrl = axis.GetCurrentController();
                    if (ctrl != null)
                    {
                        float currentPV = ctrl.LastProcessVariable;
                        // 기존 FakeSetPoint 상태 백업 (복원용)
                        _frozenOtherSPs[axis] = (bool)axis.FakeSetPointInUse ? (float)axis.FakeSetPoint : float.NaN;
                        axis.FakeSetPoint.Us = currentPV;
                        axis.FakeSetPointInUse.Us = true;
                    }
                }
                catch { }
            }

            // 고도 유지 활성 조건: Hover 축 + Pitch 축 모두 등록됨 + Hover PV 읽기 성공
            if (_altitudeSourceAxis != null && _pitchTargetAxis != null)
            {
                try
                {
                    var hoverCtrl = _altitudeSourceAxis.GetCurrentController();
                    if (hoverCtrl != null)
                    {
                        _altHoldStartAltitude = hoverCtrl.LastProcessVariable;
                        _altHoldActive = true;
                    }
                }
                catch { }
            }
        }

        /// <summary>
        /// 매 틱 호출.
        ///   1. 다른 축 (피치 제외) SP 를 freeze 값으로 재적용
        ///   2. 피치 축: 고도 유지 offset 계산 후 SP 주입
        ///      - 피치가 튜닝 대상이면 excitation + offset
        ///      - 피치가 대상 아니면 offset 만
        /// </summary>
        private void ApplyOtherAxesFixture()
        {
            // FakeSetPoint 방식: Capture 에서 이미 FakeSetPointInUse=true 설정.
            // FakeSetPoint 는 FTD 내부에서 유지되므로 매 틱 재적용 불필요.
            // 피치 고도 유지만 매 틱 업데이트 필요.

            // 피치 고도 유지 — Hover PV 로 고도 오차 → 피치 FakeSetPoint 실시간 조정
            if (!_altHoldActive || _pitchTargetAxis == null || _altitudeSourceAxis == null) return;
            try
            {
                var hoverCtrl = _altitudeSourceAxis.GetCurrentController();
                if (hoverCtrl == null) return;
                double currentAlt = hoverCtrl.LastProcessVariable;
                double altErr = _altHoldStartAltitude - currentAlt;
                double clampMax = _s.PitchAltHoldClamp;
                double offset = Math.Max(-clampMax, Math.Min(clampMax, _s.PitchAltHoldGain * altErr));

                if (_pitchTargetAxis == this._focus)
                {
                    // 피치가 튜닝 대상: excitation 위에 offset 더함 (SetPointAdjust 로)
                    _pitchTargetAxis.SetPointAdjust.Us = _baseSetPointAdjust + (float)_lastExciteValue + (float)offset;
                }
                else
                {
                    // 피치 고정 (다른 축 튜닝 중): FakeSetPoint 에 고도 보정 offset 반영
                    var pitchCtrl = _pitchTargetAxis.GetCurrentController();
                    if (pitchCtrl != null)
                    {
                        float basePitch = pitchCtrl.LastProcessVariable;
                        _pitchTargetAxis.FakeSetPoint.Us = basePitch + (float)offset;
                    }
                }
            }
            catch { }
        }

        /// <summary>튜닝 종료 시. FakeSetPoint 해제 — 다른 축 SP 가 다시 AI 에 의해 제어됨.</summary>
        private void ReleaseOtherAxesFixture()
        {
            // FakeSetPointInUse 를 원복 (이전 상태로)
            foreach (var kv in _frozenOtherSPs)
            {
                try
                {
                    if (float.IsNaN(kv.Value))
                    {
                        // 이전에 FakeSetPoint 미사용 → 해제
                        kv.Key.FakeSetPointInUse.Us = false;
                    }
                    else
                    {
                        // 이전에 FakeSetPoint 사용 중이었음 → 원래 값 복원
                        kv.Key.FakeSetPoint.Us = kv.Value;
                    }
                }
                catch { }
            }
            _frozenOtherSPs.Clear();
            _altHoldActive = false;
            _altitudeSourceAxis = null;
            _pitchTargetAxis = null;
        }

        // ============================================================
        // 축 자동 발견 (리플렉션) — _focus 의 부모에서 형제 VCM 열거
        // ============================================================

        /// <summary>
        /// _focus 로부터 부모 객체를 리플렉션으로 탐색, 형제 VariableControllerMaster 발견.
        /// 성공하면 _tabsByAxis 에 자동 등록 (tab=null — SP 접근만 가능, UI 설정 없음).
        /// 실패하면 기존 수동 등록 방식으로 폴백.
        /// </summary>
        private void DiscoverSiblingAxes()
        {
            if (_axisDiscoveryAttempted) return;
            _axisDiscoveryAttempted = true;
            if (_focus == null) return;

            try
            {
                var siblings = FindSiblingControllers(_focus);
                int added = 0;
                foreach (var vcm in siblings)
                {
                    if (vcm == null || vcm == _focus) continue;
                    if (!_tabsByAxis.ContainsKey(vcm))
                    {
                        _tabsByAxis[vcm] = null;  // tab 없음 (자동발견), SP 접근만 가능
                        added++;
                    }
                }
                if (added > 0)
                    _sess.LastMessage = $"Auto-discovered {added} sibling axes / {added}개 형제 축 자동 발견";
            }
            catch { }
        }

        /// <summary>
        /// VCM 의 부모 체인을 리플렉션으로 탐색하여 형제 VCM 목록 반환.
        /// 전략: (1) 필드에서 부모 찾기 (2) 부모의 필드/프로퍼티에서 VCM 컬렉션 찾기.
        /// </summary>
        private static List<VariableControllerMaster> FindSiblingControllers(VariableControllerMaster focus)
        {
            var result = new List<VariableControllerMaster>();
            Type vcmType = focus.GetType();
            var allFields = new List<System.Reflection.FieldInfo>();
            var allProps = new List<System.Reflection.PropertyInfo>();
            const System.Reflection.BindingFlags bf =
                System.Reflection.BindingFlags.Instance |
                System.Reflection.BindingFlags.NonPublic |
                System.Reflection.BindingFlags.Public;

            // ── 전략 1: focus 자신의 필드에서 부모 객체 탐색 ──
            for (Type t = vcmType; t != null && t != typeof(object); t = t.BaseType)
            {
                allFields.AddRange(t.GetFields(bf | System.Reflection.BindingFlags.DeclaredOnly));
                allProps.AddRange(t.GetProperties(bf | System.Reflection.BindingFlags.DeclaredOnly));
            }

            // 부모 후보: 필드/프로퍼티 중 VCM 이 아니고, null 아닌 참조 타입
            foreach (var field in allFields)
            {
                if (field.FieldType.IsValueType) continue;
                if (field.FieldType == typeof(string)) continue;
                if (typeof(VariableControllerMaster).IsAssignableFrom(field.FieldType)) continue;

                object parent = null;
                try { parent = field.GetValue(focus); } catch { continue; }
                if (parent == null) continue;

                // ── 전략 2: 부모에서 VCM 컬렉션 탐색 ──
                var found = ExtractVcmsFromObject(parent);
                if (found.Count >= 2) // 최소 2개 (self + 형제)
                {
                    result.AddRange(found);
                    return result; // 첫 성공한 경로 사용
                }
            }

            // 프로퍼티도 시도
            foreach (var prop in allProps)
            {
                if (!prop.CanRead) continue;
                if (prop.PropertyType.IsValueType || prop.PropertyType == typeof(string)) continue;

                object parent = null;
                try { parent = prop.GetValue(focus); } catch { continue; }
                if (parent == null) continue;

                var found = ExtractVcmsFromObject(parent);
                if (found.Count >= 2)
                {
                    result.AddRange(found);
                    return result;
                }
            }

            return result;
        }

        /// <summary>객체의 필드/프로퍼티에서 VCM 인스턴스를 모두 추출.</summary>
        private static List<VariableControllerMaster> ExtractVcmsFromObject(object obj)
        {
            var result = new List<VariableControllerMaster>();
            if (obj == null) return result;
            Type objType = obj.GetType();
            const System.Reflection.BindingFlags bf =
                System.Reflection.BindingFlags.Instance |
                System.Reflection.BindingFlags.NonPublic |
                System.Reflection.BindingFlags.Public;

            // 직접 VCM 필드
            foreach (var f in objType.GetFields(bf))
            {
                try
                {
                    if (typeof(VariableControllerMaster).IsAssignableFrom(f.FieldType))
                    {
                        var vcm = f.GetValue(obj) as VariableControllerMaster;
                        if (vcm != null) result.Add(vcm);
                    }
                    // VCM 배열
                    else if (f.FieldType.IsArray &&
                             typeof(VariableControllerMaster).IsAssignableFrom(f.FieldType.GetElementType()))
                    {
                        var arr = f.GetValue(obj) as Array;
                        if (arr != null)
                            foreach (var item in arr)
                            {
                                var vcm = item as VariableControllerMaster;
                                if (vcm != null) result.Add(vcm);
                            }
                    }
                    // IEnumerable<VCM> (List, etc.)
                    else if (typeof(System.Collections.IEnumerable).IsAssignableFrom(f.FieldType)
                             && !f.FieldType.IsValueType && f.FieldType != typeof(string))
                    {
                        var enumerable = f.GetValue(obj) as System.Collections.IEnumerable;
                        if (enumerable != null)
                            foreach (var item in enumerable)
                            {
                                var vcm = item as VariableControllerMaster;
                                if (vcm != null) result.Add(vcm);
                            }
                    }
                }
                catch { }
            }

            // 프로퍼티도
            foreach (var p in objType.GetProperties(bf))
            {
                if (!p.CanRead) continue;
                try
                {
                    if (typeof(VariableControllerMaster).IsAssignableFrom(p.PropertyType))
                    {
                        var vcm = p.GetValue(obj) as VariableControllerMaster;
                        if (vcm != null) result.Add(vcm);
                    }
                    else if (typeof(System.Collections.IEnumerable).IsAssignableFrom(p.PropertyType)
                             && !p.PropertyType.IsValueType && p.PropertyType != typeof(string))
                    {
                        var enumerable = p.GetValue(obj) as System.Collections.IEnumerable;
                        if (enumerable != null)
                            foreach (var item in enumerable)
                            {
                                var vcm = item as VariableControllerMaster;
                                if (vcm != null) result.Add(vcm);
                            }
                    }
                }
                catch { }
            }

            return result;
        }

        // ============================================================
        // Auto Tune (FRIT)
        // ============================================================

        /// <summary>
        /// [자동 튜닝] 버튼 클릭 시 호출.
        /// 가진 설정을 자동으로 잡고 → 녹화 시작.
        /// 이후 OnUiFixed가 매 틱마다 데이터를 모으다가 MinSamples 도달하면 자동으로 계산.
        /// </summary>
        private void AutoTuneNow()
        {
            if (_autoState != AutoTuneState.Idle && _autoState != AutoTuneState.Done && _autoState != AutoTuneState.Failed)
            {
                _sess.LastMessage = "Tuning already in progress / 튜닝 이미 진행 중";
                return;
            }

            RestoreSetPointAdjustIfNeeded();

            // 바로 녹화 시작 — 현재 PID C₀ 로 폐루프 데이터 수집 (FRIT seed 로도 사용)
            StartAutoTuneRecording();
        }

        /// <summary>
        /// 포화 해소 완료 후 실제 녹화를 시작하는 메서드.
        /// </summary>
        private void StartAutoTuneRecording()
        {
            double dt = Time.fixedDeltaTime;
            if (dt <= 0) dt = 0.02;
            double fs = 1.0 / dt;

            // 자연 변동 측정 → 시작 진폭 결정
            double naturalStd = 0;
            if (_naturalYCount >= 10)
            {
                double sum = 0;
                double sqSum = 0;
                int n = Math.Min(_naturalYCount, NaturalBufSize);
                for (int i = 0; i < n; i++)
                    sum += _naturalYBuf[i];
                double mean = sum / n;
                for (int i = 0; i < n; i++)
                {
                    double d = _naturalYBuf[i] - mean;
                    sqSum += d * d;
                }
                naturalStd = Math.Sqrt(sqSum / Math.Max(1, n - 1));
            }
            _sess.NaturalYStd = naturalStd;

            // 시작 진폭: 자연 변동의 3배 이상, 최소 0.3
            double startAmp = Math.Max(0.3, naturalStd * 3.0);
            startAmp = Math.Min(startAmp, _s.AdaptiveAmpMax);

            // SP 가진: SetPointAdjust에 멀티사인 추가, 원본 PID는 그대로 동작
            _s.ExciteEnabled = true;
            _s.ExciteWave = WaveType.MultiSine;
            _s.ExciteAmp = (float)startAmp;
            _s.ExciteFreqHz = 0.05f;
            _s.ChirpEndHz = (float)Math.Min(fs / 4.0, 2.0);
            _s.AdaptiveAmp = true;

            _autoState = AutoTuneState.Recording;
            StartRecording();
            _sess.AdaptiveCurrentAmp = _s.ExciteAmp;
            _sess.LastMessage = $"Recording (SP excite, amp={startAmp:0.00}) / 녹화 중 (SP 가진)";
        }

        /// <summary>
        /// 녹화 완료 후 다음 틱에 호출. 자동 튜닝 계산 파이프라인:
        /// 1) 최장 연속 블록 선택 (포화 구멍 제외) + step prelude 제외
        /// 2) 데이터 품질 체크 (포화율, y 변화량)
        /// 3) Ts 자동 스캔 (10단계) + FRIT (LM 비선형 최적화)
        /// 4) 파라미터 안정성 기반 best Ts 선택
        /// </summary>
        private void AutoTuneCompute()
        {
            double dt = Time.fixedDeltaTime;
            if (dt <= 0) dt = 0.02;
            double fs = 1.0 / dt;

            // 전체 수집 데이터 사용 (블록 분리 제거). 포화 + transient tail 은 FRIT 가중치에서 down-weight.
            int blkLen = _sess.U.Count;
            if (blkLen < 64)
            {
                _autoState = AutoTuneState.Failed;
                _sess.LastMessage = $"Auto-tune failed: only {blkLen} samples collected. / 자동 튜닝 실패: 수집 샘플 {blkLen}개.";
                return;
            }

            double[] u = _sess.U.ToArray();
            double[] y = _sess.Y.ToArray();
            bool[]   sat = _sess.Saturated.ToArray();

            double yStd = StdDev(y);
            if (yStd < 1e-6)
            {
                _autoState = AutoTuneState.Failed;
                _sess.LastMessage = "Auto-tune failed: no change in y. Check PID connection. / 자동 튜닝 실패: y 변화 없음.";
                return;
            }

            // 디버그: u, y 범위 기록
            double uMin = u[0], uMax = u[0];
            double yMin = y[0], yMax = y[0];
            for (int i = 1; i < blkLen; i++)
            {
                if (u[i] < uMin) uMin = u[i];
                if (u[i] > uMax) uMax = u[i];
                if (y[i] < yMin) yMin = y[i];
                if (y[i] > yMax) yMax = y[i];
            }
            _sess.LastMessage = $"Data: N={blkLen} valid={_sess.EffectiveValidCount} sat={_sess.SaturatedCount} u=[{uMin:0.00},{uMax:0.00}] y=[{yMin:0.0},{yMax:0.0}] yStd={yStd:0.000}";

            // τ = dt 고정 (FTD 순수 지연 ≈ 1틱), nM=2 (FTD 제어 대상 대부분 2차)
            _s.ModelDelayTau = (float)dt;
            _s.CutoffHz = (float)(fs / 8.0);
            _s.ModelOrderNm = 2;

            // 현재 PID 값을 LM 초기 시드로 사용
            double kp0 = this._focus.Pid.kP.Us;
            double ti0 = this._focus.Pid.kI.Us;
            double td0 = this._focus.Pid.kD.Us;

            // ── Ts 자동 스캔 (10단계, 로그 간격 0.1 ~ 1.0초) ──
            // FRIT 는 LM 비선형 최적화 → VRFT(선형 LS) 대비 비싸므로 40→10단계 축소.
            // 인접 Ts 간 파라미터 변화율이 작은 가장 작은 Ts 선택 (안정 영역).
            const int tsSteps = 10;
            double[] tsArr = new double[tsSteps + 1];
            FritResult[] results = new FritResult[tsSteps + 1];
            bool[] validArr = new bool[tsSteps + 1];

            for (int si = 0; si <= tsSteps; si++)
            {
                tsArr[si] = 0.1 * Math.Pow(10.0, (double)si / tsSteps); // 0.1 ~ 1.0
                _s.SettlingTimeTs = (float)tsArr[si];
                try
                {
                    results[si] = ComputeFritPid(u, y, sat, dt, _s, kp0, ti0, td0);
                    validArr[si] = results[si].Kp > 0 && !double.IsNaN(results[si].Rmse);
                }
                catch { validArr[si] = false; }
            }

            // 인접 Ts 간 max(|ΔKp/Kp|, |ΔTi/Ti|, |ΔTd/Td|) 가 임계값 이하인 가장 작은 Ts
            const double stabilityThreshold = 0.3;
            FritResult bestResult = default;
            double bestTs = 1.0;
            bool anyFound = false;
            for (int si = 0; si < tsSteps; si++)
            {
                if (!validArr[si] || !validArr[si + 1]) continue;

                double dKp = Math.Abs(results[si + 1].Kp - results[si].Kp) / Math.Max(Math.Abs(results[si].Kp), 1e-6);
                double dTi = Math.Abs(results[si + 1].Ti - results[si].Ti) / Math.Max(Math.Abs(results[si].Ti), 1e-6);
                double dTd = Math.Abs(results[si + 1].Td - results[si].Td) / Math.Max(Math.Abs(results[si].Td), 1e-6);
                double maxChange = Math.Max(dKp, Math.Max(dTi, dTd));

                if (maxChange < stabilityThreshold)
                {
                    bestResult = results[si];
                    bestTs = tsArr[si];
                    anyFound = true;
                    break;
                }
            }

            // 안정 영역 못 찾으면 RMSE 최소 Ts fallback
            if (!anyFound)
            {
                double bestRmse = double.MaxValue;
                for (int si = 0; si <= tsSteps; si++)
                {
                    if (!validArr[si]) continue;
                    if (results[si].Rmse < bestRmse)
                    {
                        bestRmse = results[si].Rmse;
                        bestResult = results[si];
                        bestTs = tsArr[si];
                        anyFound = true;
                    }
                }
            }

            if (!anyFound)
            {
                _autoState = AutoTuneState.Failed;
                _sess.LastMessage = "Auto-tune failed: FRIT did not converge for any Ts / 모든 Ts 에서 FRIT 수렴 실패";
                return;
            }

            _s.SettlingTimeTs = (float)bestTs;
            FritResult best = bestResult;

            _sess.HasResult = true;
            _sess.Kp = best.Kp; _sess.Ti = best.Ti; _sess.Td = best.Td;
            _sess.KpSE = best.KpSE; _sess.TiSE = best.TiSE; _sess.TdSE = best.TdSE;
            _sess.FitRmse = best.Rmse;

            _autoState = AutoTuneState.Done;
            string conv = best.Converged ? "converged" : "max-iter";
            string warn = string.IsNullOrEmpty(best.Warning) ? "" : " [⚠ " + best.Warning + "]";
            _sess.LastMessage = $"Done | FRIT Ts={bestTs:0.00} ({conv}, {best.Iterations} iter) Kp={best.Kp:0.000} Ti={best.Ti:0.1} Td={best.Td:0.00} rmse={best.Rmse:0.0000}{warn}";
        }

        private void ValidateAxes()
        {
            if (_autoState == AutoTuneState.Validating)
            {
                _sess.LastMessage = "Validation already in progress / 검증 이미 진행 중";
                return;
            }

            // Ensure axis discovery
            DiscoverSiblingAxes();

            if (_tabsByAxis.Count < 2)
            {
                _sess.LastMessage = "Need >= 2 registered axes for validation. Open each axis PID UI first. / 검증에 2개 이상 축 필요. 각 축 PID UI를 먼저 열어주세요.";
                return;
            }

            // Initialize validation buffers — one list per registered axis
            _sess.ValidateY.Clear();
            foreach (var _ in _tabsByAxis) _sess.ValidateY.Add(new List<double>());
            _sess.ValidateStartT = 0;
            _autoState = AutoTuneState.Validating;
            double valDur = GetValidateDuration();
            _sess.LastMessage = $"Validating: collecting y on all axes for {valDur:0}s... / 검증: 전 축 y 수집 중 ({valDur:0}초)...";
        }

        private void OnValidateTick(double dt)
        {
            if (_autoState != AutoTuneState.Validating) return;

            _sess.ValidateStartT += dt;

            // Collect y from each registered axis
            int idx = 0;
            foreach (var kv in _tabsByAxis)
            {
                if (idx >= _sess.ValidateY.Count) break;
                var ctrl = kv.Key.GetCurrentController();
                double yVal = ctrl != null ? ctrl.LastProcessVariable : 0;
                _sess.ValidateY[idx].Add(yVal);
                idx++;
            }

            // 축별 검증 시간: 느린 축(Yaw/Hover/Forward) 15초, 빠른 축 5초
            // Yaw 0.05Hz = 주기 20초 → 5초로는 1/4 주기만 관측. drift/oscillation 구분 불가.
            double valDuration = GetValidateDuration();
            if (_sess.ValidateStartT >= valDuration)
            {
                var stdValues = new List<double>();
                var axisNames = new List<string>();

                idx = 0;
                foreach (var kv in _tabsByAxis)
                {
                    string axisLabel;
                    if (kv.Value != null && kv.Value._s != null)
                        axisLabel = kv.Value._s.AxisKind.ToString();
                    else
                        axisLabel = $"Axis {idx + 1}";

                    if (idx < _sess.ValidateY.Count && _sess.ValidateY[idx].Count > 1)
                    {
                        var ys = _sess.ValidateY[idx];
                        double mean = 0;
                        for (int i = 0; i < ys.Count; i++) mean += ys[i];
                        mean /= ys.Count;
                        double variance = 0;
                        for (int i = 0; i < ys.Count; i++) variance += (ys[i] - mean) * (ys[i] - mean);
                        variance /= (ys.Count - 1);
                        double std = Math.Sqrt(variance);
                        stdValues.Add(std);
                        axisNames.Add(axisLabel);
                    }
                    else
                    {
                        stdValues.Add(0);
                        axisNames.Add(axisLabel);
                    }
                    idx++;
                }

                // Compute median
                double median = 0;
                if (stdValues.Count > 0)
                {
                    var sorted = new List<double>(stdValues);
                    sorted.Sort();
                    median = sorted[sorted.Count / 2];
                }

                // Build result message — 상대 기준 (중간값 2배) + 절대 기준 (std > 0.3) 이중 검사
                // 상대만 쓰면 전축 나쁠 때 못 잡음. 절대 기준이 최후 안전망.
                const double ABS_STD_THRESHOLD = 0.3;
                var parts = new List<string>();
                for (int i = 0; i < stdValues.Count; i++)
                {
                    bool relativeHigh = median > 1e-9 && stdValues[i] > 2.0 * median;
                    bool absoluteHigh = stdValues[i] > ABS_STD_THRESHOLD;
                    string flag = relativeHigh ? " (HIGH)" : absoluteHigh ? " (HIGH-ABS)" : "";
                    parts.Add($"{axisNames[i]}: yStd={stdValues[i]:0.000}{flag}");
                }

                _autoState = AutoTuneState.Done;
                _sess.LastMessage = "Validate: " + string.Join(", ", parts);
            }
        }

        /// <summary>축 타입에 따른 검증 수집 시간. 느린 축은 최저 주파수 한 주기 이상 필요.</summary>
        private double GetValidateDuration()
        {
            // 느린 축: Yaw(0.05Hz=20s주기), Hover, Forward, Strafe → 15초
            // 빠른 축: Pitch, Roll → 5초
            switch (_s.AxisKind)
            {
                case AxisType.Yaw:
                case AxisType.Hover:
                case AxisType.Forward:
                case AxisType.Strafe:
                    return 15.0;
                default:
                    return 5.0;
            }
        }

        private void CaptureSetPointAdjustBase()
        {
            try
            {
                _baseSetPointAdjust = this._focus.SetPointAdjust.Us;
                _hasBaseSetPointAdjust = true;
            }
            catch
            {
                _hasBaseSetPointAdjust = false;
                _sess.LastMessage = "SetPointAdjust access failed. Excitation may not work. / SetPointAdjust 접근 실패.";
            }
        }

        private void RestoreSetPointAdjustIfNeeded()
        {
            if (!_hasBaseSetPointAdjust) return;
            try { this._focus.SetPointAdjust.Us = _baseSetPointAdjust; }
            catch { }
        }

        /// <summary>
        /// 매 틱마다 SetPoint에 가진 신호를 더함.
        /// SP = 원래SP + x(t) 형태로, PID가 이 변화에 반응하게 만들어서
        /// 플랜트의 동특성 정보를 u/y 데이터에 담기 위한 것.
        ///
        /// 가진이 왜 필요한가?
        /// - PID가 안정적으로 잘 작동하면 u/y가 거의 일정 → 플랜트 정보 없음
        /// - 외부에서 SP를 흔들어야 PID가 반응하고, 그 반응에서 플랜트 특성이 드러남
        /// </summary>
        /// <summary>현재 틱 가진값 (피치 고도유지 offset 계산에 사용). 비활성/조건 미충족 시 0.</summary>
        private double _lastExciteValue = 0.0;

        private void ApplyExcitation(float dt)
        {
            _lastExciteValue = 0.0;
            if (!_s.ExciteEnabled) return;        // 가진 꺼져있으면 무시
            if (_s.ExciteWave == WaveType.Off) return;
            if (!_hasBaseSetPointAdjust) return;   // 원래 SP를 백업 못 했으면 무시

            double t = _sess.T;  // 녹화 시작부터의 경과 시간 (블록 분리 제거됨)
            double amp = Math.Max(0.0, _s.ExciteAmp); // 진폭 (음수 방지)

            // 포화 회피: |u|가 포화 임계값에 가까우면 가진 진폭을 줄임
            double absU = Math.Abs(_sess.LastU);
            double satMargin = _s.SaturationThreshold - absU; // 포화까지 남은 여유
            if (satMargin < 0.3 && satMargin > 0.0)
            {
                // 여유가 0.3~0 → 스케일 1.0~0.1
                double scale = Math.Max(0.1, satMargin / 0.3);
                amp *= scale;
            }
            else if (satMargin <= 0.0)
            {
                amp *= 0.1; // 이미 포화 근처면 최소 가진
            }

            double x = 0.0;

            // ── Step prelude: 최초 0.5초 동안 일정 SP offset 유지 (DC 정보 주입) ──
            // 멀티사인은 DC 성분 없음 → FRIT 의 DC 동작 매칭이 외삽 영역이 됨.
            // 0.5초 step 은 DC 방향 Fisher information 보강 → 세 방법 모두 DC 분산 감소.
            const double STEP_PRELUDE_SEC = 0.5;
            if (_s.ExciteWave == WaveType.MultiSine && _sess.T < STEP_PRELUDE_SEC)
            {
                x = amp; // 일정 양의 offset
                _lastExciteValue = x;
                try { this._focus.SetPointAdjust.Us = _baseSetPointAdjust + (float)x; } catch { }
                return;
            }

            switch (_s.ExciteWave)
            {
                case WaveType.Sine:
                    {
                        double w = 2.0 * Math.PI * Math.Max(0.01, _s.ExciteFreqHz);
                        x = amp * Math.Sin(w * t);
                        break;
                    }
                case WaveType.Chirp:
                    {
                        // 로그 chirp - 저주파에서 오래 머물러 Ti 추정에 유리
                        // 순시 주파수: f(t) = f0 * (f1/f0)^(t/T)
                        // 위상: φ(t) = 2π * f0*T/ln(f1/f0) * ((f1/f0)^(t/T) - 1)
                        double f0 = Math.Max(0.01, _s.ChirpStartHz);
                        double f1 = Math.Max(f0 * 1.1, _s.ChirpEndHz);
                        double T = Math.Max(1.0, _s.MinSamples * dt);
                        double ratio = f1 / f0;
                        double lnRatio = Math.Log(ratio);
                        double phase;
                        if (t <= T)
                        {
                            phase = 2.0 * Math.PI * f0 * T / lnRatio * (Math.Pow(ratio, t / T) - 1.0);
                        }
                        else
                        {
                            double phaseAtT = 2.0 * Math.PI * f0 * T / lnRatio * (ratio - 1.0);
                            phase = phaseAtT + 2.0 * Math.PI * f1 * (t - T);
                        }
                        x = amp * Math.Sin(phase);
                        break;
                    }
                case WaveType.MultiSine:
                    {
                        // 12성분 멀티사인 (P/D 모드) + 저주파 square wave (I 모드 DC 정보)
                        // 진폭 예산 분할: 멀티사인 (1-r) : square r,  r = SquareAmpRatio
                        double fBase = Math.Max(0.01, _s.ExciteFreqHz);
                        double fMax = Math.Max(fBase * 2.0, _s.ChirpEndHz);
                        int nComp = 12;

                        double sqRatio = Math.Max(0.0, Math.Min(0.8, _s.SquareAmpRatio));
                        double msAmp = amp * (1.0 - sqRatio);
                        double sqAmp = amp * sqRatio;

                        // 슈뢰더 위상 멀티사인 — P/D 식별
                        double compAmp = msAmp / Math.Sqrt(nComp);
                        for (int ci = 0; ci < nComp; ci++)
                        {
                            double fi = fBase * Math.Pow(fMax / fBase, (double)ci / (nComp - 1));
                            double phi = -Math.PI * ci * (ci + 1) / nComp;
                            x += compAmp * Math.Sin(2.0 * Math.PI * fi * t + phi);
                        }

                        // 저주파 square wave — 각 half-period 동안 SP 일정 → 적분기 sustained 오차 → Ti 식별 강화
                        // 평균 0 이라 자세 bias 없음. f_sq 주기보다 짧은 chatter 쿨다운 (3초) 과 자연스럽게 어우러짐.
                        if (sqAmp > 0)
                        {
                            double sqFreq = Math.Max(0.01, _s.SquareFreqHz);
                            double sinPhase = Math.Sin(2.0 * Math.PI * sqFreq * t);
                            x += sqAmp * (sinPhase >= 0 ? 1.0 : -1.0);
                        }
                        break;
                    }
            }

            _lastExciteValue = x;
            try
            {
                this._focus.SetPointAdjust.Us = _baseSetPointAdjust + (float)x;
            }
            catch { }
        }

        // ============================================================
        // Compute / Apply
        // ============================================================

        private struct FritResult
        {
            public double Kp, Ti, Td, Rmse;
            public double KpSE, TiSE, TdSE;     // Cramér-Rao 표준오차 (95% CI ≈ ±2·SE)
            public string Warning;
            public int Iterations;
            public int IrlsIterations;          // IRLS 반복 횟수
            public bool Converged;
        }

        private void ComputeNow()
        {
            try
            {
                double dt = Time.fixedDeltaTime;
                if (dt <= 0) dt = 0.02;

                int blkLen = _sess.U.Count;
                if (_sess.EffectiveValidCount < _s.MinSamples / 4)
                {
                    _sess.LastMessage = $"Insufficient valid samples: {_sess.EffectiveValidCount}. Collect more / 유효 샘플 부족: {_sess.EffectiveValidCount}";
                    return;
                }

                double[] u = _sess.U.ToArray();
                double[] y = _sess.Y.ToArray();
                bool[]   sat = _sess.Saturated.ToArray();

                // 현재 PID 값을 LM 초기 시드로 사용
                double kp0 = this._focus.Pid.kP.Us;
                double ti0 = this._focus.Pid.kI.Us;
                double td0 = this._focus.Pid.kD.Us;

                FritResult r = ComputeFritPid(u, y, sat, dt, _s, kp0, ti0, td0);

                _sess.HasResult = true;
                _sess.Kp = r.Kp;
                _sess.Ti = r.Ti;
                _sess.Td = r.Td;
                _sess.KpSE = r.KpSE; _sess.TiSE = r.TiSE; _sess.TdSE = r.TdSE;
                _sess.FitRmse = r.Rmse;

                string conv = r.Converged ? "converged" : "max-iter";
                string body = $"FRIT {conv} ({r.Iterations} iter, rmse={r.Rmse:0.0000})";
                _sess.LastMessage = string.IsNullOrEmpty(r.Warning)
                    ? "Computation complete / 계산 완료: " + body
                    : "Computation complete (warning: " + r.Warning + ") / 계산 완료: " + body;
            }
            catch (Exception e)
            {
                _sess.LastMessage = "Computation failed / 계산 실패: " + e.Message;
            }
        }

        private void ApplyToPid()
        {
            try
            {
                if (!_sess.HasResult)
                {
                    _sess.LastMessage = "No result. Compute first. / 결과가 없습니다.";
                    return;
                }

                // 최소 단위 반영
                double kp = RoundToStep(Math.Max(0.001, _sess.Kp), 0.001);
                double ti = RoundToStep(Math.Max(0.0, _sess.Ti), 0.1);
                double td = RoundToStep(Math.Max(0.0, _sess.Td), 0.01);

                // 게임 UI 관례 (kI가 250이면 off처럼 쓰는 경우가 많음)
                if (ti > 250.0) ti = 250.0;
                if (td > 10.0) td = 10.0;

                this._focus.Pid.kP.Us = (float)kp;
                this._focus.Pid.kI.Us = (float)ti; // Ti
                this._focus.Pid.kD.Us = (float)td; // Td

                _sess.LastMessage = $"Applied: Kp={kp:0.000}, Ti={ti:0.0}, Td={td:0.00}";
            }
            catch (Exception e)
            {
                _sess.LastMessage = "Apply failed / 적용 실패: " + e.Message;
            }
        }

        /// <summary>
        /// ★ FRIT (Fictitious Reference Iterative Tuning) 핵심 계산 — 시간 영역 ★
        ///
        /// 입력: u[](제어 출력), y[](프로세스 변수), sat[](포화 플래그), dt, s, (Kp0,Ti0,Td0) 초기값
        /// 출력: FritResult {Kp, Ti, Td, Rmse, Warning, Iterations, Converged}
        ///
        /// 비용:
        ///   r̃(θ)[k] = y[k] + (1/C(θ)) · u[k]    가상 레퍼런스 (시간 영역 IIR 역필터)
        ///   ŷ(θ)[k] = M(z) · r̃(θ)[k-delay]      이산화된 참조 모델 (Tustin)
        ///   J(θ)    = Σ w[k] · (y[k] - ŷ(θ)[k])²    가중 LS  (포화 인덱스: w=ε)
        ///   → Levenberg-Marquardt 로 (Kp, Ti, Td) 최적화.
        ///
        /// 주파수 영역 대비 장점:
        ///   1. FFT 없음 → LM 반복당 비용 ↓
        ///   2. spectral contamination (포화로 인한 비선형 leakage) 없음 → per-sample 가중치 100% 유효
        ///   3. circular convolution wrap-around 없음 (causal)
        ///   4. 순수 지연을 정수 틱으로 정확히 처리
        ///
        /// 가중치 구현: sqrt-스케일링 트릭
        ///   ||√w·y - √w·ŷ||² = Σ w·(y-ŷ)²
        ///   → observedY = √w·y, model 출력에 √w 곱해서 LM 에 던지면 가중 LS 와 등가.
        ///
        /// 안정성: 1/C(z) 의 pole 은 C(z) 분자의 zero. 단위원 밖이면 역필터 발산 → soft barrier.
        /// </summary>
        private static FritResult ComputeFritPid(double[] u, double[] y, bool[] sat, double dt, Settings s,
                                                  double Kp0, double Ti0, double Td0)
        {
            int N = Math.Min(u.Length, y.Length);
            if (sat == null || sat.Length != N) throw new Exception("u/y/sat length mismatch / 길이 불일치");
            if (N < 64) throw new Exception("Too few samples / 샘플이 너무 적습니다.");

            // ── 1단계: 디트렌드 (DC + 선형 추세 제거) ──
            double[] ud = new double[N]; Array.Copy(u, ud, N); Detrend(ud);
            double[] yd = new double[N]; Array.Copy(y, yd, N); Detrend(yd);

            // ── 2단계: 참조 모델 M(z) 이산화 (Tustin, θ 와 무관 → LM 외부에서 한 번만) ──
            // M(s) = exp(-s·τM) / (1 + s·aM)^nM,  aM = 0.2·Ts
            // 1차 LP H₁(z) = (1 + z⁻¹) / (β₀ + β₁ z⁻¹)
            //   β₀ = 1 + 2aM/dt,  β₁ = 1 - 2aM/dt
            // nM 번 캐스케이드. 순수 지연은 정수 틱 shift 로 처리.
            double ts = Math.Max(0.05, s.SettlingTimeTs);
            int nM = ClampInt(s.ModelOrderNm, 1, 10);
            double tauM = Math.Max(0.0, s.ModelDelayTau);
            double aM = 0.2 * ts;
            double beta0 = 1.0 + 2.0 * aM / dt;
            double beta1 = 1.0 - 2.0 * aM / dt;
            int delayN = Math.Max(0, (int)Math.Round(tauM / dt));

            // ── 3단계: 포화 + IIR transient tail 까지 확장한 effective saturation ──
            // 포화 직후 1/C(z) 역필터 state 가 회복하는 데 ~TransientTailSamples 틱 소요.
            // 그 구간 동안 계산되는 e[k] 가 오염 → effSat 로 down-weight.
            int tail = Math.Max(0, s.TransientTailSamples);
            bool[] effSat = new bool[N];
            int since = int.MaxValue / 2;   // 시작 부분은 깨끗하다고 가정
            for (int k = 0; k < N; k++)
            {
                if (sat[k]) { effSat[k] = true; since = 0; }
                else { since++; if (since <= tail) effSat[k] = true; }
            }

            // per-sample 가중치 (effSat: SAT_WEIGHT, 나머지: 1)
            const double SAT_WEIGHT = 1e-3;
            double[] sqrtW = new double[N];
            int nEffValid = 0;
            for (int i = 0; i < N; i++)
            {
                sqrtW[i] = Math.Sqrt(effSat[i] ? SAT_WEIGHT : 1.0);
                if (!effSat[i]) nEffValid++;
            }
            if (nEffValid < 32)
                throw new Exception($"Too few effective valid samples after tail ({nEffValid}) / 유효 샘플 부족");

            // ── 4단계: LM 모델 함수 (θ → √w · ŷ) ──
            Func<MathNet.Numerics.LinearAlgebra.Vector<double>,
                 MathNet.Numerics.LinearAlgebra.Vector<double>,
                 MathNet.Numerics.LinearAlgebra.Vector<double>> model = (theta, xUnused) =>
            {
                double kp = Math.Max(1e-6, theta[0]);
                double ti = Math.Max(1e-3, theta[1]);
                double td = Math.Max(0.0,  theta[2]);

                // PID 이산화 (backward Euler for I and D):
                //   C(z) = Kp · (a₀ + a₁z⁻¹ + a₂z⁻²) / (1 - z⁻¹)
                //     a₀ = 1 + dt/Ti + Td/dt
                //     a₁ = -(1 + 2·Td/dt)
                //     a₂ = Td/dt
                double a0 = 1.0 + dt/ti + td/dt;
                double a1 = -(1.0 + 2.0*td/dt);
                double a2 = td/dt;

                // 1/C(z) 안정성: C 분자 polynomial (a₀ z² + a₁ z + a₂) 의 zero 가 단위원 내부?
                double disc = a1*a1 - 4.0*a0*a2;
                bool stable;
                if (disc >= 0)
                {
                    double sqrtD = Math.Sqrt(disc);
                    double z1 = (-a1 + sqrtD) / (2.0*a0);
                    double z2 = (-a1 - sqrtD) / (2.0*a0);
                    stable = (Math.Abs(z1) < 0.9999 && Math.Abs(z2) < 0.9999);
                }
                else
                {
                    // 복소 conjugate pair: |z|² = a₂/a₀
                    stable = (a2/a0 < 0.9999);
                }

                var result = VB.Dense(N);
                if (!stable)
                {
                    // soft barrier: LM 이 unstable region 으로 가면 큰 residual 로 후퇴 유도
                    for (int i = 0; i < N; i++) result[i] = 1e6 * sqrtW[i];
                    return result;
                }

                // ── (a) e = (1/C) · u  ─ 시간 영역 IIR 역필터 ──
                //   (1 - z⁻¹) · u[k] = Kp · (a₀ + a₁z⁻¹ + a₂z⁻²) · e[k]
                //   e[k] = (u[k] - u[k-1] - Kp·a₁·e[k-1] - Kp·a₂·e[k-2]) / (Kp·a₀)
                double[] e = new double[N];
                for (int k = 0; k < N; k++)
                {
                    double uPrev = (k > 0) ? ud[k-1] : 0.0;
                    double e1 = (k > 0) ? e[k-1] : 0.0;
                    double e2 = (k > 1) ? e[k-2] : 0.0;
                    e[k] = ((ud[k] - uPrev) - kp*a1*e1 - kp*a2*e2) / (kp*a0);
                }

                // ── (b) 가상 레퍼런스 r̃ = y + e ──
                double[] rt = new double[N];
                for (int k = 0; k < N; k++) rt[k] = yd[k] + e[k];

                // ── (c) 순수 지연: r̃_d[k] = r̃[k - delayN] ──
                double[] rtd = new double[N];
                for (int k = 0; k < N; k++) rtd[k] = (k >= delayN) ? rt[k - delayN] : 0.0;

                // ── (d) M(z) 캐스케이드: 1차 LP 를 nM 번 적용 ──
                //   y[k] = (x[k] + x[k-1] - β₁·y[k-1]) / β₀
                double[] cur = rtd;
                for (int stage = 0; stage < nM; stage++)
                {
                    double[] next = new double[N];
                    for (int k = 0; k < N; k++)
                    {
                        double xPrev = (k > 0) ? cur[k-1] : 0.0;
                        double yPrev = (k > 0) ? next[k-1] : 0.0;
                        next[k] = (cur[k] + xPrev - beta1*yPrev) / beta0;
                    }
                    cur = next;
                }

                // ── (e) √w 스케일 + NaN/Inf 검사 ──
                for (int i = 0; i < N; i++)
                {
                    double v = sqrtW[i] * cur[i];
                    if (double.IsNaN(v) || double.IsInfinity(v))
                    {
                        // 수치 발산 → soft barrier
                        for (int j = 0; j < N; j++) result[j] = 1e6 * sqrtW[j];
                        return result;
                    }
                    result[i] = v;
                }
                return result;
            };

            // ── 5단계: 관측값 + 초기값 sanity ──
            var obsX = VB.Dense(N, i => (double)i);                       // dummy

            if (Kp0 <= 1e-6 || double.IsNaN(Kp0) || Kp0 > 1.0) Kp0 = 0.1;
            if (Ti0 <= 0.1 || Ti0 >= 250.0 || double.IsNaN(Ti0)) Ti0 = Math.Max(0.5, ts * 2.0);
            if (Td0 < 0 || Td0 > 10.0 || double.IsNaN(Td0)) Td0 = 0.05;
            var initial = VB.DenseOfArray(new[] { Kp0, Ti0, Td0 });

            // ── 6단계: IRLS 외부 루프 + LM 내부 (robust M-estimator, Huber) ──
            //   매 iter:  LM 으로 가중 LS 풀고 → 잔차 보고 → Huber 가중치 업데이트 → 다음 iter
            //   effSat 인덱스의 saturation 가중치 (=1e-3) 는 유지, 나머지에 Huber 가중치 곱함.
            //   3 회 반복이면 보통 robust 가중치 수렴.
            const int IRLS_MAX_ITER = 3;
            const double HUBER_K = 1.5;        // δ = K · σ_robust
            MathNet.Numerics.Optimization.NonlinearMinimizationResult lmResult = null;
            int irlsIter = 0;

            for (int irls = 0; irls < IRLS_MAX_ITER; irls++)
            {
                var obsY = VB.Dense(N, i => sqrtW[i] * yd[i]);
                var objective = MathNet.Numerics.Optimization.ObjectiveFunction.NonlinearModel(model, obsX, obsY);
                var lm = new MathNet.Numerics.Optimization.LevenbergMarquardtMinimizer(maximumIterations: 30);

                try
                {
                    lmResult = lm.FindMinimum(objective, initial);
                }
                catch (Exception ex)
                {
                    return new FritResult
                    {
                        Kp = Kp0, Ti = Ti0, Td = Td0,
                        Rmse = double.NaN,
                        Warning = "LM failed / LM 실패: " + ex.Message,
                        Iterations = 0,
                        IrlsIterations = irls,
                        Converged = false
                    };
                }
                initial = lmResult.MinimizingPoint;
                irlsIter = irls + 1;

                // 마지막 iter 에서는 가중치 업데이트 불필요
                if (irls >= IRLS_MAX_ITER - 1) break;

                // 잔차 계산 (raw, unweight)
                var ypredW = model(lmResult.MinimizingPoint, obsX);
                double[] resAbs = new double[N];
                var unsatResList = new List<double>();
                for (int i = 0; i < N; i++)
                {
                    double pred = ypredW[i] / Math.Max(1e-12, sqrtW[i]);
                    double r = yd[i] - pred;
                    resAbs[i] = Math.Abs(r);
                    if (!effSat[i]) unsatResList.Add(resAbs[i]);
                }
                if (unsatResList.Count < 8) break;

                // MAD 기반 robust scale 추정
                unsatResList.Sort();
                double mad = unsatResList[unsatResList.Count / 2];
                double sigmaR = 1.4826 * Math.Max(mad, 1e-6);
                double delta = HUBER_K * sigmaR;

                // Huber 가중치 업데이트 (effSat 는 saturation 가중치 유지)
                for (int i = 0; i < N; i++)
                {
                    if (effSat[i]) continue;     // saturation weight 보존
                    double absR = resAbs[i];
                    double wH = (absR <= delta) ? 1.0 : (delta / absR);
                    sqrtW[i] = Math.Sqrt(wH);
                }
            }

            double Kp = lmResult.MinimizingPoint[0];
            double Ti = lmResult.MinimizingPoint[1];
            double Td = lmResult.MinimizingPoint[2];

            // ── 7단계: RMSE (effSat 제외, robust 가중치도 무시한 raw 잔차) ──
            var finalWeighted = model(lmResult.MinimizingPoint, obsX);
            double sse = 0;
            int nValidRmse = 0;
            for (int i = 0; i < N; i++)
            {
                if (effSat[i]) continue;
                double pred = finalWeighted[i] / Math.Max(1e-12, sqrtW[i]);
                double err = yd[i] - pred;
                sse += err * err;
                nValidRmse++;
            }
            double rmse = (nValidRmse > 0) ? Math.Sqrt(sse / nValidRmse) : double.NaN;

            // ── 8단계: CRLB (Cramér-Rao 표준오차) ──
            //   J = ∂(√w·ŷ)/∂θ at θ*, σ² ≈ ssr_w / (N_eff - 3)
            //   cov ≈ σ² · (Jᵀ J)⁻¹ → diag 가 분산, sqrt 가 표준오차
            double kpSE = double.NaN, tiSE = double.NaN, tdSE = double.NaN;
            try
            {
                double[] eps = { Math.Max(1e-6, Math.Abs(Kp) * 1e-4),
                                 Math.Max(1e-4, Math.Abs(Ti) * 1e-4),
                                 Math.Max(1e-7, Math.Abs(Td) * 1e-4) };
                double[,] Jmat = new double[N, 3];
                for (int j = 0; j < 3; j++)
                {
                    var thp = lmResult.MinimizingPoint.Clone();
                    var thm = lmResult.MinimizingPoint.Clone();
                    thp[j] += eps[j]; thm[j] -= eps[j];
                    var yp = model(thp, obsX);
                    var ym = model(thm, obsX);
                    double invDen = 0.5 / eps[j];
                    for (int i = 0; i < N; i++) Jmat[i, j] = (yp[i] - ym[i]) * invDen;
                }
                // JtJ
                double[,] JtJ = new double[3, 3];
                for (int a = 0; a < 3; a++)
                    for (int b = 0; b < 3; b++)
                    {
                        double sumJJ = 0;
                        for (int i = 0; i < N; i++) sumJJ += Jmat[i, a] * Jmat[i, b];
                        JtJ[a, b] = sumJJ;
                    }
                double[,] inv = Invert3x3(JtJ);
                // σ² from weighted residuals over effective valid samples
                int nEff = 0;
                double ssrW = 0;
                for (int i = 0; i < N; i++)
                {
                    if (effSat[i]) continue;
                    if (sqrtW[i] < 0.5) continue;   // IRLS-downweighted outlier 도 제외
                    double rW = sqrtW[i] * yd[i] - finalWeighted[i];
                    ssrW += rW * rW;
                    nEff++;
                }
                if (nEff > 3 && inv != null)
                {
                    double sigma2 = ssrW / (nEff - 3);
                    kpSE = Math.Sqrt(Math.Max(0, sigma2 * inv[0, 0]));
                    tiSE = Math.Sqrt(Math.Max(0, sigma2 * inv[1, 1]));
                    tdSE = Math.Sqrt(Math.Max(0, sigma2 * inv[2, 2]));
                }
            }
            catch { /* CRLB 실패해도 본 결과는 유효 */ }

            // ── 9단계: 경계 클램프 + 경고 ──
            string warning = null;
            if (Kp < 0) { warning = $"Kp<0 ({Kp:0.000}) clamped to 0"; Kp = 0; }
            if (Ti < 0.1) Ti = 0.1;
            if (Ti > 250.0) Ti = 250.0;
            if (Td < 0) Td = 0;
            if (Td > 10.0) Td = 10.0;

            if (double.IsNaN(Kp) || double.IsInfinity(Kp)) { Kp = 0; warning = "Kp NaN/Inf"; }
            if (double.IsNaN(Ti) || double.IsInfinity(Ti)) Ti = 250.0;
            if (double.IsNaN(Td) || double.IsInfinity(Td)) Td = 0;

            return new FritResult
            {
                Kp = Kp, Ti = Ti, Td = Td,
                KpSE = kpSE, TiSE = tiSE, TdSE = tdSE,
                Rmse = rmse,
                Warning = warning,
                Iterations = lmResult.Iterations,
                IrlsIterations = irlsIter,
                Converged = (lmResult.ReasonForExit == MathNet.Numerics.Optimization.ExitCondition.Converged)
            };
        }

        /// <summary>3×3 matrix inversion via cofactor expansion. Returns null if singular.</summary>
        private static double[,] Invert3x3(double[,] m)
        {
            double a = m[0, 0], b = m[0, 1], c = m[0, 2];
            double d = m[1, 0], e = m[1, 1], f = m[1, 2];
            double g = m[2, 0], h = m[2, 1], i = m[2, 2];
            double det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);
            if (Math.Abs(det) < 1e-30) return null;
            double inv = 1.0 / det;
            double[,] r = new double[3, 3];
            r[0, 0] = (e*i - f*h) * inv;
            r[0, 1] = (c*h - b*i) * inv;
            r[0, 2] = (b*f - c*e) * inv;
            r[1, 0] = (f*g - d*i) * inv;
            r[1, 1] = (a*i - c*g) * inv;
            r[1, 2] = (c*d - a*f) * inv;
            r[2, 0] = (d*h - e*g) * inv;
            r[2, 1] = (b*g - a*h) * inv;
            r[2, 2] = (a*e - b*d) * inv;
            return r;
        }

        // ============================================================
        // UI helpers (FTD 패턴: new SubjectiveFloatClampedWithBar + M.m)
        // ============================================================

        private SubjectiveButton<VariableControllerMaster> MakeButton(string label, string tip, Action<VariableControllerMaster> onClick)
        {
            return new SubjectiveButton<VariableControllerMaster>(
                this._focus,
                M.m<VariableControllerMaster>(_ => label),
                M.m<VariableControllerMaster>(new ToolTip(tip, 260f)),
                null,
                onClick
            );
        }

        private SubjectiveToggle<VariableControllerMaster> MakeToggle(string label, string tip, Func<bool> getter, Action<bool> setter, string tag = null)
        {
            return new SubjectiveToggle<VariableControllerMaster>(
                this._focus,
                M.m<VariableControllerMaster>(_ => label),
                M.m<VariableControllerMaster>(new ToolTip(tip, 260f)),
                (VariableControllerMaster _, bool b) => setter(b),
                null,
                (VariableControllerMaster _) => getter(),
                tag == null ? Array.Empty<string>() : new[] { tag }
            );
        }

        private SubjectiveButton<VariableControllerMaster> MakeCycleButton(string title, string tip, Func<string> valueText, Action onClick, string tag = null)
        {
            return new SubjectiveButton<VariableControllerMaster>(
                this._focus,
                M.m<VariableControllerMaster>(_ => $"{title}: {valueText()} (click/클릭)"),
                M.m<VariableControllerMaster>(new ToolTip(tip, 260f)),
                null,
                _ => onClick()
            );
        }

        private SubjectiveFloatClampedWithBar<VariableControllerMaster> MakeSliderFloat(
            string titleKo,
            string tipKo,
            Func<float> getter,
            Action<float> setter,
            float min,
            float max,
            float step,
            string format,
            string tag = null)
        {
            return new SubjectiveFloatClampedWithBar<VariableControllerMaster>(
                M.m<VariableControllerMaster>(_ => min),
                M.m<VariableControllerMaster>(_ => max),
                M.m<VariableControllerMaster>(_ => getter()),
                M.m<VariableControllerMaster>(_ => step),
                this._focus,
                M.m<VariableControllerMaster>(_ => $"{titleKo}: {getter().ToString(format)}"),
                (VariableControllerMaster _, float f) => setter(f),
                (VariableControllerMaster _, float f) => $"Set to {f.ToString(format)} / {titleKo} → {f.ToString(format)}",
                M.m<VariableControllerMaster>(new ToolTip(tipKo, 260f)),
                tag == null ? Array.Empty<string>() : new[] { tag }
            );
        }

        private SubjectiveFloatClampedWithBar<VariableControllerMaster> MakeSliderInt(
            string titleKo,
            string tipKo,
            Func<int> getter,
            Action<int> setter,
            int min,
            int max,
            int step,
            string format,
            string tag = null)
        {
            return new SubjectiveFloatClampedWithBar<VariableControllerMaster>(
                M.m<VariableControllerMaster>(_ => min),
                M.m<VariableControllerMaster>(_ => max),
                M.m<VariableControllerMaster>(_ => getter()),
                M.m<VariableControllerMaster>(_ => step),
                this._focus,
                M.m<VariableControllerMaster>(_ => $"{titleKo}: {getter().ToString(format)}"),
                (VariableControllerMaster _, float f) => setter((int)Math.Round(f)),
                (VariableControllerMaster _, float f) => $"Set to {(int)Math.Round(f)} / {titleKo} → {(int)Math.Round(f)}",
                M.m<VariableControllerMaster>(new ToolTip(tipKo, 260f)),
                tag == null ? Array.Empty<string>() : new[] { tag }
            );
        }

        // ============================================================
        // small utils
        // ============================================================

        private static string WaveToKo(WaveType w)
        {
            switch (w)
            {
                case WaveType.Off: return "Off";
                case WaveType.Sine: return "Sine";
                case WaveType.Chirp: return "Chirp";
                case WaveType.MultiSine: return "MultiSine";
                default: return w.ToString();
            }
        }

        private static float Clamp(float v, float lo, float hi)
        {
            if (v < lo) return lo;
            if (v > hi) return hi;
            return v;
        }

        private static int ClampInt(int v, int lo, int hi)
        {
            if (v < lo) return lo;
            if (v > hi) return hi;
            return v;
        }

        private static double RoundToStep(double v, double step)
        {
            if (step <= 0) return v;
            return Math.Round(v / step) * step;
        }

        private static int NextPow2(int n)
        {
            int v = 1;
            while (v < n) v <<= 1;
            return v;
        }

        /// <summary>DC + 선형 추세 제거 (in-place). spectral leakage 저감.</summary>
        private static void Detrend(double[] x)
        {
            int N = x.Length;
            if (N < 2) return;
            double sumT = 0, sumTT = 0, sumX = 0, sumXT = 0;
            for (int i = 0; i < N; i++)
            {
                double t = i;
                sumT += t;
                sumTT += t * t;
                sumX += x[i];
                sumXT += x[i] * t;
            }
            double meanT = sumT / N;
            double meanX = sumX / N;
            double den = sumTT - N * meanT * meanT;
            double slope = Math.Abs(den) < 1e-15 ? 0.0 : (sumXT - N * meanT * meanX) / den;
            double intercept = meanX - slope * meanT;
            for (int i = 0; i < N; i++)
                x[i] -= (slope * i + intercept);
        }

        private static double StdDev(double[] data)
        {
            if (data.Length < 2) return 0.0;
            double mean = 0;
            for (int i = 0; i < data.Length; i++)
                mean += data[i];
            mean /= data.Length;
            double sum = 0;
            for (int i = 0; i < data.Length; i++)
            {
                double d = data[i] - mean;
                sum += d * d;
            }
            return Math.Sqrt(sum / (data.Length - 1));
        }

        /// <summary>
        /// 플랜트 지연 추정: 위상 기울기 기반.
        ///
        /// H(jω) = Y/U의 위상에서 순수 지연을 추출.
        /// 순수 지연 exp(-jωτ)는 위상 = -ωτ (주파수에 비례하는 선형 위상).
        /// 저주파 영역의 위상 기울기 = -τ → τ = -dφ/dω.
        /// 저주파만 사용하여 플랜트 동특성(극점/영점)의 위상과 분리.
        /// </summary>
        private static double EstimateDelay(double[] u, double[] y, double dt)
        {
            int N = Math.Min(u.Length, y.Length);
            if (N < 16) return 0.0;

            double[] ud = new double[N]; Array.Copy(u, ud, N); Detrend(ud);
            double[] yd = new double[N]; Array.Copy(y, yd, N); Detrend(yd);

            int Nfft = NextPow2(2 * N);
            double fs = 1.0 / dt;

            Complex[] Uf = new Complex[Nfft];
            Complex[] Yf = new Complex[Nfft];
            for (int i = 0; i < N; i++)
            {
                Uf[i] = new Complex(ud[i], 0);
                Yf[i] = new Complex(yd[i], 0);
            }

            Fourier.Forward(Uf, FourierOptions.Matlab);
            Fourier.Forward(Yf, FourierOptions.Matlab);

            // H(jω) = Y/U (Wiener 정규화)
            double maxUmag2 = 0;
            for (int k = 0; k < Nfft; k++)
            {
                double m2 = Uf[k].Real * Uf[k].Real + Uf[k].Imaginary * Uf[k].Imaginary;
                if (m2 > maxUmag2) maxUmag2 = m2;
            }
            double reg = maxUmag2 * 1e-4;

            // 저주파 빈에서 위상 수집 (DC 제외, ~fs/8까지) + 위상 언래핑
            int maxBin = Math.Max(2, Nfft / 8);
            double sumWF = 0, sumWP = 0, sumWW = 0, sumW = 0;
            double prevPhase = 0;
            double cumUnwrap = 0;

            for (int k = 1; k <= maxBin; k++)
            {
                double m2 = Uf[k].Real * Uf[k].Real + Uf[k].Imaginary * Uf[k].Imaginary;
                Complex H = (Complex.Conjugate(Uf[k]) * Yf[k]) / (m2 + reg);

                double w = 2.0 * Math.PI * k * fs / Nfft; // 각주파수
                double rawPhase = Math.Atan2(H.Imaginary, H.Real); // (-π, π]

                // 위상 언래핑: 인접 빈 간 점프가 π를 넘으면 2π 보정
                if (k > 1)
                {
                    double diff = rawPhase - prevPhase;
                    if (diff > Math.PI) cumUnwrap -= 2.0 * Math.PI;
                    else if (diff < -Math.PI) cumUnwrap += 2.0 * Math.PI;
                }
                prevPhase = rawPhase;
                double phase = rawPhase + cumUnwrap;

                double weight = m2 / (m2 + reg); // SNR 기반 가중치

                // 가중 선형 회귀: phase ≈ offset + slope * w
                // slope = -τ
                sumWF += weight * w * phase;
                sumWP += weight * phase;
                sumWW += weight * w * w;
                sumW  += weight * w;
            }

            // slope = (Σw·wf - Σw·Σwp/Σ1) / (Σw·ww - (Σw)²/Σ1)
            // 간소화: 가중 최소자승으로 기울기 추출
            double denom = sumWW * sumW - sumW * sumW; // 이건 항상 0이 아님 (w가 다 다르니까)
            // 더 안정적인 형태: 직접 Σw*phase*w / Σw*w*w (절편 무시, DC 제외했으니)
            double slope = sumWW > 1e-12 ? sumWF / sumWW : 0.0;

            // τ = -slope (위상 기울기가 음수면 양의 지연)
            double tau = -slope;
            return Math.Max(0.0, Math.Min(tau, 0.2)); // FTD 환경: 순수 지연 최대 0.2초 (10틱)
        }

        private static double EstimateSettlingTime(double[] y, double dt)
        {
            int N = y.Length;
            if (N < 16) return 2.0;

            // 디트렌드(DC + 선형 추세 제거) → 드리프트에 의한 시정수 과대추정 방지
            double[] yd = new double[N]; Array.Copy(y, yd, N); Detrend(yd);

            int Nfft = NextPow2(2 * N);
            Complex[] Yc = new Complex[Nfft];
            for (int i = 0; i < N; i++)
                Yc[i] = new Complex(yd[i], 0);

            Fourier.Forward(Yc, FourierOptions.Matlab);

            // PSD → 자기상관
            Complex[] AC = new Complex[Nfft];
            for (int k = 0; k < Nfft; k++)
                AC[k] = Yc[k] * Complex.Conjugate(Yc[k]);

            Fourier.Inverse(AC, FourierOptions.Matlab);

            double peak = AC[0].Real;
            if (peak < 1e-12) return 2.0;

            // 자기상관이 피크의 5% 아래로 떨어지는 지점 → 주요 시정수
            double threshold = 0.05 * peak;
            int tauIdx = N / 4; // fallback
            for (int i = 1; i < N / 2; i++)
            {
                if (AC[i].Real < threshold)
                {
                    tauIdx = i;
                    break;
                }
            }

            // 정착시간 ≈ 4 × 시정수
            double settlingTime = 4.0 * tauIdx * dt;
            return Math.Max(0.5, settlingTime);
        }
    }
}
