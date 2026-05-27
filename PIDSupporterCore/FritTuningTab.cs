// ============================================================================
// FritTuningTab.cs — FRIT (Fictitious Reference Iterative Tuning) PID 자동튜닝
// ============================================================================
//
// ┌─────────────────────────────────────────────────────────────────────────┐
// │ 🎯 한 줄 요약                                                            │
// │   폐루프 데이터 (u, y) 만으로 PID 파라미터 (Kp, Ti, Td) 를 자동 추정.     │
// │   플랜트 모델 식별 불필요. 시간 영역 IIR + Levenberg-Marquardt 최적화.    │
// └─────────────────────────────────────────────────────────────────────────┘
//
// ┌─────────────────────────────────────────────────────────────────────────┐
// │ 🔬 핵심 수식                                                             │
// │                                                                          │
// │   가상 레퍼런스:  r̃(θ)[k] = y[k] + C(θ)⁻¹ · u[k]                         │
// │   참조 모델 응답:  ŷ(θ)[k] = M(z) · r̃(θ)[k]                              │
// │   비용 함수:      J(θ) = Σ w[k] · (y[k] - ŷ(θ)[k])²    ← LM 최소화      │
// │                                                                          │
// │   θ = (Kp, Ti, Td),   M(s) = exp(-s·τM) / (1 + s·0.2·Ts)^nM             │
// │                                                                          │
// │   가중치 w[k] = w_sat[k] · w_huber[k]                                    │
// │     - w_sat:   포화 + IIR transient tail 에서 ε ≈ 1e-3                   │
// │     - w_huber: IRLS 갱신, 이상치는 δ/|r| 로 감쇠                          │
// └─────────────────────────────────────────────────────────────────────────┘
//
// ┌─────────────────────────────────────────────────────────────────────────┐
// │ 🌊 파이프라인                                                            │
// │                                                                          │
// │   [Auto Tune] → 멀티사인 + square wave 주입                              │
// │              → 폐루프 (u, y, Saturated) 연속 수집                         │
// │              → EffectiveValidCount ≥ MinSamples (시간 상한 없음)         │
// │              → Ts 자동 스캔 (10단계 0.1~1.0초)                            │
// │              → 각 Ts: IRLS (3 iter) × LM (30 iter) + CRLB                │
// │              → 파라미터 안정성 기반 best Ts 선택                          │
// │              → [Apply] 로 게임 PID 에 기록                                │
// └─────────────────────────────────────────────────────────────────────────┘
//
// ============================================================================
// 📚 메서드 / 타입 색인
// ============================================================================
//
// ── 출처 범례 ──
//   [자체]    이 모드에서 직접 만든 코드
//   [C#]      C# / .NET 기본
//   [FTD]     FTD 게임 DLL (BrilliantSkies.*)
//   [Unity]   Unity 엔진
//   [MathNet] MathNet.Numerics 라이브러리
//
// ── 라이프사이클 / 상태 머신 ──
//   [자체] FritTuningTab(window, focus)        생성자 (FTD SuperScreen 상속)
//   [자체] Build()                             override. UI 요소 배치 (FTD가 호출)
//   [자체] OnUiFixed()                         매 물리 틱 호출. 데이터 수집의 심장
//   [자체] enum AutoTuneState                  Idle/Diagnosing/Recording/Computing/Done/Failed/Validating
//
// ── UI 빌더 (Build() 가 호출) ──
//   [자체] BuildStatus()                       상태 표시 영역
//   [자체] BuildSettingsSliders()              Ts/τ/MinSamples 슬라이더
//   [자체] BuildExcitationControls()           가진 토글/축 타입/기타
//   [자체] BuildActionButtons()                AutoTune/Record/Reset/Compute/Apply/Validate
//   [자체] BuildResult()                       결과 패널 (Kp ± SE, Dual PID 분해)
//   [자체] MakeButton/MakeToggle/MakeCycleButton/MakeSliderFloat/MakeSliderInt
//                                              FTD UI 컴포넌트 헬퍼
//
// ── 자동 튜닝 진입 / 종료 ──
//   [자체] AutoTuneNow()                       [자동 튜닝] 클릭 → Diagnosing 진입
//   [자체] OnDiagnoseTick(dt)                  3초 사전 진단 (가진 OFF, |u| 통계 → 판정)
//   [자체] StartAutoTuneRecording()            진단 통과 시 가진 설정 + StartRecording
//   [자체] StartRecording() / StopRecording()  세션 초기화 / SP 복원
//   [자체] AutoTuneCompute()                   녹화 완료 → Ts 스캔 → FRIT 결정
//
// ── 가진 신호 (ApplyExcitation) ──
//   [자체] ApplyExcitation(dt)                 Multi-width doublet 패턴 (광대역 자극 + 평균 zero)
//   [자체] CaptureSetPointAdjustBase()         원래 SP 백업
//   [자체] RestoreSetPointAdjustIfNeeded()     SP 복원
//   [자체] enum WaveType                       Off/Sine/Chirp/MultiSine
//
// ── 축 분리 (Axis Fixture) ──
//   [자체] CaptureOtherAxesFixture()           튜닝 중 다른 축 SP 고정 + 고도 유지 활성
//   [자체] ApplyOtherAxesFixture()             매 틱 피치 고도 보정 offset 주입
//   [자체] ReleaseOtherAxesFixture()           튜닝 후 FakeSetPoint 해제
//   [자체] DiscoverSiblingAxes()               리플렉션으로 형제 축 자동 발견
//   [자체] FindSiblingControllers/ExtractVcmsFromObject  리플렉션 헬퍼
//   [자체] enum AxisType                       Unspecified/Yaw/Roll/Pitch/Hover/Forward/Strafe
//
// ── 검증 (Validate) ──
//   [자체] ValidateAxes()                      전 축 y 5~15초 수집 시작
//   [자체] OnValidateTick(dt)                  검증 매 틱 + 결과 집계
//   [자체] GetValidateDuration()               축 타입별 검증 시간
//
// ── 결과 적용 ──
//   [자체] ComputeNow()                        [계산] 클릭: 수동 FRIT 실행
//   [자체] ApplyToPid()                        [적용] 클릭: 게임 PID 에 쓰기
//
// ── ★ FRIT 핵심 (이 파일의 알고리즘 본체) ──
//   [자체] ComputeFritPid(u, y, sat, dt, s, kp0, ti0, td0)
//                                              시간 영역 FRIT + IRLS + CRLB
//                                              return FritResult
//   [자체] struct FritResult                    Kp/Ti/Td + SE + Iter + Converged
//   [자체] Invert3x3(matrix)                    CRLB 용 3×3 역행렬 (cofactor)
//
// ── 신호 처리 유틸 ──
//   [자체] Detrend(x)                          DC + 선형 추세 제거 (in-place)
//   [자체] StdDev(data)                        표준편차
//   [자체] NextPow2(n)                         2의 거듭제곱 (FFT 용 — 현재 미사용)
//   [자체] RoundToStep(v, step)                step 단위 반올림
//   [자체] Clamp / ClampInt                    범위 제한
//   [자체] WaveToKo(w)                         WaveType → 한국어
//
// ── 외부 의존성 ──
//   [FTD]     SuperScreen<T> / VariableControllerMaster / IVariableController
//   [FTD]     ConsoleWindow, ScreenSegment*, SubjectiveDisplay/Button/Toggle
//   [FTD]     SetPointAdjust, FakeSetPoint, Pid.kP/kI/kD
//   [Unity]   Time.fixedDeltaTime
//   [MathNet] Optimization.LevenbergMarquardtMinimizer  비선형 LS
//   [MathNet] Optimization.ObjectiveFunction.NonlinearModel  LM 모델 래퍼
//   [MathNet] LinearAlgebra.Vector<double>.Build (VB) / Matrix<double>.Build (MB)
//   [MathNet] IntegralTransforms.Fourier  FFT (현재 미사용)
//   [C#]      System.Numerics.Complex  FFT 시절 잔재
//
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

namespace PIDSupporter
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
            Idle,        // 대기 중
            Diagnosing,  // 사전 진단: 가진 OFF 로 3초 관찰 → 현재 PID 상태 판정
            Recording,   // 데이터 수집 중 (폐루프, 가진 ON)
            Computing,   // 수집 끝, FRIT 계산 중
            Done,        // 계산 완료 (결과 있음)
            Failed,      // 실패 (에러 메시지 있음)
            Validating,  // 검증 모드: 전 축 y 수집 중 (5초)
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
            // 포화 이후 IIR 회복 transient 보호 구간 (샘플 수).
            // 0 = 보호 없음 — 포화 샘플만 down-weight, 회복 transient 는 IRLS Huber 가 처리.
            //   짧은 spike 는 영향 미미, 긴 포화는 진단 단계가 미리 차단.
            //   대부분 데이터를 살림 → CRLB 정밀도 ↑.
            // 100 = 보수적 (≈2초) — 포화 이후 모든 샘플 down-weight. 데이터 손실 큼.
            public int   TransientTailSamples = 0;

            // ===== 가진(Excitation): 플랜트를 흔들어서 데이터를 만드는 신호 =====
            public bool ExciteEnabled = true;       // 가진 켤지
            public WaveType ExciteWave = WaveType.Sine; // 가진 파형 종류
            public float ExciteAmp = 0.5f;          // 가진 진폭 (SetPoint에 더해지는 크기)
            public float ExciteFreqHz = 0.6f;       // Sine/MultiSine 기본 주파수 (Hz)
            public float ChirpStartHz = 0.2f;       // Chirp 시작 주파수
            public float ChirpEndHz = 2.0f;         // Chirp 끝 주파수

            // ===== 적응형 진폭: PID가 가진을 다 눌러버릴 때 자동으로 키움 =====
            // SP-direct 라 SP 단위 (u 의 [-1, 1] 범위와 무관). 큰 amp 허용해 강한 PID 의
            // closed-loop reject 극복 가능. 액추에이터 saturation 은 별도 메커니즘이 감지.
            public bool  AdaptiveAmp = true;
            public float AdaptiveAmpMax = 10.0f;

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

            // 사전 진단 상태 (Diagnosing 단계, 3초 가진 OFF 관찰)
            public double DiagT;            // 진단 누적 시간 (초)
            public int    DiagSampleCount;
            public double DiagUMax, DiagUMin;
            public int    DiagSatCount;     // |u| ≥ 임계 카운트
            public int    DiagSignChanges;  // u 부호 변환 횟수
            public double DiagPrevU;        // 직전 u (부호 변환 검출용)

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
                DiagT = 0;
                DiagSampleCount = 0;
                DiagUMax = DiagUMin = 0;
                DiagSatCount = 0;
                DiagSignChanges = 0;
                DiagPrevU = 0;
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

        // ════════════════════════════════════════════════════════════════════════
        // OnUiFixed — 매 물리 틱 (50Hz) 호출되는 "심장박동"
        // ════════════════════════════════════════════════════════════════════════
        //
        // Harmony 패치 (VariableControllerUiFixedUpdatePatch) 가 FTD 의
        // FixedUpdateWhenActive 직후에 이 메서드를 호출.
        //
        // ─────────────────────────────────────────────────────────────────────
        // 흐름 (Recording 상태 기준)
        // ─────────────────────────────────────────────────────────────────────
        //
        //   1) State 분기:
        //      Computing  → AutoTuneCompute() 한 번 실행 후 return
        //      Validating → OnValidateTick() 호출 후 return
        //      !Recording → 자연 변동 모음 (idle 노이즈 floor) 후 return
        //      Recording  → 아래 본 흐름 진행
        //
        //   2) ApplyExcitation(dt):
        //      SetPoint 에 멀티사인 + square wave 주입.
        //      |u| 가 포화 임계 근처면 가진 진폭 자동 축소.
        //
        //   3) ApplyOtherAxesFixture():
        //      다른 축 SP 고정 + 피치 고도 유지 offset.
        //
        //   4) u, y 읽기:
        //      c.LastControlVariable / c.LastProcessVariable
        //
        //   5) 포화 추적:
        //      saturated = |u| ≥ SaturationThreshold
        //      SamplesSinceLastSat 카운터 갱신 (포화 시 0, 그 외 ++)
        //
        //   6) 적응형 진폭 (saturation 기반 binary):
        //      윈도우 (60샘플 ≈ 1.2초) 통계 누적.
        //      쿨다운 (3초) 지나면:
        //         satRate > 2% OR uPeak > 0.85  →  amp ÷ 1.5
        //         그 외                          →  amp × 1.5
        //
        //   7) 데이터 저장:
        //      U.Add(u), Y.Add(y), Saturated.Add(saturated)
        //      SamplesSinceLastSat > TransientTailSamples 면 EffectiveValidCount++
        //
        //   8) 종료 판정:
        //      EffectiveValidCount ≥ MinSamples  →  Computing 전환
        //      (시간 상한 없음 — 적응형 진폭이 비포화로 수렴할 때까지 계속 수집)
        //
        // ─────────────────────────────────────────────────────────────────────
        // 핵심 컨셉: "수집 vs 활용" 분리
        // ─────────────────────────────────────────────────────────────────────
        //   - 수집: 모든 샘플 저장 (블록 분리 / 드롭 없음)
        //   - 활용: ComputeFritPid 에서 effSat (포화 + transient tail) 가중치로 분리
        //   - 적응형 amp 가 saturation boundary 주위로 자연 수렴 → 정보 최대화
        // ════════════════════════════════════════════════════════════════════════
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

                // 사전 진단 모드 (Auto Tune 직후 3초): 가진 OFF, u 통계만 수집
                if (_autoState == AutoTuneState.Diagnosing)
                {
                    double dtDg = Time.fixedDeltaTime;
                    if (dtDg <= 0) dtDg = 0.02;
                    OnDiagnoseTick(dtDg);
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

                // 자동 튜닝 종료 조건: 비포화 유효 샘플이 MinSamples 에 도달하면 종료.
                // 시간 상한 없음 — 포화율이 높아도 적응형 진폭이 결국 가진을 줄여 비포화로 수렴.
                if (_autoState == AutoTuneState.Recording
                    && _sess.EffectiveValidCount >= _s.MinSamples)
                {
                    StopRecording();
                    _autoState = AutoTuneState.Computing;
                    _sess.LastMessage = "Auto-tune: analyzing... / 자동 튜닝: 데이터 분석 중...";
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
            seg.NameWhereApplicable = "Status";
            seg.SpaceAbove = 10f;
            seg.SpaceBelow = 10f;

            seg.AddInterpretter(new SubjectiveDisplay<VariableControllerMaster>(
                this._focus,
                M.m<VariableControllerMaster>(_ =>
                {
                    string rec;
                    if (_autoState == AutoTuneState.Diagnosing)
                        rec = "Diagnosing";
                    else if (_autoState == AutoTuneState.Validating)
                        rec = "Validating";
                    else if (_autoState == AutoTuneState.Computing)
                        rec = "Computing";
                    else if (_sess.Recording)
                        rec = "Recording";
                    else if (_autoState == AutoTuneState.Done)
                        rec = "Done";
                    else if (_autoState == AutoTuneState.Failed)
                        rec = "Failed";
                    else
                        rec = "Idle";
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
            table.NameWhereApplicable = "FRIT Settings";
            table.SpaceAbove = 10f;
            table.SpaceBelow = 10f;
            table.SqueezeTable = false;

            // dt 를 슬라이더 그리드 단위로 사용 (FTD 50Hz 기준 ~0.02s)
            float dtF = (float)Time.fixedDeltaTime;
            if (dtF <= 0f) dtF = 0.02f;

            // t_s : dt 단위 (1·dt ~ 5.0s). 5.0 상한은 무거운 함선까지 커버.
            // 내부 ComputeFritPid 가 추가로 2.5·dt 하한을 적용해 LP 안정성 보장.
            table.AddInterpretter(MakeSliderFloat(
                "Settling time t_s (s)",
                "Target settling time. Smaller = faster response.\nGrid is dt (FTD tick); min 1·dt, max 5s.\nAuto-tuning estimates this automatically.\n---\n목표 정착시간. 작을수록 빠른 응답.\n그리드 단위는 dt(FTD 틱); 최소 1·dt, 최대 5초.\n자동 튜닝 시 자동 추정됩니다.",
                () => _s.SettlingTimeTs,
                f => _s.SettlingTimeTs = Clamp(f, dtF, 5.0f),
                dtF, 5.0f, dtF, "0.000", "Ts"
            ));

            // tau_M : dt 단위 그리드 (자동 튜닝이 τ = dt 로 세팅하므로 정확히 표시되게).
            table.AddInterpretter(MakeSliderFloat(
                "Delay τ_M (s)",
                "Plant delay (dead-time). 0 = no delay.\nGrid is dt (FTD tick).\nAuto-tuning estimates this automatically.\n---\n플랜트 지연. 0이면 지연 없음.\n그리드 단위는 dt(FTD 틱).\n자동 튜닝 시 자동 추정됩니다.",
                () => _s.ModelDelayTau,
                f => _s.ModelDelayTau = Clamp(f, 0f, 5f),
                0f, 5f, dtF, "0.000", "tau"
            ));

            // min samples
            table.AddInterpretter(MakeSliderInt(
                "Min samples",
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
            seg.NameWhereApplicable = "Excitation";
            seg.SpaceAbove = 10f;
            seg.SpaceBelow = 10f;

            seg.AddInterpretter(MakeToggle(
                "Enable excitation",
                "Adds excitation signal to SetPointAdjust during recording.\nAuto-tuning configures this automatically.\n---\n녹화 중 SetPointAdjust에 가진 신호를 더합니다.\n자동 튜닝 시 자동 설정됩니다.",
                () => _s.ExciteEnabled,
                b => _s.ExciteEnabled = b,
                "excite"
            ));

            // Axis type 선택 (cycle 버튼)
            seg.AddInterpretter(MakeCycleButton(
                "Axis type",
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
                "Fix other axes",
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
                "Amplitude A",
                "Excitation amplitude. Auto-tuning sets this automatically.\n---\n자극 진폭. 자동 튜닝 시 자동 설정됩니다.",
                () => _s.ExciteAmp,
                f => _s.ExciteAmp = Clamp(f, 0f, 10f),
                0f, 10f, 0.05f, "0.00", "A"
            ));

            excTable.AddInterpretter(MakeSliderFloat(
                "Freq base Hz",
                "Base frequency for Sine/MultiSine excitation.\n---\nSine/MultiSine 가진의 기저 주파수.",
                () => _s.ExciteFreqHz,
                f => _s.ExciteFreqHz = Clamp(f, 0.01f, 5.0f),
                0.01f, 5.0f, 0.01f, "0.00", "fBase"
            ));

            excTable.AddInterpretter(MakeSliderFloat(
                "Freq max Hz",
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
            seg.NameWhereApplicable = "Actions";
            seg.SpaceAbove = 10f;
            seg.SpaceBelow = 10f;


            seg.AddInterpretter(new SubjectiveButton<VariableControllerMaster>(
                this._focus,
                M.m<VariableControllerMaster>(_ => _autoState == AutoTuneState.Recording ? "Auto-tuning..." : "Auto Tune"),
                M.m<VariableControllerMaster>(new ToolTip(
                    "Closed-loop auto-tuning: excitation → record → FRIT → PID.\n---\n폐루프 자동 튜닝: 가진 → 녹화 → FRIT → PID.", 260f)),
                null,
                _ => AutoTuneNow()
            ));

            seg.AddInterpretter(MakeButton(
                "Record start/stop",
                "Start/stop sample collection.\nDuring recording, u (output) and y (process variable) are saved every FixedUpdate.\n---\n샘플 수집을 시작/중지합니다.\n" +
                "녹화 중에는 FixedUpdate마다 u(출력), y(과정변수) 샘플을 저장합니다.",
                _ =>
                {
                    if (_sess.Recording) StopRecording();
                    else StartRecording();
                }
            ));

            seg.AddInterpretter(MakeButton(
                "Reset",
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
                "Compute (FRIT)",
                "Compute FRIT: minimize ||y - M·r̃(θ)||² over (Kp,Ti,Td) via Levenberg-Marquardt.\n" +
                "Seeds from current PID values.\n---\n" +
                "FRIT 계산: 현재 PID를 시드로 (Kp,Ti,Td)를 LM 으로 비선형 최적화.\n" +
                "비용: ||y - M·r̃(θ)||² (r̃ = y + u/C(θ))",
                _ => ComputeNow()
            ));

            seg.AddInterpretter(MakeButton(
                "Apply",
                "Apply Kp/Ti/Td to PID. (Kp: 0.001, Ti/Td: 0.1 step)\n---\nKp/Ti/Td를 PID에 적용. (Kp: 0.001, Ti/Td: 0.1 단위)",
                _ => ApplyToPid()
            ));

            seg.AddInterpretter(MakeButton(
                "Validate",
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
            seg.NameWhereApplicable = "Result";
            seg.SpaceAbove = 10f;
            seg.SpaceBelow = 10f;

            seg.AddInterpretter(new SubjectiveDisplay<VariableControllerMaster>(
                this._focus,
                M.m<VariableControllerMaster>(_ =>
                {
                    if (!_sess.HasResult)
                        return "No result yet. Press Compute.";

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
            FritExcitationInjector.Clear(this._focus);   // u-인젝션 반드시 해제
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
                    // 피치가 튜닝 대상: SP-direct 가진 위에 고도 offset 더함.
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

            // ── 사전 진단 단계 진입 ──
            // 가진 OFF 로 3초 동안 |u| 통계 관찰 → 현재 PID 가 이미 limit-cycle 이거나
            // 지속 포화 상태면 즉시 fail + 구체적 Kp 권장값 표시.
            // 통과하면 OnUiFixed 의 Diagnosing 핸들러가 StartAutoTuneRecording 호출.
            _sess.Clear();
            _autoState = AutoTuneState.Diagnosing;
            _sess.LastMessage = "Diagnosing initial PID (no excitation, 3s)... / 초기 PID 진단 중 (3초)";
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
        /// 1) 전체 (u, y, sat) 데이터 사용 — 임펄스 패턴이라 prelude 분리 불필요
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

            // ── ARX(2,1) plant identification + SIMC PID design ──
            // FRIT 의 비선형 LM + cost surface 의존성 대신 classical indirect ID:
            //   1. (u, y) 데이터에 linear LS → plant G 추출 (deterministic, single solution)
            //   2. G 의 시정수/극점/게인에 SIMC 공식 적용 → PID
            // controller-invariant (G 는 plant property), local minimum 없음, iteration 자연 수렴.
            double tauM = Math.Max(dt, (double)_s.ModelDelayTau);

            PlantModel plant = IdentifyPlantArx(u, y, sat, dt, tauM);

            if (!plant.Valid)
            {
                _autoState = AutoTuneState.Failed;
                _sess.LastMessage = $"Plant ID failed: {plant.Diagnosis} / plant 식별 실패";
                return;
            }

            // 목표 Ts 선택: 사용자가 SettlingTimeTs 슬라이더로 지정 가능.
            // 자동 default: τ_1 (1·τ_p, SIMC aggressive). 너무 빠르면 사용자가 슬라이더로 조정.
            double targetTs = (double)_s.SettlingTimeTs;
            if (targetTs <= 0 || targetTs > 5.0)
                targetTs = plant.Tau1; // 자동 default
            targetTs = Math.Max(3.0 * dt, Math.Min(5.0, targetTs));

            SimcResult simc = DesignSimcPid(plant, targetTs);

            // FTD slider 단위로 반올림
            double kpFinal = Math.Max(0.001, Math.Round(simc.Kp * 1000.0) / 1000.0);
            double tiFinal = Math.Round(simc.Ti * 10.0) / 10.0;
            double tdFinal = Math.Round(simc.Td * 100.0) / 100.0;

            _s.SettlingTimeTs = (float)targetTs;

            _sess.HasResult = true;
            _sess.Kp = kpFinal; _sess.Ti = tiFinal; _sess.Td = tdFinal;
            _sess.KpSE = 0; _sess.TiSE = 0; _sess.TdSE = 0;  // ARX 는 G 의 SE 만 제공
            _sess.FitRmse = plant.FitRmse;

            _autoState = AutoTuneState.Done;
            double tauPct = plant.Tau1 > 0 ? (plant.TauSE / plant.Tau1 * 100.0) : 0;
            _sess.LastMessage =
                $"Done | {simc.Form} | τ_1={plant.Tau1:0.000}s±{tauPct:0}% " +
                $"τ_2={plant.Tau2:0.000}s K={plant.K:0.000} → Ts={targetTs:0.000}s " +
                $"Kp={kpFinal:0.000} Ti={tiFinal:0.0} Td={tdFinal:0.00} " +
                $"(ARX rmse={plant.FitRmse:0.0000})";
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

        // ════════════════════════════════════════════════════════════════════════
        // OnDiagnoseTick — Auto Tune 직후 3초 사전 진단
        // ════════════════════════════════════════════════════════════════════════
        //
        // 목적: 가진을 켜기 전에 현재 PID 가 "튜닝 가능한 상태" 인지 확인.
        // 가진 OFF 상태에서 |u| 통계 만 모음 → 3초 후 판정.
        //
        // 판정 기준 (윈도우 끝에서):
        //   1. Limit cycle: satRate > 40%, crossRate > 0.5회/s, uSwing > 1.6
        //      → u 가 ±포화 사이 진동. 보통 Kp 너무 큼.
        //   2. 지속 포화: satRate > 40% (진동 적음)
        //      → u 가 한쪽 rail 에 박혀있음.
        //   3. 살짝 포화: satRate 15~40% → 경고 후 진행
        //   4. 정상: satRate < 15% → 정상 진행
        //
        // 실패 시 권장 Kp = currentKp × 0.4 (limit cycle) 또는 × 0.5 (지속 포화).
        // ════════════════════════════════════════════════════════════════════════
        private void OnDiagnoseTick(double dt)
        {
            if (_autoState != AutoTuneState.Diagnosing) return;

            const double DIAG_DURATION = 3.0;

            if (this._focus == null)
            {
                _autoState = AutoTuneState.Failed;
                _sess.LastMessage = "Diag failed: no focus / 진단 실패: focus 없음";
                return;
            }
            var c = this._focus.GetCurrentController();
            if (c == null)
            {
                _autoState = AutoTuneState.Failed;
                _sess.LastMessage = "Diag failed: no controller / 진단 실패: 컨트롤러 없음";
                return;
            }

            double u = c.LastControlVariable;

            // 통계 누적
            if (_sess.DiagSampleCount == 0)
            {
                _sess.DiagUMax = u;
                _sess.DiagUMin = u;
            }
            else
            {
                if (u > _sess.DiagUMax) _sess.DiagUMax = u;
                if (u < _sess.DiagUMin) _sess.DiagUMin = u;
            }
            if (Math.Abs(u) >= _s.SaturationThreshold) _sess.DiagSatCount++;
            if (_sess.DiagSampleCount > 0 && _sess.DiagPrevU != 0
                && Math.Sign(u) != Math.Sign(_sess.DiagPrevU))
                _sess.DiagSignChanges++;
            _sess.DiagPrevU = u;
            _sess.DiagSampleCount++;
            _sess.DiagT += dt;

            // 진행 표시
            double uPeakSoFar = Math.Max(Math.Abs(_sess.DiagUMax), Math.Abs(_sess.DiagUMin));
            _sess.LastMessage = $"Diagnosing... {_sess.DiagT:0.0}s/{DIAG_DURATION:0.0}s (uPeak={uPeakSoFar:0.00}) / 진단 중";

            if (_sess.DiagT < DIAG_DURATION) return;

            // ── 판정 ──
            // PIDSupporter 는 "튜너" 가 아니라 "보조" 도구. 관찰된 현상과 의심 가능한
            // 원인들만 사용자에게 전달하고, 구체적인 게인 변경은 사용자의 판단에 맡긴다.
            double satRate = (double)_sess.DiagSatCount / Math.Max(1, _sess.DiagSampleCount);
            double uSwing = _sess.DiagUMax - _sess.DiagUMin;
            double uPeak = Math.Max(Math.Abs(_sess.DiagUMax), Math.Abs(_sess.DiagUMin));
            double crossRate = _sess.DiagSignChanges / DIAG_DURATION;

            // (1) Limit cycle: ±포화 사이 빠른 진동
            if (satRate > 0.40 && crossRate > 0.5 && uSwing > 1.6)
            {
                _autoState = AutoTuneState.Failed;
                _sess.LastMessage =
                    $"⚠ Limit cycle detected (u={_sess.DiagUMin:0.00}~{_sess.DiagUMax:0.00}, " +
                    $"{crossRate:0.0}/s, sat={satRate:P0}). " +
                    $"Likely causes: Kp too high / Ti too low (windup) / Td too high. Reduce gains and retry. " +
                    $"/ Limit cycle 감지. 의심 원인: Kp 과대 / Ti 과소 (windup) / Td 과대. 게인 낮춰 재시도.";
                return;
            }

            // (2) 지속 포화: 한쪽 rail 고정
            if (satRate > 0.40)
            {
                _autoState = AutoTuneState.Failed;
                _sess.LastMessage =
                    $"⚠ Persistent saturation (sat={satRate:P0}, uPeak={uPeak:0.00}). " +
                    $"Likely causes: Kp/Ki too high, or SetPoint beyond actuator range. Check and retry. " +
                    $"/ 지속 포화. 의심 원인: Kp/Ki 과대 또는 SetPoint 가 액추에이터 한계 초과. 점검 후 재시도.";
                return;
            }

            // (3) 진단 통과 — Recording 단계로 이행
            string warn = (satRate > 0.15)
                ? $" (주의: 초기 sat={satRate:P0})"
                : "";

            // 진단 상태 초기화 (사용한 _sess.T 는 StartRecording 에서 Clear 됨)
            _sess.LastMessage = $"Diag OK (uPeak={uPeak:0.00}, sat={satRate:P0}). 녹화 시작{warn}";
            StartAutoTuneRecording();
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
        /// 매 틱마다 SetPoint 에 가진 신호를 더한다 (SP-direct).
        /// PID 가 가진된 SP 를 추종하느라 u 를 만들고, plant 가 응답.
        /// ARX 식별이 이 (u, y) 에서 plant G 를 추출.
        ///
        /// 왜 SP-direct (u-direct 가 아니라):
        /// - FRIT 의 r̃(θ) = y + u/C(θ) 수식이 "u 는 순수 PID 출력" 가정
        /// - u-direct 로 외부 ε 주입 시 ε/C(θ) 여분항이 cost surface 왜곡 → LM 이상 PID 수렴
        /// - SP-direct 는 가정 만족 → cost 깔끔 + ARX 도 well-conditioned closed-loop OLS
        /// - ARX 의 invariance: G 는 plant 고유 → C₀ 변해도 같은 G 추출 → iteration 자연 수렴
        /// </summary>
        /// <summary>현재 틱 가진값 (telemetry).</summary>
        private double _lastExciteValue = 0.0;

        private void ApplyExcitation(float dt)
        {
            _lastExciteValue = 0.0;
            FritExcitationInjector.Clear(this._focus);  // u-direct patch 무력화

            if (!_s.ExciteEnabled) return;
            if (_s.ExciteWave == WaveType.Off) return;
            if (!_hasBaseSetPointAdjust) return;

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
                        double f0 = Math.Max(0.01, _s.ChirpStartHz);
                        double f1 = Math.Max(f0 * 1.1, _s.ChirpEndHz);
                        double T = Math.Max(1.0, _s.MinSamples * dt);
                        double ratio = f1 / f0;
                        double lnRatio = Math.Log(ratio);
                        double phase;
                        if (t <= T)
                            phase = 2.0 * Math.PI * f0 * T / lnRatio * (Math.Pow(ratio, t / T) - 1.0);
                        else
                            phase = 2.0 * Math.PI * f0 * T / lnRatio * (ratio - 1.0) + 2.0 * Math.PI * f1 * (t - T);
                        x = amp * Math.Sin(phase);
                        break;
                    }
                case WaveType.MultiSine:
                    {
                        // Multi-width doublet 패턴 (aerospace 3-2-1-1 계열).
                        // 폭이 다른 doublet (+ 직후 - pulse) 을 순서대로 적용:
                        //   넓은 doublet  → 낮은 주파수 자극 (느린 plant 모드)
                        //   좁은 doublet  → 높은 주파수 자극 (빠른 plant 모드)
                        // 각 doublet 이 평균 zero 라 integrator drift 즉시 상쇄.
                        // 그 후 긴 정적 구간에 closed-loop 자연 응답 관찰.
                        //
                        // 폭 W 인 doublet 의 주 자극 주파수 ≈ 1/(2W):
                        //   1.5s → 0.33 Hz (slow modes, τ ~ 0.5-2s 인 plant)
                        //   0.7s → 0.7 Hz  (medium, τ ~ 0.2-0.5s)
                        //   0.3s → 1.7 Hz  (fast, τ ~ 0.1-0.2s)
                        //
                        // 다양한 폭으로 광대역 Fisher information → plant 시정수 모르는 상태에서도
                        // 일부 doublet 이 plant 모드에 맞아 식별성 확보. 그래도 평균 zero 유지로 안정.
                        //
                        // 스케줄 (초): 6.6초 동안 자극, 이후 ~18초 정적/관찰.
                        if (t < 1.5)        x = +amp;   // doublet 1 + (low)
                        else if (t < 3.0)   x = -amp;   // doublet 1 -
                        // 3.0 ~ 4.0: 1초 휴식
                        else if (t < 4.0)   x = 0;
                        else if (t < 4.7)   x = +amp;   // doublet 2 + (mid)
                        else if (t < 5.4)   x = -amp;   // doublet 2 -
                        // 5.4 ~ 6.0: 0.6초 휴식
                        else if (t < 6.0)   x = 0;
                        else if (t < 6.3)   x = +amp;   // doublet 3 + (high)
                        else if (t < 6.6)   x = -amp;   // doublet 3 -
                        // 6.6 이후: 모두 0 (긴 회복 + 정적 관찰)
                        break;
                    }
            }

            _lastExciteValue = x;
            try { this._focus.SetPointAdjust.Us = _baseSetPointAdjust + (float)x; } catch { }
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

        // ════════════════════════════════════════════════════════════════════════
        // ★ FRIT 핵심 계산 ★
        // ════════════════════════════════════════════════════════════════════════
        //
        // 입력:
        //   u, y        폐루프 수집 데이터 (sample N개)
        //   sat         포화 플래그 (true = |u| ≥ 0.98 일 때)
        //   dt          샘플 간격 (초)
        //   s           Settings (Ts, nM, τM, TransientTailSamples 등)
        //   Kp0,Ti0,Td0 LM 초기 시드 (보통 현재 PID 값)
        //
        // 출력: FritResult { Kp, Ti, Td, KpSE, TiSE, TdSE, Rmse, Iterations, ... }
        //
        // ─────────────────────────────────────────────────────────────────────
        // 알고리즘 흐름 (10 단계)
        // ─────────────────────────────────────────────────────────────────────
        //   1) Detrend       : u, y 에서 DC + 선형 추세 제거
        //   2) M(z) precomp  : 참조 모델 Tustin 이산화 계수 미리 계산 (θ 무관)
        //   3) effSat 확장   : 포화 + IIR transient tail 영역 마킹 → w_sat 초기화
        //   4) LM model fn   : (Kp, Ti, Td) → √w · ŷ  ( 가상레퍼런스 + M 캐스케이드)
        //   5) 초기값 sanity : Kp0/Ti0/Td0 가 비정상이면 안전 기본값
        //   6) IRLS × LM     : 3 회 반복 (각 iter 마다 잔차로 Huber 가중치 갱신)
        //   7) RMSE          : effSat / IRLS-downweighted 제외 raw 잔차
        //   8) CRLB          : FD 자코비안 + 3×3 cov 역행렬 → SE(Kp/Ti/Td)
        //   9) 경계 클램프    : Kp∈[0,1], Ti∈[0.1,250], Td∈[0,10], NaN/Inf 처리
        //  10) FritResult 반환
        //
        // ─────────────────────────────────────────────────────────────────────
        // 핵심 수식
        // ─────────────────────────────────────────────────────────────────────
        //   PID 이산화 (backward Euler 적분/미분):
        //     C(z) = Kp · (a₀ + a₁z⁻¹ + a₂z⁻²) / (1 - z⁻¹)
        //       a₀ = 1 + dt/Ti + Td/dt
        //       a₁ = -(1 + 2·Td/dt)
        //       a₂ = Td/dt
        //
        //   가상 레퍼런스 (시간 영역 IIR 역필터, 1/C(z) · u):
        //     e[k] = (u[k] - u[k-1] - Kp·a₁·e[k-1] - Kp·a₂·e[k-2]) / (Kp·a₀)
        //     r̃[k] = y[k] + e[k]
        //
        //   참조 모델 M(z) (Tustin, nM 차 캐스케이드):
        //     β₀ = 1 + 2·aM/dt,  β₁ = 1 - 2·aM/dt,  aM = 0.2·Ts
        //     H₁(z) · x[k] = (x[k] + x[k-1] - β₁·prev) / β₀
        //     순수 지연 τM/dt 정수 틱 shift
        //
        //   비용 (가중 LS via sqrt-스케일링):
        //     observedY[k] = √w[k] · y[k]
        //     model output = √w[k] · ŷ[k]
        //     LM 이 ||observedY - model||² = Σ w·(y-ŷ)² 를 자동 최소화
        //
        //   가중치 w[k] = w_sat[k] · w_huber[k]:
        //     w_sat:   포화 또는 transient tail 이면 1e-3, 아니면 1
        //     w_huber: |r| ≤ δ 면 1, 아니면 δ/|r|.  δ = 1.5·1.4826·MAD(r)
        //
        // ─────────────────────────────────────────────────────────────────────
        // 안정성: 1/C(z) 의 pole = C(z) 분자 zero. 단위원 밖이면 역필터 발산.
        //         → LM 모델 함수 시작에 zero 위치 체크, 불안정 시 큰 residual 반환 (soft barrier).
        //
        // 왜 시간 영역? (이전 주파수 영역 구현 대비)
        //   1. FFT 없음 → LM 반복당 빠름
        //   2. 포화로 인한 spectral leakage 없음 → 가중치 100% 유효
        //   3. circular convolution wrap-around 없음 (causal IIR)
        //   4. 순수 지연을 정수 틱으로 정확히 처리
        // ════════════════════════════════════════════════════════════════════════
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
            // 안전 하한: 2.5·dt (β₁ ≤ 0 보장, LP 안정성). dt=0.02 일 때 0.05s.
            double ts = Math.Max(2.5 * dt, s.SettlingTimeTs);
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

        // ════════════════════════════════════════════════════════════════════════
        // ARX(2,1) Plant Identification + SIMC PID Design
        // ════════════════════════════════════════════════════════════════════════
        //
        // 표준 closed-loop ID 정공법 (indirect method):
        //   1. (u, y) 데이터에 ARX(2,1) OLS 로 plant G 직접 식별
        //   2. G 의 극점/시정수/게인 추출
        //   3. SIMC 공식 (Skogestad) 으로 PID 산출
        //
        // FRIT 와의 차이:
        //   - FRIT: 비선형 LM, local minima 위험, cost surface 가 C₀ 의존
        //   - ARX+SIMC: linear LS, 단일 해, plant G 만 추출하면 C₀ 무관
        //
        // 왜 controller-invariant 한가:
        //   - G 는 plant 의 물리적 property (질량, 관성, 공력 etc.)
        //   - 컨트롤러가 무엇이든 같은 plant 면 같은 G
        //   - ARX OLS 가 (u, y) 의 회귀 관계에서 G 추출 → C₀ 무관
        //   - white innovation 노이즈 가정 하에 closed-loop 에서도 OLS unbiased
        //     (Ljung "System Identification" §10 등 표준 결과)
        // ════════════════════════════════════════════════════════════════════════

        /// <summary>식별된 plant model 결과.</summary>
        public struct PlantModel
        {
            public double Tau1;     // 첫 번째 시정수 (큰 쪽, dominant slow pole)
            public double Tau2;     // 두 번째 시정수 (작은 쪽, fast pole), 0 이면 1차 plant
            public double K;        // DC gain (단위 입력당 정상상태 출력)
            public double Theta;    // 순수 지연 (초)
            public double FitRmse;  // ARX fit 잔차 RMS (작을수록 모델 정확)
            public double TauSE;    // dominant τ_1 의 standard error (불확실성)
            public bool HasIntegrator; // |z₁| ≈ 1 (적분기 plant)
            public bool Valid;      // 식별 성공 여부
            public string Diagnosis; // 실패 시 이유 / 성공 시 정보
        }

        /// <summary>
        /// (u, y) 데이터에 ARX(2,1) OLS 로 plant G 식별.
        ///   y[k] = a₁·y[k-1] + a₂·y[k-2] + b·u[k-1-δ]
        /// 특성다항식 z² - a₁ z - a₂ = 0 의 근 → 이산 극점 → 연속 시정수.
        /// b 와 극점에서 DC gain K 계산.
        /// </summary>
        private static PlantModel IdentifyPlantArx(double[] u, double[] y, bool[] sat, double dt, double theta)
        {
            PlantModel m = new PlantModel { Theta = theta };
            int N = Math.Min(u.Length, Math.Min(y.Length, sat.Length));
            if (N < 64) { m.Diagnosis = "data too short"; return m; }

            int delayN = Math.Max(0, (int)Math.Round(theta / dt));
            int kStart = 2 + delayN;
            if (kStart >= N - 4) { m.Diagnosis = "delay > data"; return m; }

            double yStd = StdDev(y);
            if (yStd < 1e-4) { m.Diagnosis = "y barely moves — excitation too weak?"; return m; }

            // Detrend
            double[] yd = new double[N]; Array.Copy(y, yd, N); Detrend(yd);
            double[] ud = new double[N]; Array.Copy(u, ud, N); Detrend(ud);

            // ARX(2,1) 정규방정식 (3×3): regressors = [y[k-1], y[k-2], u[k-1-δ]]
            double s11 = 0, s12 = 0, s13 = 0, s22 = 0, s23 = 0, s33 = 0;
            double t1 = 0, t2 = 0, t3 = 0;
            int count = 0;
            for (int k = kStart; k < N; k++)
            {
                if (sat[k] || sat[k - 1] || sat[k - 2] || sat[k - 1 - delayN]) continue;
                double yt = yd[k];
                double y1 = yd[k - 1];
                double y2 = yd[k - 2];
                double u1 = ud[k - 1 - delayN];
                s11 += y1 * y1; s12 += y1 * y2; s13 += y1 * u1;
                s22 += y2 * y2; s23 += y2 * u1;
                s33 += u1 * u1;
                t1 += yt * y1; t2 += yt * y2; t3 += yt * u1;
                count++;
            }
            if (count < 32) { m.Diagnosis = $"too few clean samples ({count})"; return m; }

            // 3×3 inversion (Cramer)
            double M11 = s22 * s33 - s23 * s23;
            double M12 = s12 * s33 - s23 * s13;
            double M13 = s12 * s23 - s22 * s13;
            double det = s11 * M11 - s12 * M12 + s13 * M13;
            if (Math.Abs(det) < 1e-12) { m.Diagnosis = "regressors collinear"; return m; }

            double a1 = (t1 * M11 - s12 * (t2 * s33 - s23 * t3) + s13 * (t2 * s23 - s22 * t3)) / det;
            double a2 = (s11 * (t2 * s33 - s23 * t3) - t1 * (s12 * s33 - s23 * s13) + s13 * (s12 * t3 - t2 * s13)) / det;
            double b  = (s11 * (s22 * t3 - s23 * t2) - s12 * (s12 * t3 - s13 * t2) + t1 * (s12 * s23 - s22 * s13)) / det;
            if (double.IsNaN(a1) || double.IsNaN(a2) || double.IsNaN(b))
            { m.Diagnosis = "NaN in fit"; return m; }

            // 잔차 RMSE
            double sqResid = 0;
            int cResid = 0;
            for (int k = kStart; k < N; k++)
            {
                if (sat[k] || sat[k - 1] || sat[k - 2] || sat[k - 1 - delayN]) continue;
                double pred = a1 * yd[k - 1] + a2 * yd[k - 2] + b * ud[k - 1 - delayN];
                double e = yd[k] - pred;
                sqResid += e * e;
                cResid++;
            }
            m.FitRmse = Math.Sqrt(sqResid / Math.Max(1, cResid));

            // SE on a1 (proxy for τ_1 uncertainty): σ² · (M⁻¹)₁₁ / count
            double sigma2 = m.FitRmse * m.FitRmse;
            double seA1 = Math.Sqrt(sigma2 * M11 / Math.Max(1e-12, det));

            // 특성다항식 z² - a₁ z - a₂ = 0 → 이산 극점
            double disc = a1 * a1 + 4.0 * a2;
            double z1, z2;
            bool complex = disc < 0;
            if (!complex)
            {
                double sq = Math.Sqrt(disc);
                z1 = (a1 + sq) / 2.0;
                z2 = (a1 - sq) / 2.0;
            }
            else
            {
                // 복소 conjugate. |z|² = -a₂. 진동 plant — 우리는 dominant magnitude 시정수만 봄.
                double mag2 = -a2;
                if (mag2 <= 0) { m.Diagnosis = "complex poles, -a₂ ≤ 0"; return m; }
                double mag = Math.Sqrt(mag2);
                z1 = mag; z2 = mag;  // 둘 다 magnitude 만, 진동 frequency 는 무시
            }

            double az1 = Math.Abs(z1), az2 = Math.Abs(z2);

            // 큰 |z| = 느린 극점 (dominant), 작은 |z| = 빠른 극점
            double zSlow = Math.Max(az1, az2);
            double zFast = Math.Min(az1, az2);

            // 적분기 plant 관용 처리:
            //   진짜 적분기는 z=1. 노이즈로 추정치가 0.95~1.1 사이에 떠다닐 수 있음.
            //   엄격한 z<1 거부는 비행기 롤 angle 같은 적분기 plant 를 false reject 함.
            //   대신: 0.95 이상이면 적분기로 간주, 내부 계산엔 0.9999 cap.
            //   1.1 이상이면 진짜 불안정 추정 → fail.
            const double INTEGRATOR_LO = 0.95;
            const double INTEGRATOR_CAP = 0.9999;
            const double UNSTABLE_THRESHOLD = 1.1;

            if (zSlow > UNSTABLE_THRESHOLD)
            { m.Diagnosis = $"slow pole |z|={zSlow:0.000} > 1.1 (truly unstable)"; return m; }
            if (zSlow <= 0)
            { m.Diagnosis = $"slow pole |z|={zSlow:0.000} ≤ 0 (invalid)"; return m; }

            m.HasIntegrator = zSlow > INTEGRATOR_LO;
            double zForTau = m.HasIntegrator ? INTEGRATOR_CAP : zSlow;
            m.Tau1 = -dt / Math.Log(zForTau);

            m.Tau2 = (zFast > 0.001 && zFast < INTEGRATOR_LO)
                ? -dt / Math.Log(zFast)
                : 0.0;

            // DC gain K 계산:
            //   일반 plant:  K = b / (1 - a₁ - a₂)   (적분기 아닐 때)
            //   적분기 plant: K_i = b / dt           (denom ≈ 0)
            // HasIntegrator 가 true 이거나 denom 이 작으면 적분기 공식 사용.
            double denom = 1.0 - a1 - a2;
            if (m.HasIntegrator || Math.Abs(denom) < 1e-3)
            {
                m.K = b / Math.Max(1e-6, dt);
                m.HasIntegrator = true;
            }
            else
            {
                m.K = b / denom;
            }

            // τ_1 SE 근사: dτ/da₁ × σ_a₁  (chain rule)
            // z₁ = (a₁ + √(a₁² + 4a₂)) / 2 가 a₁ 에 대해 ∂z₁/∂a₁ = 0.5 + a₁/(2√disc)
            // τ = -dt / ln(z) → ∂τ/∂z = dt/(z·ln²z)
            double dtau_dz = (zSlow > 1e-6 && zSlow < 1) ? dt / (zSlow * Math.Log(zSlow) * Math.Log(zSlow)) : 0;
            double dz_da = !complex ? 0.5 + a1 / (2.0 * Math.Max(1e-12, Math.Sqrt(disc))) : 0.5;
            m.TauSE = Math.Abs(dtau_dz * dz_da * seA1);

            m.Valid = true;
            m.Diagnosis = m.HasIntegrator
                ? $"integrator plant, τ_other={m.Tau2:0.000}s, K={m.K:0.000}"
                : (m.Tau2 > 0.01
                    ? $"2nd-order: τ_1={m.Tau1:0.000}s τ_2={m.Tau2:0.000}s K={m.K:0.000}"
                    : $"1st-order: τ_p={m.Tau1:0.000}s K={m.K:0.000}");
            return m;
        }

        /// <summary>식별된 plant 에 SIMC PID 공식 적용.</summary>
        public struct SimcResult
        {
            public double Kp, Ti, Td;
            public string Form;        // "PI", "PID", "PI for integrator"
        }

        /// <summary>
        /// Skogestad SIMC 공식.
        ///   1차 plant K/(τ_p s + 1):    Kp = τ_p / (K·(τ_c+θ)),  Ti = min(τ_p, 4(τ_c+θ))
        ///   2차 plant K/((τ_1 s+1)(τ_2 s+1)): Kp = τ_1/(K·(τ_c+θ)), Ti = min(τ_1, 4(τ_c+θ)), Td = τ_2
        ///   적분기 K_i/s:               Kp = 1/(K_i·(τ_c+θ)),     Ti = 4(τ_c+θ),              Td = 0
        /// </summary>
        private static SimcResult DesignSimcPid(PlantModel m, double targetTs)
        {
            double tauC = Math.Max(targetTs, m.Theta);  // closed-loop τ_c ≥ θ
            double kInv = 1.0 / Math.Max(1e-6, Math.Abs(m.K));
            int sign = Math.Sign(m.K);
            if (sign == 0) sign = 1;

            SimcResult r;

            if (m.HasIntegrator)
            {
                // K_i / (s(τ_2 s + 1))  또는  K_i / s
                r.Kp = sign * kInv / (tauC + m.Theta);
                r.Ti = 4.0 * (tauC + m.Theta);
                r.Td = m.Tau2;  // 0 이면 PI
                r.Form = m.Tau2 > 0.01 ? "PID for integrator" : "PI for integrator";
            }
            else if (m.Tau2 > 0.01)
            {
                // 2차 plant
                r.Kp = sign * m.Tau1 * kInv / (tauC + m.Theta);
                r.Ti = Math.Min(m.Tau1, 4.0 * (tauC + m.Theta));
                r.Td = m.Tau2;
                r.Form = "PID";
            }
            else
            {
                // 1차 plant
                r.Kp = sign * m.Tau1 * kInv / (tauC + m.Theta);
                r.Ti = Math.Min(m.Tau1, 4.0 * (tauC + m.Theta));
                r.Td = 0.0;
                r.Form = "PI";
            }

            // FTD 입력 사정 (Ti 무한대 표현은 250)
            if (r.Ti > 250.0) r.Ti = 250.0;
            if (r.Ti < 0.1) r.Ti = 0.1;
            if (r.Td < 0) r.Td = 0;
            if (r.Td > 10.0) r.Td = 10.0;
            return r;
        }

    }
}
