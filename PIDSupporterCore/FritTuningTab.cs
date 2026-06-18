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
// │              → Ts 산출 (closed-loop BW, Skogestad-Postlethwaite §2.4.5)  │
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
//   [자체] AutoTuneCompute()                   녹화 완료 → CL-BW Ts + FRIT
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
//   [자체] RoundToStep(v, step)                step 단위 반올림
//   [자체] Clamp / ClampInt                    범위 제한
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
            QtDiagnosing, // Quick Tune: 진단 단계 (3초) — limit cycle 체크 (relay 시작 전)
            QtRelayWarmup, // Quick Tune (Relay): warm-up — 첫 1-2 주기 transient 버림
            QtRelayMeasure, // Quick Tune (Relay): stable limit cycle 측정 (3+ 주기)
        }

        /// <summary>축 타입 — 사용자가 각 tab 에서 지정. 피치 고도유지 로직 등에 사용.</summary>
        // AxisType / 축 분리 기능 제거됨 — cross-coupling 포함 데이터로 실제 환경 plant 식별.
        //   다른 축은 AI 가 자유 제어 → 실제 비행 환경 재현
        //   coupling 영향은 FRIT cost 의 small noise term 으로 흡수

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

            // ===== 포화 처리 =====
            public float SaturationThreshold = 0.98f;   // |u| >= 이 값이면 포화로 판정

            // ===== 가진(Excitation) =====
            public bool ExciteEnabled = true;       // 가진 켤지

            // ===== Quick Tune (Relay feedback — Åström-Hägglund 1984) =====
            // h: relay 출력 크기. u = ±h based on sign(SP - y).
            //    클수록 진폭 (A) 크고 SNR 좋음. 작을수록 함체 흔들림 적음.
            // ε: hysteresis. 0 이면 pure relay, > 0 이면 noise rejection.
            //    너무 크면 dead-zone 영향, 너무 작으면 noise chatter.
            public float RelayH = 0.3f;             // relay 출력 진폭 (FTD u 단위, [-1, 1] 의 일부)
            public float RelayHysteresis = 0.0f;    // hysteresis ε (PV 단위, 0 = pure relay)
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

            public readonly List<double> U = new List<double>();          // 제어 출력 기록 (= u_actual, post-clip)
            public readonly List<double> Y = new List<double>();          // 프로세스 변수 기록 (전 샘플)
            public readonly List<double> R = new List<double>();          // SP-direct 가진 신호 (FRIT instrument)
            public readonly List<double> UInject = new List<double>();    // u-direct 가진 신호 (per tick, clip 전)
                                                                          // u_PID = u_actual - u_inject (non-sat 샘플에서 정확)
            public readonly List<bool>   Saturated = new List<bool>();    // 이 샘플이 포화 중인지 (cost 에서 제외)

            // ── 포화 회복 추적 ──
            public int SaturatedCount;

            // 사전 진단 (Diagnosing 단계, 3초): 가진 OFF — limit cycle / 지속 포화 검출.
            //   통과하면 즉시 recording 진입.
            public double DiagT;            // 진단 누적 시간 (초)
            public int    DiagSampleCount;
            public double DiagUMax, DiagUMin;
            public int    DiagSatCount;     // |u| ≥ 임계 카운트
            public int    DiagSignChanges;  // u 부호 변환 횟수
            public double DiagPrevU;        // 직전 u (부호 변환 검출용)
            public double DiagYBaseline;    // 진단 동안 y 평균 (recording baseline 으로 인계)
            public double DiagYBaselineSum;
            public int    DiagYBaselineCnt;

            // PRBS (Pseudo-Random Binary Sequence) 가진 상태.
            // 10-bit LFSR, 다항식 x^10 + x^7 + 1 (maximum length, period 2^10 - 1 = 1023).
            // 각 bit 가 PRBS_BIT_DURATION 틱 동안 유지.
            public int PrbsState;                   // LFSR 현재 상태
            public int PrbsTicksInBit;              // 현재 비트가 출력된 틱 수
            public double PrbsCurrentValue;         // ±1 (이번 비트 값)

            // u-target adaptive amplitude: closed-loop 의 plant input power 를 C 와 무관하게 일정 유지.
            //   user 가 설정한 ExciteAmp 의 의미 = u 의 target std (post-clip plant input 진폭).
            //   weak C → 작은 |T(z)| → 같은 r 에서 u_std 작음 → amp 자동 증가.
            //   strong C → 큰 |T| → u_std 큼 → amp 자동 감소 (saturation 방지).
            //   학계: Hjalmarsson 2005 "input design with power constraint".
            public double AmpDyn;                   // 현재 동적 가진 진폭
            public int    TicksSinceAmpAdjust;      // 마지막 amp 조정 후 경과 틱

            // PRBS HPF state (drift cancellation): fc ≈ 0.01Hz HPF.
            //   적분기 plant + finite-window PRBS → 누적 DC bias → 비행기 천천히 drift.
            //   매우 저주파 HPF 로 DC 만 제거, fast PRBS dynamics 유지.
            //   학계: closed-loop input design with bias removal (Ljung §13.5).
            public double PrbsHpfInPrev;            // 이전 input (HPF state)
            public double PrbsHpfOutPrev;           // 이전 output (HPF state)

            // Adaptive PRBS bit duration — data-driven spectrum coverage.
            //   매 K초 마다 y/r sensitivity 분석 → 부족한 band 식별 → bit_ticks 동적 조정.
            //   bit_ticks 가 작으면 high-freq 자극, 크면 low-freq.
            //   decade band 와 대응 (fs-independent):
            //     64 → low  [0.05, 0.5] Hz
            //      8 → mid  [0.5, 5]    Hz
            //      2 → high [5, Nyquist] Hz   (Nyquist = fs/2)
            public int    PrbsBitTicks = 8;         // 초기값: mid band
            public int    TicksSinceSpectralCheck;  // 마지막 spectral check 후 경과

            // Hybrid 가진 — u-direct 보조 amp (headroom-based)
            public double UAmpDyn;                  // 현재 u-direct base amplitude (dyn 조정)

            // Information-driven termination state
            //   매 6초 max(S) 측정 → 연속 3 windows < ε 이면 well-tuned 종료
            public int    WellTunedConsecutiveCount;
            public double LastMaxSensitivity;        // 가장 최근 측정된 max(S)
            public double LastSLo;                    // 마지막 |S(low band)| — UI 시각화
            public double LastSMid;                   // 마지막 |S(mid band)|
            public double LastSHi;                    // 마지막 |S(high band)|
            // Coherence γ²(f) ∈ [0,1] — Welch cross-spectrum 의 신뢰도 metric
            //   > 0.7: 신뢰 가능, < 0.3: noise dominated
            public double LastCohLo;     // band-aggregated (UI 표시용)
            public double LastCohMid;
            public double LastCohHi;
            // Per-bin γ²(f) — Welch bin 별 (Bazanella full H₂ implementation 용)
            //   length = SEG/2, frequency bin i 의 γ²(f_i)
            public double[]? LastCohPerBin;

            // Per-bin Welch 스펙트럼 (closed-loop bandwidth Ts 추정용)
            //   T(jω) = S_yr/S_rr (Bendat-Piersol §6.4 H₁ estimator)
            //   |T(jω_B)|² = 0.5 의 ω_B 가 closed-loop bandwidth → M(s) 매칭
            //   length = SEG/2
            public double[]? LastSRR;
            public double[]? LastSYRRe;
            public double[]? LastSYRIm;
            public double   LastBinWidthHz;

            // ── Quick Tune (Relay feedback) 상태 ──
            //   QtPhase 0: warm-up (relay 시작, 첫 1~2 주기는 transient → 버림)
            //   QtPhase 1: 측정 (stable limit cycle → 주기/진폭 측정)
            //   QtPhase 2: done (relay 해제 → 계산)
            public int    QtPhase;          // 0=warmup, 1=measure
            public double QtT;              // 현재 phase 누적 시간
            public double QtY0;             // SP baseline (relay 시작 시점의 y → SP 로 사용)
            public float  QtRelayCurrent;   // 현재 relay 출력 (+h 또는 -h)
            public double QtYMin, QtYMax;   // 현재 주기 안에서의 y peak
            public readonly List<double> QtCycleTimes = new List<double>();    // zero-crossing (positive→negative) 시점 누적
            public readonly List<double> QtCycleAmps  = new List<double>();    // 매 주기의 peak-to-peak 진폭
            public double QtKc, QtTc;       // 측정 결과: critical gain & period
            public bool   QtIsStepResult;    // 결과가 Quick Tune (relay) 인지 (FRIT vs Quick 구분 표시)

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
                R.Clear();
                UInject.Clear();
                Saturated.Clear();
                SaturatedCount = 0;
                DiagT = 0;
                DiagSampleCount = 0;
                DiagUMax = DiagUMin = 0;
                DiagSatCount = 0;
                DiagSignChanges = 0;
                DiagPrevU = 0;
                DiagYBaseline = 0;
                DiagYBaselineSum = 0;
                DiagYBaselineCnt = 0;
                QtPhase = 0;
                QtT = 0;
                QtY0 = 0;
                QtRelayCurrent = 0;
                QtYMin = 0; QtYMax = 0;
                QtCycleTimes.Clear();
                QtCycleAmps.Clear();
                QtKc = QtTc = 0;
                QtIsStepResult = false;
                // PRBS 초기화: state = 1 (어떤 non-zero seed 든 OK, 결정론적)
                PrbsState = 1;
                PrbsTicksInBit = 0;
                PrbsCurrentValue = 1.0;   // 첫 bit = state & 1 = 1 → +A
                AmpDyn = 0;               // StartRecording 에서 초기값 0.3 으로 설정
                UAmpDyn = 0;              // StartRecording 에서 초기화
                TicksSinceAmpAdjust = 0;
                PrbsHpfInPrev = 0;
                PrbsHpfOutPrev = 0;
                PrbsBitTicks = 8;         // 초기 = mid band (≈0.16s bit, 새 decade band 기준)
                TicksSinceSpectralCheck = 0;
                WellTunedConsecutiveCount = 0;
                LastMaxSensitivity = 1.0;  // 초기 unknown
                LastSLo = LastSMid = LastSHi = 1.0;
                LastCohLo = LastCohMid = LastCohHi = 0.0;
                LastCohPerBin = null;
                LastSRR = LastSYRRe = LastSYRIm = null;
                LastBinWidthHz = 0.0;
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


        // 축 분리 / 피치 고도 유지 기능 제거됨 — cross-coupling 포함 데이터로 실제 환경 식별.

        // 가진 적용 시 원래 SetPoint를 백업해두고, 녹화 끝나면 복원하기 위한 변수.
        // SetPointAdjust = FTD에서 PID의 목표값을 외부에서 조절하는 파라미터.
        private bool _hasBaseSetPointAdjust;
        private float _baseSetPointAdjust;

        // u-direct excitation 용 y baseline.
        // Diagnose Phase 0 (가진 OFF 3초) 에서 측정된 y 평균. 수집 동안 plant 가 base 에서
        // 너무 벗어나면 (= 비행기 자세 무너지면) amp 줄여서 안전 확보.
        private double _recordingYBaseline;

        /// <summary>
        /// 생성자. FTD가 PID 편집 UI를 열 때 패치에서 호출.
        /// : base(window, focus) = 부모 클래스(SuperScreen) 생성자에 window와 focus를 넘김.
        /// this._focus = focus (부모에서 설정됨) → 이후 this._focus로 PID 제어기에 접근.
        /// </summary>
        public FritTuningTab(ConsoleWindow window, VariableControllerMaster focus) : base(window, focus)
        {
            this.Name = new Content("FRIT Tuning", new ToolTip("Auto-estimate PID (Kp, Ti, Td) via FRIT.\n---\nFRIT로 PID(Kp, Ti, Td)를 자동 추정합니다.", 220f), "frit");
        }

        /// <summary>
        /// FTD UI 시스템이 탭을 그릴 때 호출. UI 요소들을 여기서 생성/배치.
        /// override = 부모 클래스의 같은 이름 메서드를 덮어쓰기.
        /// </summary>
        public override void Build()
        {
            BuildStatus();              // 상태 표시 영역
            BuildDataMonitoring();      // 데이터 수집 모니터링 (sensitivity, sat, amp 등)
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

                // Quick Tune (Relay feedback): 진단 → warm-up → 측정
                if (_autoState == AutoTuneState.QtDiagnosing
                    || _autoState == AutoTuneState.QtRelayWarmup
                    || _autoState == AutoTuneState.QtRelayMeasure)
                {
                    double dtQt = Time.fixedDeltaTime;
                    if (dtQt <= 0) dtQt = 0.02;
                    OnQuickTuneTick(dtQt);
                    return;
                }

                if (!_sess.Recording)
                {
                    RestoreSetPointAdjustIfNeeded();
                    return;
                }

                double dt = Time.fixedDeltaTime;
                if (dt <= 0) dt = 0.02;

                ApplyExcitation((float)dt);

                IVariableController c = this._focus.GetCurrentController();
                if (c == null) return;

                // u: 제어 출력(컨트롤 변수), y: 프로세스 변수, sp: 목표값
                // u 는 plant 가 실제로 받는 값 = clip 된 값. 안전을 위해 명시적 clamp.
                double uRaw = c.LastControlVariable;
                double u = Math.Max(-1.0, Math.Min(1.0, uRaw));
                double y = c.LastProcessVariable;

                // 포화 추적 (telemetry/UI). 회귀에선 clamped u = actual plant input → unbiased.
                bool saturated = Math.Abs(uRaw) >= _s.SaturationThreshold;
                if (saturated) _sess.SaturatedCount++;

                _sess.U.Add(u);
                _sess.Y.Add(y);
                _sess.R.Add(_lastExciteValue);   // SP-direct 가진 = FRIT instrument
                _sess.UInject.Add(_lastUInject); // u-direct 가진 (cost 보정용)
                _sess.Saturated.Add(saturated);

                _sess.T += dt;

                // ── 데이터 driven 종료 (information-based) ──
                //   매 6초 spectral monitor:
                //     1) PRBS bit_ticks 조정 (sensitivity 큰 band 자극)
                //     2) max(|S|) 측정
                //   종료 조건:
                //     A) max(S) < ε_well_tuned for 3 consecutive windows → 종료 ("well-tuned")
                //     B) 60초 hard timeout → 종료 (정보 충분, FRIT 실행)
                //   초기 최소: 256 samples (FFT 가능)
                _sess.TicksSinceSpectralCheck++;
                if (_sess.TicksSinceSpectralCheck >= SPECTRAL_INTERVAL && _sess.U.Count >= SPECTRAL_FFT_LEN)
                {
                    double maxS = UpdatePrbsBitTicksFromSpectrum();
                    _sess.TicksSinceSpectralCheck = 0;
                    if (maxS >= 0)
                    {
                        _sess.LastMaxSensitivity = maxS;
                        if (maxS < WELL_TUNED_S_THRESHOLD)
                            _sess.WellTunedConsecutiveCount++;
                        else
                            _sess.WellTunedConsecutiveCount = 0;
                    }
                }

                if (_sess.U.Count % 240 == 0)
                {
                    string bandName = _sess.PrbsBitTicks == 64 ? "low"
                                    : _sess.PrbsBitTicks == 8 ? "mid"
                                    : "high";
                    _sess.LastMessage =
                        $"Collecting... N={_sess.U.Count} ({_sess.T:0.0}s/{MAX_COLLECT_SEC:0.0}s), " +
                        $"max|S|={_sess.LastMaxSensitivity:0.000}, well-tuned count={_sess.WellTunedConsecutiveCount}/{WELL_TUNED_CONSEC_REQUIRED}, " +
                        $"sat={_sess.SaturatedCount}, amp={_sess.AmpDyn:0.000}/u={_sess.UAmpDyn:0.000}, band={bandName} / 수집중";
                }

                // 종료 — 60초 hard timeout 만 (well-tuned 자동 종료 비활성화).
                //   well-tuned 판정은 *진짜 well-tuned vs 그냥 조용함* 구분이 어려움.
                //   baseline 비교 + SNR check 추가 전에는 단순 timeout 만 사용.
                //   UI 의 max|S|, |S| per band bar 는 관찰용으로 유지 (사용자가 직접 판단).
                if (_autoState == AutoTuneState.Recording && _sess.T >= MAX_COLLECT_SEC)
                {
                    StopRecording();
                    _autoState = AutoTuneState.Computing;
                    _sess.LastMessage = $"Collection {MAX_COLLECT_SEC}s complete, analyzing... / 수집 완료, 분석 중";
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
                        $"Samples: {_sess.U.Count}  (elapsed {_sess.T:0.0}s)\n" +
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

        /// <summary>
        /// Display-only bar (setter no-op).
        /// </summary>
        private SubjectiveFloatClampedWithBar<VariableControllerMaster> MakeDisplayBar(
            float min, float max, float step,
            Func<float> getter, Func<string> labelFn, string tipKo)
        {
            return new SubjectiveFloatClampedWithBar<VariableControllerMaster>(
                M.m<VariableControllerMaster>(_ => min),
                M.m<VariableControllerMaster>(_ => max),
                M.m<VariableControllerMaster>(_ => getter()),
                M.m<VariableControllerMaster>(_ => step),
                this._focus,
                M.m<VariableControllerMaster>(_ => labelFn()),
                (VariableControllerMaster _, float f) => { /* display only */ },
                (VariableControllerMaster _, float f) => "",
                M.m<VariableControllerMaster>(new ToolTip(tipKo, 280f)),
                Array.Empty<string>()
            );
        }

        /// <summary>
        /// 데이터 수집 모니터링 — 사용자가 수집 상황을 한눈에 볼 수 있게.
        ///   1) Sensitivity bars (Low/Mid/High band) — 어느 band 가 정보 풍부한지
        ///   2) PRBS bit (현재 자극 band)
        ///   3) Saturation rate — target 10-25% 인지
        ///   4) Amplitudes (SP, u-direct)
        ///   5) Time progress + well-tuned count
        /// 학계: input design monitoring (Hjalmarsson 2005) 의 시각적 표현.
        /// </summary>
        private void BuildDataMonitoring()
        {
            ScreenSegmentStandard seg = base.CreateStandardSegment(InsertPosition.OnCursor);
            seg.BackgroundStyleWhereApplicable = ConsoleStyles.Instance.Styles.Segments.OptionalSegmentDarkBackgroundWithHeader.Style;
            seg.NameWhereApplicable = "Data Monitoring";
            seg.SpaceAbove = 10f;
            seg.SpaceBelow = 10f;

            // ── Sensitivity bars (Low / Mid / High) ──
            //   |S(f)| 큼 = controller 가 그 band 못 잡음 = 정보 풍부
            //   |S(f)| 작음 (≈ 0) = controller 완벽 reject = 정보 부족
            // 공통 설명: PRBS bit/band/freq mapping (각 sensitivity bar tooltip 끝에 부착)
            const string bandExplain =
                "\n\nPRBS bit duration ↔ excited band:\n" +
                "  bit=4 (0.1s)  → high band (~5Hz)\n" +
                "  bit=16 (0.4s) → mid band (~1.25Hz)\n" +
                "  bit=64 (1.6s) → low band (~0.3Hz)\n" +
                "bit = ticks one PRBS pulse stays at ±1 before flipping.\n" +
                "Status message shows 'band=low/mid/high' (the band currently excited).\n---\n" +
                "PRBS 비트 길이 ↔ 자극 주파수 매핑.\n" +
                "bit = PRBS 한 비트 (±1) 가 유지되는 틱 수.\n" +
                "Status 메시지의 band=low/mid/high 는 현재 자극 중 band.";

            seg.AddInterpretter(MakeDisplayBar(
                0f, 1.5f, 0.01f,
                () => (float)_sess.LastSLo,
                () => $"|S| Low (0.05-0.5Hz): {_sess.LastSLo:0.000}" +
                      (_sess.PrbsBitTicks == 64 ? " ← exciting" : ""),
                "Sensitivity |S(f)| in low decade (0.05-0.5 Hz, drift / integrator zone).\n" +
                "Larger |S| = controller doesn't reject = information-rich.\n" +
                "Smaller |S| ≈ 0 = perfect tracking = no info.\n" +
                "Currently exciting if band=low.\n---\n" +
                "저주파 decade (drift, 적분기) sensitivity. 크면 정보 풍부." + bandExplain
            ));

            seg.AddInterpretter(MakeDisplayBar(
                0f, 1.5f, 0.01f,
                () => (float)_sess.LastSMid,
                () => $"|S| Mid (0.5-5Hz): {_sess.LastSMid:0.000}" +
                      (_sess.PrbsBitTicks == 8 ? " ← exciting" : ""),
                "Sensitivity in mid decade (0.5-5 Hz, plant dynamics / Kp zone).\n" +
                "Currently exciting if band=mid.\n---\n" +
                "중간 decade (plant dynamics, Kp) sensitivity." + bandExplain
            ));

            seg.AddInterpretter(MakeDisplayBar(
                0f, 1.5f, 0.01f,
                () => (float)_sess.LastSHi,
                () => $"|S| High (5-Nyquist): {_sess.LastSHi:0.000}" +
                      (_sess.PrbsBitTicks == 2 ? " ← exciting" : ""),
                "Sensitivity in high decade (5 Hz - Nyquist, M transition / Td zone).\n" +
                "Nyquist = fs/2 (FTD physics 40Hz → Nyquist 20Hz).\n" +
                "Currently exciting if band=high.\n---\n" +
                "고주파 decade (M transition, Td) sensitivity.\nNyquist = fs/2 (FTD 40Hz → 20Hz)." + bandExplain
            ));

            // ── Coherence γ²(f) — Welch cross-spectrum 의 신뢰도 ──
            //   0 ~ 1 범위. > 0.7: 신뢰 가능, < 0.3: noise dominated.
            //   |S| 값이 작아도 coherence 높으면 *진짜 well-tuned*, 낮으면 *측정 부족*.
            const string cohExplain =
                "\n\nCoherence γ²(f) ∈ [0, 1]:\n" +
                "  γ² close to 1: |S| measurement reliable\n" +
                "  γ² close to 0: noise-dominated, ignore the |S| value\n" +
                "Welch cross-spectrum estimation (Bendat-Piersol 2010).\n---\n" +
                "측정 신뢰도. 1 가까우면 |S| 신뢰 가능, 0 가까우면 noise 위주.";

            seg.AddInterpretter(MakeDisplayBar(
                0f, 1.0f, 0.01f,
                () => (float)_sess.LastCohLo,
                () => $"Coherence Low: {_sess.LastCohLo:0.000}",
                "Coherence in low frequency band. Reliability of |S| Low.\n---\n" +
                "저주파 sensitivity 측정 신뢰도." + cohExplain
            ));

            seg.AddInterpretter(MakeDisplayBar(
                0f, 1.0f, 0.01f,
                () => (float)_sess.LastCohMid,
                () => $"Coherence Mid: {_sess.LastCohMid:0.000}",
                "Coherence in mid frequency band. Reliability of |S| Mid.\n---\n" +
                "중간 주파수 sensitivity 측정 신뢰도." + cohExplain
            ));

            seg.AddInterpretter(MakeDisplayBar(
                0f, 1.0f, 0.01f,
                () => (float)_sess.LastCohHi,
                () => $"Coherence High: {_sess.LastCohHi:0.000}",
                "Coherence in high frequency band. Reliability of |S| High.\n---\n" +
                "고주파 sensitivity 측정 신뢰도." + cohExplain
            ));

            // ── Saturation rate ──
            seg.AddInterpretter(MakeDisplayBar(
                0f, 0.5f, 0.01f,
                () =>
                {
                    int n = _sess.U.Count;
                    return (n > 0) ? (float)((double)_sess.SaturatedCount / n) : 0f;
                },
                () =>
                {
                    int n = _sess.U.Count;
                    double rate = (n > 0) ? (double)_sess.SaturatedCount / n : 0;
                    return $"Saturation rate: {rate:P1} (target 10-25%)";
                },
                "Fraction of saturated samples. Target 10-25% balances:\n" +
                "  too low → weak plant excitation\n" +
                "  too high → nonlinear distortion\n" +
                "Adaptive amp adjusts to hit this range."
            ));

            // ── Amplitudes (SP and u-direct) ──
            seg.AddInterpretter(MakeDisplayBar(
                0f, (float)AMP_DYN_MAX, 0.1f,
                () => (float)_sess.AmpDyn,
                () => $"SP amp: {_sess.AmpDyn:0.000}",
                "SP-direct excitation amplitude (sat-aware adaptive, up to 10)."
            ));

            seg.AddInterpretter(MakeDisplayBar(
                0f, (float)U_AMP_MAX, 0.01f,
                () => (float)_sess.UAmpDyn,
                () => $"u amp: {_sess.UAmpDyn:0.000} (headroom-bounded)",
                "u-direct excitation amplitude. Bounded by γ·(1-|u_C|) per tick (safety)."
            ));

            // ── Collection time progress ──
            seg.AddInterpretter(MakeDisplayBar(
                0f, (float)MAX_COLLECT_SEC, 0.1f,
                () => (float)_sess.T,
                () => $"Collection: {_sess.T:0.0}s / {MAX_COLLECT_SEC:0.0}s (max)",
                "Collection time. Hard timeout at 60s. Early stop if well-tuned detected."
            ));

            // ── Well-tuned consecutive count ──
            seg.AddInterpretter(MakeDisplayBar(
                0f, (float)WELL_TUNED_CONSEC_REQUIRED, 1f,
                () => (float)_sess.WellTunedConsecutiveCount,
                () => $"Well-tuned count: {_sess.WellTunedConsecutiveCount}/{WELL_TUNED_CONSEC_REQUIRED} " +
                      $"(max|S|={_sess.LastMaxSensitivity:0.000}, need < {WELL_TUNED_S_THRESHOLD})",
                "Number of consecutive 6s windows where max|S| < " + WELL_TUNED_S_THRESHOLD + ".\n" +
                "If reaches " + WELL_TUNED_CONSEC_REQUIRED + ", collection stops with 'well-tuned' verdict.\n" +
                "Closed-loop identifiability limit (Forssell-Ljung 1999)."
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

            // t_s : Compute 버튼이 직접 사용. Auto-tune 은 sweep 후 best 값으로 덮어씀.
            table.AddInterpretter(MakeSliderFloat(
                "Settling time t_s (s)",
                "Target settling time used by Compute button (manual).\nAuto-tune sweeps {0.1, 0.3, 1.0, 3.0, 10.0} and writes the best back here.\n---\nCompute 버튼이 직접 사용 (수동).\n자동 튜닝은 {0.1, 0.3, 1.0, 3.0, 10.0} 을 sweep 해서 best 값으로 덮어씀.",
                () => _s.SettlingTimeTs,
                f => _s.SettlingTimeTs = Clamp(f, dtF, 10.0f),
                dtF, 10.0f, dtF, "0.000", "Ts"
            ));

            // n_M : Compute 가 직접 사용, Auto-tune 은 {2,3,4} sweep 후 best 로 덮어씀.
            table.AddInterpretter(MakeSliderInt(
                "Model order n_M",
                "Reference model order. nM=2: plant only. nM=3: plant + actuator lag. nM=4: cascaded.\nUsed by Compute button (manual).\nAuto-tune sweeps {2,3,4} and writes the best back here.\n---\n참조 모델 차수. nM=2: plant 만. nM=3: plant+actuator. nM=4: 다단.\nCompute 버튼이 직접 사용 (수동).\n자동 튜닝은 {2,3,4} sweep 해서 best 로 덮어씀.",
                () => _s.ModelOrderNm,
                v => _s.ModelOrderNm = Math.Max(1, Math.Min(4, v)),
                1, 4, 1, "0", "nM"
            ));

            // tau_M : dt 단위 그리드 (자동 튜닝이 τ = dt 로 세팅하므로 정확히 표시되게).
            table.AddInterpretter(MakeSliderFloat(
                "Delay τ_M (s)",
                "Plant delay (dead-time). 0 = no delay.\nGrid is dt (FTD tick).\nAuto-tuning estimates this automatically.\n---\n플랜트 지연. 0이면 지연 없음.\n그리드 단위는 dt(FTD 틱).\n자동 튜닝 시 자동 추정됩니다.",
                () => _s.ModelDelayTau,
                f => _s.ModelDelayTau = Clamp(f, 0f, 5f),
                0f, 5f, dtF, "0.000", "tau"
            ));

            // Relay amplitude h — Quick Tune (Relay feedback) 의 PID 출력 크기.
            //   relay 가 u = ±h 로 강제 출력. FTD u 범위 [-1, 1] 의 fraction.
            //   클수록 limit cycle 진폭 큼 (SNR 좋음), 작을수록 함체 흔들림 적음.
            //   포화 방지: h < 1 권장.
            table.AddInterpretter(MakeSliderFloat(
                "Relay amplitude h — Quick Tune",
                "Relay output amplitude (replaces PID output during Quick Tune).\nRange [0.05, 1.0] of saturation. Larger = bigger limit cycle, better SNR.\nKeep h < 0.5 to avoid saturation.\n---\nRelay 출력 크기. PID 출력 대체.\n범위 [0.05, 1.0]. 클수록 limit cycle 진폭 크고 SNR 좋음.\n포화 방지 위해 h < 0.5 권장.",
                () => _s.RelayH,
                f => _s.RelayH = Clamp(f, 0.05f, 1.0f),
                0.05f, 1.0f, 0.05f, "0.00", "relayH"
            ));

            // Hysteresis ε — relay 의 switching threshold (노이즈 reject).
            //   0 = pure relay (Åström-Hägglund 1984 기본).
            //   > 0 = hysteresis 추가 (노이즈 환경 권장).
            //   단위 = PV 단위 (y 와 동일).
            table.AddInterpretter(MakeSliderFloat(
                "Hysteresis ε — Quick Tune",
                "Hysteresis for relay switching (noise rejection). 0 = pure relay.\nIncrease if relay chatters (high-freq switching). Unit = PV (y).\n---\nRelay switching hysteresis (노이즈 거부).\n0 = pure relay. relay 가 떨면 키우기. 단위 = PV (y).",
                () => _s.RelayHysteresis,
                f => _s.RelayHysteresis = Clamp(f, 0f, 5.0f),
                0f, 5.0f, 0.01f, "0.00", "hyst"
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

            // Axis type / Fix other axes 토글 제거됨 — cross-coupling 포함 데이터로 실제 환경 식별.

            // Amplitude A / Freq base / Freq max 슬라이더 제거됨.
            //   adaptive 가 sat rate 기반 자동 조정 (SP_amp, u_amp).
            //   PRBS bit_ticks 가 sensitivity 기반 자동 결정.
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
                null!,
                _ => AutoTuneNow()
            ));

            seg.AddInterpretter(new SubjectiveButton<VariableControllerMaster>(
                this._focus,
                M.m<VariableControllerMaster>(_ =>
                    _autoState == AutoTuneState.QtRelayWarmup || _autoState == AutoTuneState.QtRelayMeasure
                        ? "Relay tuning..." : "Quick Tune (Relay)"),
                M.m<VariableControllerMaster>(new ToolTip(
                    "Relay feedback (Åström-Hägglund 1984) + Ziegler-Nichols formula.\n" +
                    "Industry standard #1. PID briefly replaced by ±h relay; limit cycle measured.\n" +
                    "~30s. h amplitude controls vessel oscillation magnitude.\n---\n" +
                    "Relay 피드백 + Ziegler-Nichols 공식. 산업 표준 #1.\n" +
                    "PID 가 잠시 ±h relay 로 교체됨; limit cycle 측정.\n" +
                    "약 30초. h 가 함체 진동 크기 결정.", 300f)),
                null!,
                _ => QuickTuneNow()
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
                    FritExcitationInjector.Clear(this._focus);
                    RelayOutputInjector.Clear(this._focus);
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

                    // 표준오차 표시 (CRLB, NaN 이면 생략).
                    //   pct = SE / |val| × 100. > 50% 이면 "uncertain" 표시.
                    //   학계 가이드: pct < 10% 좋음, 10-30% 보통, > 50% 신뢰 X.
                    string fmtSE(double v, double se, string vFmt) {
                        if (double.IsNaN(se) || double.IsInfinity(se) || v == 0) return v.ToString(vFmt);
                        double pct = 100.0 * se / Math.Abs(v);
                        string flag = pct > 50.0 ? "  [uncertain]" :
                                      pct > 30.0 ? "  [low conf]" : "";
                        return v.ToString(vFmt) + $"  ±{se.ToString(vFmt)}  ({pct:0}%)" + flag;
                    }
                    string header = _sess.QtIsStepResult
                        ? $"── Quick Tune (Relay + Ziegler-Nichols) ──\n" +
                          $"K_c = {_sess.QtKc:0.000} (critical gain)\n" +
                          $"T_c = {_sess.QtTc:0.00}s (critical period)\n"
                        : $"── Single PID (FRIT) ──\n";
                    string result =
                        header +
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
            // Hybrid 가진 초기 amp:
            //   SP_amp = 보수적 0.3 초기값 (adaptive 가 sat rate 기반 자동 조정, 사용자 슬라이더 제거됨)
            //   u_amp = 보수적 0.03 (headroom envelope 안에서 추가 안전)
            _sess.AmpDyn = 0.3;
            _sess.UAmpDyn = 0.03;
            _s.ExciteEnabled = true;
            _sess.LastMessage = "Recording started / 녹화 시작";

            CaptureSetPointAdjustBase();
        }

        private void StopRecording()
        {
            _sess.Recording = false;
            RestoreSetPointAdjustIfNeeded();
            FritExcitationInjector.Clear(this._focus);
            RelayOutputInjector.Clear(this._focus);  // 방어적: relay 활성 중이면 같이 정리

            if (_autoState == AutoTuneState.Recording)
                _autoState = AutoTuneState.Idle;
            _sess.LastMessage = "Recording stopped / 녹화 중지";
        }

        // ============================================================
        // Axis Fixture 기능 제거됨 — cross-coupling 포함 데이터로 실제 환경 식별
        // ============================================================


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

            // SP-direct PRBS 가진. amp 는 StartRecording 의 초기값 (0.3) → adaptive 가 sat rate 보고 조정.
            _s.ExciteEnabled = true;

            _autoState = AutoTuneState.Recording;
            StartRecording();
            _sess.LastMessage = "Recording (PRBS, adaptive amp) / 녹화 중";
        }

        /// <summary>
        /// 녹화 완료 후 다음 틱에 호출. 자동 튜닝 계산 파이프라인:
        /// 1) 전체 (u, y, sat) 데이터 사용 — 임펄스 패턴이라 prelude 분리 불필요
        /// 2) 데이터 품질 체크 (포화율, y 변화량)
        /// 3) |T(jω)| -3dB 점 ω_B → Ts 산출 (Skogestad-Postlethwaite §2.4.5)
        /// 4) nM ∈ {2,3,4} sweep + 9-seed multistart LM → Bazanella H₂ 최소 채택
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
            double[]? r = _sess.R.Count == _sess.U.Count ? _sess.R.ToArray() : null;
            double[]? uInject = _sess.UInject.Count == _sess.U.Count ? _sess.UInject.ToArray() : null;
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
            _sess.LastMessage = $"Data: N={blkLen} sat={_sess.SaturatedCount} u=[{uMin:0.00},{uMax:0.00}] y=[{yMin:0.0},{yMax:0.0}] yStd={yStd:0.000}";

            // τ = dt 고정 (FTD 순수 지연 ≈ 1틱). nM 은 sweep 으로 결정.
            _s.ModelDelayTau = (float)dt;
            _s.CutoffHz = (float)(fs / 8.0);

            // 현재 PID 값을 LM 초기 시드로 사용
            double kp0 = this._focus.Pid.kP.Us;
            double ti0 = this._focus.Pid.kI.Us;
            double td0 = this._focus.Pid.kD.Us;

            // ── FRIT (Fictitious Reference Iterative Tuning) — Soma/Kaneko 2004 ──
            // Model-free PID tuning. (u, y, currentPid) 로 직접 PID 산출.
            //   Cost: full H₂ formulation (Bazanella 2012 §5.3, per-bin integration)
            //         J = Σ γ²(f)·|R(f,θ)|² + Σ (1-γ²(f))·|C(jω,θ)|²·|R(f,θ)|²
            //   LM 최적화, 9-seed multistart (current + 8 grid corners).
            //   Ts 산출: closed-loop bandwidth matching (Skogestad-Postlethwaite §2.4.5)
            //            |T(jω_B)|² = 1/2 → ω_B → Ts = √(2^(1/nM)-1)/(0.2·ω_B)
            //   nM ∈ {2,3,4} 만 sweep (각 nM 에서 Ts 가 자동 매칭됨).
            double tauM = Math.Max(dt, (double)_s.ModelDelayTau);
            double[]? cohPerBin = _sess.LastCohPerBin;

            FritOptResult fr = RunFritBandwidthSweep(
                u, uInject, y, sat, dt, tauM,
                cohPerBin,
                _sess.LastSRR, _sess.LastSYRRe, _sess.LastSYRIm, _sess.LastBinWidthHz,
                kp0, ti0, td0,
                out double tsBest, out int nMBest, out string bwDiag);

            if (double.IsInfinity(fr.Cost) || double.IsNaN(fr.Cost) || tsBest <= 0)
            {
                _autoState = AutoTuneState.Failed;
                string reason = tsBest <= 0
                    ? $"Closed-loop bandwidth: {bwDiag} — 시작 PID 강화 후 재시도 / increase starting PID gain"
                    : "FRIT failed (no valid seed converged) / FRIT 실패";
                _sess.LastMessage = reason;
                return;
            }

            // FTD slider 단위로 반올림
            double kpFinal = Math.Max(0.001, Math.Round(fr.Kp * 1000.0) / 1000.0);
            double tiFinal = Math.Round(fr.Ti * 10.0) / 10.0;
            double tdFinal = Math.Round(fr.Td * 100.0) / 100.0;

            // FTD slider 강제 cap
            if (kpFinal > 1.0) kpFinal = 1.0;
            if (tiFinal > 250.0) tiFinal = 250.0;
            if (tiFinal < 0.1) tiFinal = 0.1;
            if (tdFinal < 0) tdFinal = 0;
            if (tdFinal > 10.0) tdFinal = 10.0;

            _sess.HasResult = true;
            _sess.QtIsStepResult = false;   // FRIT path → Quick Tune 헤더 해제
            _sess.Kp = kpFinal; _sess.Ti = tiFinal; _sess.Td = tdFinal;
            _sess.KpSE = fr.KpSE; _sess.TiSE = fr.TiSE; _sess.TdSE = fr.TdSE;
            _sess.FitRmse = Math.Sqrt(Math.Max(0, fr.Cost));

            // 찾은 best Ts / nM 를 슬라이더에 반영 (다음 Compute 가 같은 값으로 시작 가능)
            _s.SettlingTimeTs = (float)Math.Min(10.0, Math.Max(dt, tsBest));
            _s.ModelOrderNm = nMBest;

            _autoState = AutoTuneState.Done;
            _sess.LastMessage =
                $"Done | FRIT (CL-BW Ts={tsBest:0.000}s, nM={nMBest}; {bwDiag}) → " +
                $"Kp={kpFinal:0.000} Ti={tiFinal:0.0} Td={tdFinal:0.00} " +
                $"(cost={fr.Cost:0.0000}, {fr.Diag})";
        }

        // ════════════════════════════════════════════════════════════════════════
        // Quick Tune — Relay feedback (Åström-Hägglund 1984)
        // ════════════════════════════════════════════════════════════════════════
        //
        // 산업 표준 #1 PID auto-tuning. ABB / Yokogawa / Honeywell 의 base.
        //
        // 단계:
        //   QtDiagnosing  (3s):  limit cycle / 포화 검출 (relay 시작 전, FRIT 와 공유)
        //   QtRelayWarmup (자동): 첫 ~2 주기 = transient → 진폭/주기 측정에서 제외
        //   QtRelayMeasure (자동): 3+ stable 주기 측정 → A, T 평균
        //   compute       : K_c = 4h/(πA), T_c = T → Ziegler-Nichols formula → PID
        //
        // Ziegler-Nichols PID formula (1942, closed-loop method):
        //   Kp = 0.6 · K_c
        //   Ti = 0.5 · T_c
        //   Td = 0.125 · T_c
        //
        // 출처:
        //   - Åström, Hägglund (1984) "Automatic tuning of simple regulators with
        //     specifications on phase and amplitude margins", Automatica 20, 645-651
        //   - Ziegler, Nichols (1942) "Optimum settings for automatic controllers",
        //     ASME Transactions 64, 759-768
        //   - Åström, Hägglund (1995) PID Controllers: Theory, Design, and Tuning
        // ════════════════════════════════════════════════════════════════════════
        private const int    QT_RELAY_WARMUP_CYCLES = 2;   // 첫 2 주기 transient → 무시
        private const int    QT_RELAY_MIN_MEASURE_CYCLES = 3; // 측정용 3 주기 이상
        private const double QT_RELAY_MAX_DUR = 60.0;       // 60초 안에 끝 (안전)
        private const double QT_RELAY_PERIOD_TOL = 0.20;    // 주기 ±20% 안에 안정 판정

        /// <summary>[Quick Tune] 버튼 — Relay feedback 진입점.</summary>
        private void QuickTuneNow()
        {
            if (_autoState != AutoTuneState.Idle && _autoState != AutoTuneState.Done && _autoState != AutoTuneState.Failed)
            {
                _sess.LastMessage = "Tuning already in progress / 튜닝 이미 진행 중";
                return;
            }

            RestoreSetPointAdjustIfNeeded();
            _sess.Clear();
            _sess.QtIsStepResult = true;
            _autoState = AutoTuneState.QtDiagnosing;
            _sess.LastMessage = "Quick Tune (Relay): diagnosing (3s)... / 진단 중 (3초)";
        }

        /// <summary>
        /// Relay feedback test 진행 — 진단 → warm-up → 측정 → 종료.
        /// </summary>
        private void OnQuickTuneTick(double dt)
        {
            if (this._focus == null)
            {
                _autoState = AutoTuneState.Failed;
                return;
            }
            var c = this._focus.GetCurrentController();
            if (c == null)
            {
                // 방어적: relay 활성 중이면 정리
                RelayOutputInjector.Clear(this._focus);
                _autoState = AutoTuneState.Failed;
                _sess.LastMessage = "no controller";
                return;
            }

            double u = c.LastControlVariable;
            double y = c.LastProcessVariable;

            // ── QtDiagnosing: limit cycle / 포화 체크 (3초)
            if (_autoState == AutoTuneState.QtDiagnosing)
            {
                if (_sess.DiagSampleCount == 0) { _sess.DiagUMax = u; _sess.DiagUMin = u; }
                else { if (u > _sess.DiagUMax) _sess.DiagUMax = u; if (u < _sess.DiagUMin) _sess.DiagUMin = u; }
                if (Math.Abs(u) >= _s.SaturationThreshold) _sess.DiagSatCount++;
                if (_sess.DiagSampleCount > 0 && _sess.DiagPrevU != 0 && Math.Sign(u) != Math.Sign(_sess.DiagPrevU))
                    _sess.DiagSignChanges++;
                _sess.DiagPrevU = u;
                _sess.DiagSampleCount++;
                _sess.DiagYBaselineSum += y;
                _sess.DiagYBaselineCnt++;
                _sess.DiagT += dt;
                _sess.LastMessage = $"Quick Tune | Diag {_sess.DiagT:0.0}s/3s";
                if (_sess.DiagT < 3.0) return;

                double satRate = (double)_sess.DiagSatCount / Math.Max(1, _sess.DiagSampleCount);
                double crossRate = _sess.DiagSignChanges / 3.0;
                double uSwing = _sess.DiagUMax - _sess.DiagUMin;
                double uPeak = Math.Max(Math.Abs(_sess.DiagUMax), Math.Abs(_sess.DiagUMin));
                if (satRate > 0.40 && crossRate > 0.5 && uSwing > 1.6)
                {
                    _autoState = AutoTuneState.Failed;
                    _sess.LastMessage = DiagFailLimitCycleMessage(_sess.DiagUMin, _sess.DiagUMax, crossRate, satRate);
                    return;
                }
                if (satRate > 0.40)
                {
                    _autoState = AutoTuneState.Failed;
                    _sess.LastMessage = DiagFailSaturationMessage(satRate, uPeak);
                    return;
                }

                // y baseline = SP for relay reference
                _sess.QtY0 = _sess.DiagYBaselineSum / Math.Max(1, _sess.DiagYBaselineCnt);

                // Warm-up phase 시작 — relay ON
                _autoState = AutoTuneState.QtRelayWarmup;
                _sess.QtT = 0;
                _sess.QtYMin = y;
                _sess.QtYMax = y;
                // 초기 relay 방향 = sign(SP - y)
                double err = _sess.QtY0 - y;
                _sess.QtRelayCurrent = (err >= 0) ? _s.RelayH : -_s.RelayH;
                RelayOutputInjector.Set(this._focus, _sess.QtRelayCurrent);
                _sess.LastMessage = $"Quick Tune | Relay warm-up (h={_s.RelayH:0.00})...";
                return;
            }

            // ── QtRelayWarmup / QtRelayMeasure: relay 제어 + zero-crossing 추적
            if (_autoState == AutoTuneState.QtRelayWarmup || _autoState == AutoTuneState.QtRelayMeasure)
            {
                _sess.QtT += dt;
                if (_sess.QtT > QT_RELAY_MAX_DUR)
                {
                    RelayOutputInjector.Clear(this._focus);
                    _autoState = AutoTuneState.Failed;
                    _sess.LastMessage = $"Relay Tune: timeout (60s). No stable limit cycle.";
                    return;
                }

                // y peak 추적 (현재 반주기 안에서)
                if (y > _sess.QtYMax) _sess.QtYMax = y;
                if (y < _sess.QtYMin) _sess.QtYMin = y;

                // Relay switching 로직 — hysteresis 포함
                double err = _sess.QtY0 - y;
                double eps = _s.RelayHysteresis;
                float prevRelay = _sess.QtRelayCurrent;
                if (_sess.QtRelayCurrent > 0 && err < -eps)
                {
                    // y 가 SP 보다 ε 만큼 위 → relay - 방향으로 (이번 반주기 끝)
                    _sess.QtRelayCurrent = -_s.RelayH;
                }
                else if (_sess.QtRelayCurrent < 0 && err > eps)
                {
                    // y 가 SP 보다 ε 만큼 아래 → relay + 방향으로 (이번 반주기 끝)
                    _sess.QtRelayCurrent = _s.RelayH;
                }
                bool switched = (_sess.QtRelayCurrent != prevRelay);
                // 매 틱 재주입: relay 값은 반주기 동안 일정하지만, 전환 시에만 Set 하면
                // 반주기(보통 >0.2s) 동안 인젝터 TTL(0.2s)이 만료되어 relay 주입이 끊기고
                // PID 가 복귀 → limit cycle 붕괴. 전환 여부와 무관하게 매 틱 Set 해
                // LastTime 을 갱신, TTL 만료를 막는다 (FRIT 가진과 동일한 패턴).
                RelayOutputInjector.Set(this._focus, _sess.QtRelayCurrent);

                // 한 주기 완료 검출: relay + → - → + 사이 (한 full cycle)
                //   즉 prev = +h, current = -h 로 전환 시점부터 다음 + 전환까지가 반주기.
                //   pure measurement 는 + 에서 + 로 돌아오는 시점 (full period).
                //   peak detect: + → - 전환 시 그 사이의 y_max 가 그 주기의 peak
                if (switched && _sess.QtRelayCurrent < 0)
                {
                    // + → - 전환: y_max 측정 완료, peak-to-peak 추정 위해 다음 cycle 까지 기다림
                    // 이 시점을 cycle 시작으로 기록
                    _sess.QtCycleTimes.Add(_sess.QtT);
                    // y_max, y_min 의 현재 값으로 진폭 추정 (다음 cycle 시작 시점에서 리셋)
                    double amp = (_sess.QtYMax - _sess.QtYMin) / 2.0;
                    if (_sess.QtCycleTimes.Count > 1) _sess.QtCycleAmps.Add(amp);
                    _sess.QtYMax = y;
                    _sess.QtYMin = y;
                }

                int cyclesDone = _sess.QtCycleTimes.Count - 1; // 첫 entry 는 시작점

                // Warm-up → Measure 전환
                if (_autoState == AutoTuneState.QtRelayWarmup)
                {
                    _sess.LastMessage = $"Quick Tune | Warm-up: {cyclesDone} cycles (need {QT_RELAY_WARMUP_CYCLES})";
                    if (cyclesDone >= QT_RELAY_WARMUP_CYCLES)
                    {
                        // 측정 phase 진입 — cycle 시간/진폭 누적 초기화 (warm-up 데이터는 버림)
                        _sess.QtCycleTimes.Clear();
                        _sess.QtCycleAmps.Clear();
                        _autoState = AutoTuneState.QtRelayMeasure;
                        _sess.QtT = 0;
                    }
                    return;
                }

                // QtRelayMeasure
                _sess.LastMessage = $"Quick Tune | Measuring: {cyclesDone} stable cycles (need {QT_RELAY_MIN_MEASURE_CYCLES})";

                if (cyclesDone >= QT_RELAY_MIN_MEASURE_CYCLES)
                {
                    // 안정성 체크: 마지막 3 주기의 길이 / 진폭이 일관된가
                    int n = _sess.QtCycleTimes.Count;
                    double[] periods = new double[n - 1];
                    for (int i = 0; i < n - 1; i++) periods[i] = _sess.QtCycleTimes[i + 1] - _sess.QtCycleTimes[i];
                    double pAvg = 0;
                    for (int i = 0; i < periods.Length; i++) pAvg += periods[i];
                    pAvg /= periods.Length;
                    bool stable = true;
                    for (int i = 0; i < periods.Length; i++)
                    {
                        if (Math.Abs(periods[i] - pAvg) / pAvg > QT_RELAY_PERIOD_TOL) { stable = false; break; }
                    }
                    if (!stable) return;   // 더 측정

                    // 측정 완료 — relay 해제 + 계산
                    RelayOutputInjector.Clear(this._focus);
                    try { ComputeRelayTune(); }
                    catch (Exception ex)
                    {
                        _autoState = AutoTuneState.Failed;
                        _sess.LastMessage = "Relay Tune compute failed: " + ex.Message;
                    }
                    return;
                }
            }
        }

        /// <summary>
        /// 측정한 K_c (critical gain), T_c (critical period) 에서 PID 계산 — Ziegler-Nichols 1942 표준.
        ///
        /// 측정 공식 (Åström-Hägglund 1984):
        ///   K_c = 4·h / (π·A)
        ///     h = relay 진폭
        ///     A = limit cycle 진폭 (peak amplitude, y 가 SP 중심으로 ±A 진동)
        ///   T_c = limit cycle 주기 (= ultimate period at -180° phase)
        ///
        /// PID formula (Ziegler-Nichols 1942 closed-loop method):
        ///   Kp = 0.6 · K_c
        ///   Ti = 0.5 · T_c
        ///   Td = 0.125 · T_c
        /// </summary>
        private void ComputeRelayTune()
        {
            int nCycles = _sess.QtCycleAmps.Count;
            int nPeriods = _sess.QtCycleTimes.Count - 1;
            if (nCycles < 1 || nPeriods < 1)
            {
                _autoState = AutoTuneState.Failed;
                _sess.LastMessage = "Relay Tune: insufficient cycles measured.";
                return;
            }

            // 평균 진폭 A, 주기 T_c
            double aAvg = 0;
            for (int i = 0; i < nCycles; i++) aAvg += _sess.QtCycleAmps[i];
            aAvg /= nCycles;

            double tcAvg = 0;
            for (int i = 0; i < nPeriods; i++) tcAvg += _sess.QtCycleTimes[i + 1] - _sess.QtCycleTimes[i];
            tcAvg /= nPeriods;

            if (aAvg < 1e-6 || tcAvg < 1e-3)
            {
                _autoState = AutoTuneState.Failed;
                _sess.LastMessage = $"Relay Tune: A={aAvg:0.0000} or T_c={tcAvg:0.000} too small.";
                return;
            }

            // K_c = 4h/(πA), T_c = measured period
            double kcHat = 4.0 * _s.RelayH / (Math.PI * aAvg);
            double tcHat = tcAvg;

            // Ziegler-Nichols PID
            double kpCalc = 0.6 * kcHat;
            double tiCalc = 0.5 * tcHat;
            double tdCalc = 0.125 * tcHat;

            // 저장 (FOPDT 필드는 사용 안 함 — 결과 패널에서 K_c, T_c 표시)
            _sess.QtKc = kcHat;
            _sess.QtTc = tcHat;

            // FTD slider 단위 반올림 + cap
            double kpFinal = Math.Max(0.001, Math.Round(kpCalc * 1000.0) / 1000.0);
            double tiFinal = Math.Round(tiCalc * 10.0) / 10.0;
            double tdFinal = Math.Round(tdCalc * 100.0) / 100.0;
            if (kpFinal > 1.0) kpFinal = 1.0;
            if (tiFinal > 250.0) tiFinal = 250.0;
            if (tiFinal < 0.1) tiFinal = 0.1;
            if (tdFinal < 0) tdFinal = 0;
            if (tdFinal > 10.0) tdFinal = 10.0;

            _sess.HasResult = true;
            _sess.Kp = kpFinal; _sess.Ti = tiFinal; _sess.Td = tdFinal;
            _sess.KpSE = double.NaN; _sess.TiSE = double.NaN; _sess.TdSE = double.NaN;
            _sess.FitRmse = 0;

            _autoState = AutoTuneState.Done;
            _sess.LastMessage =
                $"Done | Relay+ZN (K_c={kcHat:0.000}, T_c={tcHat:0.00}s, A={aAvg:0.0000}, h={_s.RelayH:0.00}) → " +
                $"Kp={kpFinal:0.000} Ti={tiFinal:0.0} Td={tdFinal:0.00}";
        }

        private void ValidateAxes()
        {
            // Validate 기능: 축 분리 기능 제거됨에 따라 단일 축 측정만 지원.
            if (_autoState == AutoTuneState.Validating)
            {
                _sess.LastMessage = "Validation already in progress / 검증 이미 진행 중";
                return;
            }
            _sess.ValidateY.Clear();
            _sess.ValidateY.Add(new List<double>());  // 단일 축 (focus 만)
            _sess.ValidateStartT = 0;
            _autoState = AutoTuneState.Validating;
            double valDur = GetValidateDuration();
            _sess.LastMessage = $"Validating: collecting y for {valDur:0}s... / 검증: y 수집 중 ({valDur:0}초)...";
        }

        // ════════════════════════════════════════════════════════════════════════
        // OnDiagnoseTick — Auto Tune 직후 사전 진단 (3초, 가진 OFF)
        // ════════════════════════════════════════════════════════════════════════
        //
        // 현재 PID 가 *튜닝 가능 상태* 인지 확인. 판정:
        //   1. Limit cycle: satRate > 40%, crossRate > 0.5/s, uSwing > 1.6 → 실패
        //   2. 지속 포화:   satRate > 40% (진동 적음)                       → 실패
        //   3. 정상: 즉시 Recording 진입
        // 진단 동안 누적한 y 평균 = recording 의 baseline (u-direct safety).
        // ════════════════════════════════════════════════════════════════════════
        private const double DIAG_DUR = 3.0;

        private void OnDiagnoseTick(double dt)
        {
            if (_autoState != AutoTuneState.Diagnosing) return;

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
            double y = c.LastProcessVariable;

            // 가진 OFF — |u| 통계 + y baseline 누적
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

            _sess.DiagYBaselineSum += y;
            _sess.DiagYBaselineCnt++;

            _sess.DiagT += dt;

            double uPeakSoFar = Math.Max(Math.Abs(_sess.DiagUMax), Math.Abs(_sess.DiagUMin));
            _sess.LastMessage = $"Diag... {_sess.DiagT:0.0}s/{DIAG_DUR:0.0}s (uPeak={uPeakSoFar:0.00}) / 진단";

            if (_sess.DiagT < DIAG_DUR) return;

            // 판정
            double satRate = (double)_sess.DiagSatCount / Math.Max(1, _sess.DiagSampleCount);
            double uSwing = _sess.DiagUMax - _sess.DiagUMin;
            double uPeak = Math.Max(Math.Abs(_sess.DiagUMax), Math.Abs(_sess.DiagUMin));
            double crossRate = _sess.DiagSignChanges / DIAG_DUR;

            if (satRate > 0.40 && crossRate > 0.5 && uSwing > 1.6)
            {
                _autoState = AutoTuneState.Failed;
                _sess.LastMessage = DiagFailLimitCycleMessage(_sess.DiagUMin, _sess.DiagUMax, crossRate, satRate);
                return;
            }
            if (satRate > 0.40)
            {
                _autoState = AutoTuneState.Failed;
                _sess.LastMessage = DiagFailSaturationMessage(satRate, uPeak);
                return;
            }

            // 정상 → 즉시 Recording 단계로
            _sess.DiagYBaseline = _sess.DiagYBaselineSum / Math.Max(1, _sess.DiagYBaselineCnt);
            _recordingYBaseline = _sess.DiagYBaseline;
            _sess.LastMessage = $"Diag OK (sat={satRate:P0}). 녹화 시작 / Recording.";
            StartAutoTuneRecording();
        }

        // 진단 실패 메시지 — FRIT/Relay 공용 (bilingual + 세부 수치 통일)
        private static string DiagFailLimitCycleMessage(double uMin, double uMax, double crossRate, double satRate)
        {
            double uSwing = uMax - uMin;
            return $"⚠ Limit cycle (u={uMin:0.00}~{uMax:0.00}, swing={uSwing:0.00}, {crossRate:0.0}/s, sat={satRate:P0})\n" +
                   $"  Kp 과대 / Ti 과소 (windup) / Td 과대 의심 — 게인 낮춰 재시도\n" +
                   $"  Kp too high / Ti too low (windup) / Td too high — reduce gains and retry";
        }

        private static string DiagFailSaturationMessage(double satRate, double uPeak)
        {
            return $"⚠ Persistent saturation / 지속 포화 (sat={satRate:P0}, uPeak={uPeak:0.00})\n" +
                   $"  Kp/Ki 과대 또는 SP 가 액추에이터 한계 초과 의심\n" +
                   $"  Kp/Ki too high or SP exceeds actuator limit";
        }

        private void OnValidateTick(double dt)
        {
            if (_autoState != AutoTuneState.Validating) return;

            _sess.ValidateStartT += dt;

            // Single-axis: focus 의 y 만 수집
            if (_sess.ValidateY.Count > 0)
            {
                var ctrl = this._focus.GetCurrentController();
                double yVal = ctrl != null ? ctrl.LastProcessVariable : 0;
                _sess.ValidateY[0].Add(yVal);
            }

            double valDuration = GetValidateDuration();
            if (_sess.ValidateStartT >= valDuration)
            {
                double std = 0;
                if (_sess.ValidateY.Count > 0 && _sess.ValidateY[0].Count > 1)
                {
                    var ys = _sess.ValidateY[0];
                    double mean = 0;
                    for (int i = 0; i < ys.Count; i++) mean += ys[i];
                    mean /= ys.Count;
                    double variance = 0;
                    for (int i = 0; i < ys.Count; i++) variance += (ys[i] - mean) * (ys[i] - mean);
                    variance /= (ys.Count - 1);
                    std = Math.Sqrt(variance);
                }

                _autoState = AutoTuneState.Done;
                _sess.LastMessage = $"Validate: yStd={std:0.000}";
            }
        }

        /// <summary>검증 수집 시간. 축 타입 식별 없으므로 보수적 10초 고정.</summary>
        private double GetValidateDuration() => 10.0;

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
        /// SP-direct 가진. WaveType.MultiSine 일 때 PRBS (Pseudo-Random Binary Sequence) 사용.
        /// Ljung "System Identification" §13 의 표준 식별 입력. 다른 wave 형식 (Sine/Chirp) 은
        /// 수동 실험용으로 보존.
        ///
        /// PRBS 의 성질 (수학적):
        ///   · 평균 0 (정확) → integrator plant drift 누적 0
        ///   · 진폭 ±A 항상 (피크 팩터 = 1)
        ///   · 자기상관 ≈ impulse (broadband 스펙트럼, 0 ~ 1/(2·T_b) 거의 균일)
        ///   · maximum length sequence (period 2^10 - 1 = 1023 bit)
        ///   · deterministic (같은 seed = 같은 시퀀스 → reproducible)
        ///
        /// 휴리스틱 임의값 없음. 파라미터:
        ///   · 진폭 A — 사용자 슬라이더 (Excite Amp)
        ///   · bit duration T_b — PRBS_BIT_TICKS 상수 (코드)
        /// </summary>
        /// <summary>현재 틱 SP-direct 가진값 (FRIT instrument 용).</summary>
        private double _lastExciteValue = 0.0;
        /// <summary>현재 틱 u-direct 가진값 (FRIT cost 보정용 — patch 가 u 에 더한 값).</summary>
        private double _lastUInject = 0.0;

        // PRBS bit duration: 한 비트가 유지되는 틱 수.
        // dt=0.025 기준 4틱 = 0.1초 → broadband 스펙트럼 0~5Hz (FTD plant 일반 범위 cover).
        private const int PRBS_BIT_TICKS = 4;

        // PRBS LFSR feedback polynomial x^10 + x^7 + 1 → maximum length 1023.
        // bit 9 (MSB) XOR bit 6.
        private const int PRBS_TAP1 = 9;
        private const int PRBS_TAP2 = 6;
        private const int PRBS_MASK = 0x3FF;  // 10-bit

        // Hybrid PRBS adaptive amplitudes — SP-direct main + u-direct supplement.
        //   학계 근거:
        //     SP_amp adaptive: Hjalmarsson 2005 "input design with output constraint"
        //     u_amp headroom: Ljung §13.5 "additive perturbation with safety bound"
        //     Saturation 처리: Söderström-Stoica §8.5
        private const int AMP_ADJUST_INTERVAL = 80;
        // 단계당 변동 한계
        private const double AMP_RATIO_MAX = 1.5;
        private const double AMP_RATIO_MIN = 0.5;
        // SP-direct amp 범위 — 큰 plant (배, 함선) 의 강한 자극 필요 시 cap 까지 증가.
        //   SP > 1 의미: SP 가 actuator 범위 (±1) 보다 큰 step 요청. PID 가 saturation 까지 노력.
        //   학계 (Hjalmarsson 2005): SP perturbation 의 fundamental upper bound 없음.
        //   다만 adaptive logic 이 sat rate 보고 자동 조정 → 정보 효율 max 영역 자동 도달.
        private const double AMP_DYN_MIN = 0.01;
        private const double AMP_DYN_MAX = 10.0;
        // u-direct amp 범위 — headroom-bounded 라 cap 까지 키워도 안전.
        //   매 틱 γ·(1-|u_C|) clamp 적용 → PID 가 saturation 가까우면 자동 0.
        //   PID 가 한가할 때만 큰 amplitude → 안전.
        private const double U_AMP_MIN = 0.005;
        private const double U_AMP_MAX = 1.0;
        // u-direct headroom factor γ — u_amp_k ≤ γ · (1 - |u_C_k|)
        //   γ = 0.5 → u 의 headroom 절반까지 사용 (안전, Hjalmarsson 권장)
        private const double U_HEADROOM_GAMMA = 0.5;
        // Saturation 빈도 기반 — sat rate target ~10-25%
        private const double SAT_RATE_HIGH = 0.30;
        private const double SAT_RATE_TARGET_LOW = 0.10;

        // PRBS HPF coefficient (drift cancellation): fc ≈ 0.01Hz at dt=0.025.
        // α = exp(-2π·fc·dt) ≈ 1 - 2π·0.01·0.025 ≈ 0.9984. PRBS bit (0.1s) 동안 decay 무시 가능.
        private const double PRBS_HPF_ALPHA = 0.9984;

        // Spectral monitor — y/r FFT 로 sensitivity 분석.
        //   FFT length 256 (power of 2) = ~6.4 초 window @ dt=0.025
        //   3 decade band (Bode 1945 전통):
        //     low  [0.05, 0.5] Hz       — drift, 적분기
        //     mid  [0.5, 5]   Hz       — plant dynamics, Kp
        //     high [5, Nyquist] Hz     — M transition, Td (Parseval 정통, 마진 없음)
        //   부족 band (sensitivity 큰 곳) 자극 + max(S) 로 well-tuned 감지.
        private const int SPECTRAL_FFT_LEN = 256;
        private const int SPECTRAL_INTERVAL = 240;  // ~6초 마다 체크

        // Information-driven termination
        //   max(|S|) < ε 가 연속 K windows 만족 → "well-tuned" 종료
        //   ε = 0.1: 모든 band 에서 |T| ∈ [0.9, 1.1] = 90% tracking 이상
        //   K = 3: 약 18초 동안 consistent → noise 가 아닌 진짜 saturation
        //   60초 hard timeout: 그 시간까지 well-tuned 안 되면 정보 충분 → FRIT 실행
        private const double WELL_TUNED_S_THRESHOLD = 0.1;
        private const int WELL_TUNED_CONSEC_REQUIRED = 3;
        private const double MAX_COLLECT_SEC = 60.0;

        /// <summary>
        /// Welch periodogram + Cross-spectrum + Coherence-weighted sensitivity.
        ///
        /// 학계 정통 (Welch 1967, Bendat-Piersol 2010, Ljung 1999 §6.3):
        ///   1. 데이터를 50% overlap segment 로 분할 (각 256 samples)
        ///   2. Hanning window 적용 (spectral leakage 차단)
        ///   3. 각 segment FFT
        ///   4. Power spectrum S_yy, S_rr + Cross-spectrum S_yr 누적 평균
        ///   5. Transfer function: T(f) = S_yr(f) / S_rr(f)  (complex)
        ///   6. Coherence: γ²(f) = |S_yr|² / (S_yy · S_rr) ∈ [0, 1]
        ///   7. Sensitivity: S(f) = |1 - T(f)|
        ///   8. Band-wise: coherence-weighted average
        ///
        /// 핵심 효과:
        ///   - K 개 window 평균 → noise variance 1/K (정확도 ↑)
        ///   - Coherence weighting → low-signal bin (numerical artifact) 자동 차단
        ///   - 신뢰도 (γ²) 자체가 metric → 사용자가 결과 신뢰성 판단 가능
        /// </summary>
        /// <returns>max(|S|) over all bands, 또는 -1 if invalid</returns>
        private double UpdatePrbsBitTicksFromSpectrum()
        {
            int N = _sess.Y.Count;
            if (N < SPECTRAL_FFT_LEN) return -1;
            if (_sess.R.Count < SPECTRAL_FFT_LEN) return -1;

            double dt = Time.fixedDeltaTime;
            if (dt <= 0) dt = 0.025;

            const int SEG_LEN = SPECTRAL_FFT_LEN;
            int step = SEG_LEN / 2;  // 50% overlap (Welch 1967 권장)
            int K = (N - SEG_LEN) / step + 1;
            if (K < 1) return -1;

            // Hanning window — spectral leakage 차단
            var hanning = new double[SEG_LEN];
            for (int i = 0; i < SEG_LEN; i++)
                hanning[i] = 0.5 - 0.5 * Math.Cos(2.0 * Math.PI * i / (SEG_LEN - 1));

            // Cross-spectrum accumulators
            var S_yy = new double[SEG_LEN];
            var S_rr = new double[SEG_LEN];
            var S_yr_re = new double[SEG_LEN];
            var S_yr_im = new double[SEG_LEN];

            var ySamples = new Complex[SEG_LEN];
            var rSamples = new Complex[SEG_LEN];
            int validK = 0;

            for (int k = 0; k < K; k++)
            {
                int start = k * step;
                if (start + SEG_LEN > N) break;

                // Detrend per-segment
                double yMean = 0, rMean = 0;
                for (int i = 0; i < SEG_LEN; i++)
                {
                    yMean += _sess.Y[start + i];
                    rMean += _sess.R[start + i];
                }
                yMean /= SEG_LEN; rMean /= SEG_LEN;

                for (int i = 0; i < SEG_LEN; i++)
                {
                    ySamples[i] = new Complex((_sess.Y[start + i] - yMean) * hanning[i], 0);
                    rSamples[i] = new Complex((_sess.R[start + i] - rMean) * hanning[i], 0);
                }

                try
                {
                    MathNet.Numerics.IntegralTransforms.Fourier.Forward(ySamples);
                    MathNet.Numerics.IntegralTransforms.Fourier.Forward(rSamples);
                }
                catch { continue; }

                // Accumulate cross-spectrum
                for (int i = 0; i < SEG_LEN; i++)
                {
                    double yRe = ySamples[i].Real, yIm = ySamples[i].Imaginary;
                    double rRe = rSamples[i].Real, rIm = rSamples[i].Imaginary;
                    S_yy[i] += yRe * yRe + yIm * yIm;
                    S_rr[i] += rRe * rRe + rIm * rIm;
                    // Y · conj(R) = (yRe + j·yIm) · (rRe - j·rIm)
                    S_yr_re[i] += yRe * rRe + yIm * rIm;
                    S_yr_im[i] += yIm * rRe - yRe * rIm;
                }
                validK++;
            }

            if (validK < 1) return -1;

            // Normalize by K
            for (int i = 0; i < SEG_LEN; i++)
            {
                S_yy[i] /= validK;
                S_rr[i] /= validK;
                S_yr_re[i] /= validK;
                S_yr_im[i] /= validK;
            }

            // Band bin 범위 — decade-spaced (Bode 1945 전통):
            //   Lo  [0.05, 0.5] Hz       — decade 1 (drift, 적분기)
            //   Mid [0.5, 5] Hz          — decade 2 (함체 dynamics)
            //   Hi  [5, Nyquist] Hz      — decade 3 (M transition, noise)
            //   Nyquist 까지 사용: Parseval 정통, 시간영역 MSE 와 수학적 등가.
            //   fs=50 → hi=[5, 25Hz], fs=40 → hi=[5, 20Hz].
            double binWidthHz = 1.0 / (SEG_LEN * dt);
            int loStart = Math.Max(1, (int)Math.Round(0.05 / binWidthHz));
            int loEnd = (int)Math.Round(0.5 / binWidthHz);
            int midEnd = (int)Math.Round(5.0 / binWidthHz);
            int hiEnd = SEG_LEN / 2;  // Nyquist exact

            // Coherence-weighted |S| 각 band
            ComputeBandWelchSensitivity(S_yy, S_rr, S_yr_re, S_yr_im, loStart, loEnd,
                out double sLo, out double cohLo);
            ComputeBandWelchSensitivity(S_yy, S_rr, S_yr_re, S_yr_im, loEnd, midEnd,
                out double sMid, out double cohMid);
            ComputeBandWelchSensitivity(S_yy, S_rr, S_yr_re, S_yr_im, midEnd, hiEnd,
                out double sHi, out double cohHi);

            // UI 저장 (band-aggregated)
            _sess.LastSLo = sLo;
            _sess.LastSMid = sMid;
            _sess.LastSHi = sHi;
            _sess.LastCohLo = cohLo;
            _sess.LastCohMid = cohMid;
            _sess.LastCohHi = cohHi;

            // Per-bin γ²(f_i) — Bazanella full H₂ implementation 용
            //   γ²(f) = |S_yr(f)|² / (S_yy(f) · S_rr(f))
            //   Welch 정통 (Bendat-Piersol 2010 §11.3)
            int halfSeg = SEG_LEN / 2;
            double[] cohPerBin = new double[halfSeg];
            double[] sRRhalf = new double[halfSeg];
            double[] sYRReHalf = new double[halfSeg];
            double[] sYRImHalf = new double[halfSeg];
            for (int i = 0; i < halfSeg; i++)
            {
                sRRhalf[i] = S_rr[i];
                sYRReHalf[i] = S_yr_re[i];
                sYRImHalf[i] = S_yr_im[i];

                double denom = S_yy[i] * S_rr[i];
                if (denom > 1e-30)
                {
                    double numer = S_yr_re[i] * S_yr_re[i] + S_yr_im[i] * S_yr_im[i];
                    double g2 = numer / denom;
                    cohPerBin[i] = Math.Max(0.0, Math.Min(1.0, g2));
                }
                else cohPerBin[i] = 0.0;
            }
            _sess.LastCohPerBin = cohPerBin;

            // Closed-loop bandwidth Ts 추정용 per-bin 스펙트럼 저장
            //   T(jω) = S_yr/S_rr,  ω_B = first ω with |T|²≤0.5
            //   M(s) bandwidth = ω_B 매칭 → Ts(nM)
            _sess.LastSRR = sRRhalf;
            _sess.LastSYRRe = sYRReHalf;
            _sess.LastSYRIm = sYRImHalf;
            _sess.LastBinWidthHz = binWidthHz;

            // 부족 band = sensitivity 가장 큰 곳 (정보 풍부 가능)
            //   bit_ticks = N → PRBS bit rate = fs/N Hz → 자극 spectrum ≤ 그 rate
            //   각 band 의 상한 cover 하도록 N 선택 (fs-independent):
            //     Low  [0.05, 0.5] Hz   → N=64 (rate ≈ fs/64, covers low)
            //     Mid  [0.5, 5]   Hz   → N=8  (rate = fs/8, covers up to 5Hz at fs=40)
            //     High [5, Nyquist] Hz → N=2  (rate = fs/2 = Nyquist, broadband high)
            int newBitTicks;
            if (sLo >= sMid && sLo >= sHi)      newBitTicks = 64;
            else if (sMid >= sHi)               newBitTicks = 8;
            else                                newBitTicks = 2;
            _sess.PrbsBitTicks = newBitTicks;

            return Math.Max(sLo, Math.Max(sMid, sHi));
        }

        /// <summary>
        /// 한 band 의 coherence-weighted |S| 평균.
        ///   각 bin: T(f) = S_yr/S_rr (complex), S(f) = |1-T|, γ²(f) = |S_yr|²/(S_yy·S_rr)
        ///   weighted average: Σ S·γ² / Σ γ²
        /// </summary>
        private static void ComputeBandWelchSensitivity(
            double[] S_yy, double[] S_rr, double[] S_yr_re, double[] S_yr_im,
            int binStart, int binEnd,
            out double sWeighted, out double cohAvg)
        {
            double sumSW = 0;
            double sumW = 0;
            double sumCoh = 0;
            int cnt = 0;

            for (int i = binStart; i < binEnd; i++)
            {
                if (S_yy[i] < 1e-15 || S_rr[i] < 1e-15) continue;

                // |S_yr|²
                double yrMagSq = S_yr_re[i] * S_yr_re[i] + S_yr_im[i] * S_yr_im[i];

                // Coherence γ² = |S_yr|² / (S_yy · S_rr)
                double coh = yrMagSq / (S_yy[i] * S_rr[i]);
                if (coh > 1.0) coh = 1.0;  // numerical safety
                if (coh < 0.0) coh = 0.0;

                // T(f) = S_yr / S_rr (complex). S_rr 은 real positive.
                double tRe = S_yr_re[i] / S_rr[i];
                double tIm = S_yr_im[i] / S_rr[i];

                // S(f) = |1 - T| = sqrt((1-T_re)² + T_im²)
                double oneMinusTRe = 1.0 - tRe;
                double s = Math.Sqrt(oneMinusTRe * oneMinusTRe + tIm * tIm);

                sumSW += s * coh;
                sumW += coh;
                sumCoh += coh;
                cnt++;
            }

            sWeighted = (sumW > 1e-9) ? sumSW / sumW : 0;
            cohAvg = (cnt > 0) ? sumCoh / cnt : 0;
        }

        private void ApplyExcitation(float dt)
        {
            _lastExciteValue = 0.0;
            _lastUInject = 0.0;

            if (!_s.ExciteEnabled || !_hasBaseSetPointAdjust)
            {
                FritExcitationInjector.Clear(this._focus);
                try { this._focus.SetPointAdjust.Us = _baseSetPointAdjust; } catch { }
                return;
            }

            // ── Hybrid PRBS (SP-direct + u-direct) ──
            //   같은 PRBS 신호 + HPF 를 SP 와 u 양쪽 inject:
            //     SP 에 SP_amp · PRBS · HPF   (main, FRIT 의 r 신호)
            //     u 에  u_amp · PRBS · HPF   (보조, controller 우회 plant 자극)
            //
            //   학계 근거:
            //     SP main: FRIT cost 식과 자연 호환 (r̃ = y + 1/C·u_C 가 SP 따라감)
            //     u 보조: Söderström-Stoica §8.5 additive perturbation
            //     headroom: Hjalmarsson 2005 amplitude bound
            //
            //   같은 PRBS 신호 사용 — 단순 + 두 경로 상관 OK (FRIT instrument 으로는 r=SP_inject 사용)

            // ── Saturation-aware adaptive (SP_amp) ──
            _sess.TicksSinceAmpAdjust++;
            int n = _sess.U.Count;
            if (_sess.TicksSinceAmpAdjust >= AMP_ADJUST_INTERVAL && n >= AMP_ADJUST_INTERVAL)
            {
                int start = n - AMP_ADJUST_INTERVAL;
                int satCount = 0;
                for (int i = start; i < n; i++)
                    if (_sess.Saturated[i]) satCount++;
                double satRate = (double)satCount / AMP_ADJUST_INTERVAL;

                double scale;
                if (satRate > SAT_RATE_HIGH)            scale = 0.7;
                else if (satRate < SAT_RATE_TARGET_LOW) scale = 1.4;
                else                                     scale = 1.0;

                _sess.AmpDyn *= scale;
                if (_sess.AmpDyn > AMP_DYN_MAX) _sess.AmpDyn = AMP_DYN_MAX;
                else if (_sess.AmpDyn < AMP_DYN_MIN) _sess.AmpDyn = AMP_DYN_MIN;

                // u_amp 도 같은 sat rate 기반 조정 (보조 가진도 plant 자극 만족)
                _sess.UAmpDyn *= scale;
                if (_sess.UAmpDyn > U_AMP_MAX) _sess.UAmpDyn = U_AMP_MAX;
                else if (_sess.UAmpDyn < U_AMP_MIN) _sess.UAmpDyn = U_AMP_MIN;

                _sess.TicksSinceAmpAdjust = 0;
            }

            // PRBS bit (LFSR) — bit duration 는 sensitivity-aware spectral monitor 가 조정
            _sess.PrbsTicksInBit++;
            if (_sess.PrbsTicksInBit >= _sess.PrbsBitTicks)
            {
                int newBit = ((_sess.PrbsState >> PRBS_TAP1) ^ (_sess.PrbsState >> PRBS_TAP2)) & 1;
                _sess.PrbsState = ((_sess.PrbsState << 1) | newBit) & PRBS_MASK;
                _sess.PrbsCurrentValue = (newBit == 1) ? 1.0 : -1.0;
                _sess.PrbsTicksInBit = 0;
            }

            // HPF (DC drift cancellation) — PRBS 신호 자체에 적용
            double prbsRaw = _sess.PrbsCurrentValue;
            double prbsHpf = PRBS_HPF_ALPHA * (_sess.PrbsHpfOutPrev + prbsRaw - _sess.PrbsHpfInPrev);
            _sess.PrbsHpfInPrev = prbsRaw;
            _sess.PrbsHpfOutPrev = prbsHpf;

            // SP-direct: SP 에 SP_amp · prbsHpf 주입
            double spInject = _sess.AmpDyn * prbsHpf;
            try { this._focus.SetPointAdjust.Us = _baseSetPointAdjust + (float)spInject; } catch { }

            // u-direct: headroom envelope 안에서 u_amp · prbsHpf 주입
            //   현재 u_C 측정: PID 출력 (LastControlVariable, pre-clip)
            //   headroom = 1 - |u_C|, u-direct max = γ · headroom
            //   p = clamp(u_amp · prbsHpf, ±u_amp_max_k)
            var c = this._focus.GetCurrentController();
            double uC = (c != null) ? Math.Max(-1.5, Math.Min(1.5, c.LastControlVariable)) : 0;
            double headroom = Math.Max(0, 1.0 - Math.Abs(uC));
            double uAmpMax = U_HEADROOM_GAMMA * headroom;
            double uInjectRaw = _sess.UAmpDyn * prbsHpf;
            double uInject = (uInjectRaw > uAmpMax) ? uAmpMax
                          : (uInjectRaw < -uAmpMax) ? -uAmpMax
                          : uInjectRaw;
            FritExcitationInjector.Set(this._focus, (float)uInject);

            // FRIT instrument = SP inject (main 신호)
            // u-direct 의 값도 저장 → cost 식에서 (1/C)·(u_actual - u_inject) 계산
            _lastExciteValue = spInject;
            _lastUInject = uInject;
        }

        // ============================================================
        // Compute / Apply
        // ============================================================

        /// <summary>
        /// Compute (FRIT) 버튼 — 슬라이더의 Ts/nM 직접 사용 (수동 모드).
        ///   9-seed multistart + u-direct 보정 + 대역별 coherence 가중.
        ///   Auto-tune 과 달리 Ts/nM sweep 안 함 — 사용자가 슬라이더로 지정한 값 그대로.
        /// </summary>
        private void ComputeNow()
        {
            try
            {
                double dt = Time.fixedDeltaTime;
                if (dt <= 0) dt = 0.02;

                int blkLen = _sess.U.Count;
                if (blkLen < 64)
                {
                    _sess.LastMessage = $"Insufficient samples: {blkLen}. Collect more / 샘플 부족: {blkLen}";
                    return;
                }

                double[] u = _sess.U.ToArray();
                double[] y = _sess.Y.ToArray();
                double[]? uInject = _sess.UInject.Count == _sess.U.Count ? _sess.UInject.ToArray() : null;
                bool[] sat = _sess.Saturated.ToArray();

                double yStd = StdDev(y);
                if (yStd < 1e-6)
                {
                    _sess.LastMessage = "No change in y. / y 변화 없음.";
                    return;
                }

                double tauM = Math.Max(dt, (double)_s.ModelDelayTau);
                double userTs = Math.Max(3.0 * dt, _s.SettlingTimeTs);
                int nM = Math.Max(1, Math.Min(4, _s.ModelOrderNm));
                double[]? cohPerBin = _sess.LastCohPerBin;

                // 현재 PID 시드 (9 시드 중 첫번째)
                double kp0 = this._focus.Pid.kP.Us;
                double ti0 = this._focus.Pid.kI.Us;
                double td0 = this._focus.Pid.kD.Us;

                FritOptResult fr = RunFritMultistart(u, uInject, y, sat, dt, userTs, nM, tauM,
                                                     cohPerBin, kp0, ti0, td0);

                if (double.IsInfinity(fr.Cost) || double.IsNaN(fr.Cost))
                {
                    _sess.LastMessage = "FRIT failed (no valid seed converged) / FRIT 실패";
                    return;
                }

                // FTD slider 단위 반올림 + cap
                double kpFinal = Math.Max(0.001, Math.Round(fr.Kp * 1000.0) / 1000.0);
                double tiFinal = Math.Round(fr.Ti * 10.0) / 10.0;
                double tdFinal = Math.Round(fr.Td * 100.0) / 100.0;
                if (kpFinal > 1.0) kpFinal = 1.0;
                if (tiFinal > 250.0) tiFinal = 250.0;
                if (tiFinal < 0.1) tiFinal = 0.1;
                if (tdFinal < 0) tdFinal = 0;
                if (tdFinal > 10.0) tdFinal = 10.0;

                _sess.HasResult = true;
                _sess.QtIsStepResult = false;   // Compute (FRIT) path → Quick Tune 헤더 해제
                _sess.Kp = kpFinal; _sess.Ti = tiFinal; _sess.Td = tdFinal;
                _sess.KpSE = fr.KpSE; _sess.TiSE = fr.TiSE; _sess.TdSE = fr.TdSE;
                _sess.FitRmse = Math.Sqrt(Math.Max(0, fr.Cost));

                _sess.LastMessage =
                    $"Compute Done | FRIT (9 seeds, manual Ts={userTs:0.000}s, nM={nM}) → " +
                    $"Kp={kpFinal:0.000} Ti={tiFinal:0.0} Td={tdFinal:0.00} " +
                    $"(cost={fr.Cost:0.0000}, {fr.Diag})";
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


        // ============================================================
        // UI helpers (FTD 패턴: new SubjectiveFloatClampedWithBar + M.m)
        // ============================================================

        private SubjectiveButton<VariableControllerMaster> MakeButton(string label, string tip, Action<VariableControllerMaster> onClick)
        {
            return new SubjectiveButton<VariableControllerMaster>(
                this._focus,
                M.m<VariableControllerMaster>(_ => label),
                M.m<VariableControllerMaster>(new ToolTip(tip, 260f)),
                null!,
                onClick
            );
        }

        private SubjectiveToggle<VariableControllerMaster> MakeToggle(string label, string tip, Func<bool> getter, Action<bool> setter, string? tag = null)
        {
            return new SubjectiveToggle<VariableControllerMaster>(
                this._focus,
                M.m<VariableControllerMaster>(_ => label),
                M.m<VariableControllerMaster>(new ToolTip(tip, 260f)),
                (VariableControllerMaster _, bool b) => setter(b),
                null!,
                (VariableControllerMaster _) => getter(),
                tag == null ? Array.Empty<string>() : new[] { tag }
            );
        }

        private SubjectiveButton<VariableControllerMaster> MakeCycleButton(string title, string tip, Func<string> valueText, Action onClick, string? tag = null)
        {
            return new SubjectiveButton<VariableControllerMaster>(
                this._focus,
                M.m<VariableControllerMaster>(_ => $"{title}: {valueText()} (click/클릭)"),
                M.m<VariableControllerMaster>(new ToolTip(tip, 260f)),
                null!,
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
            string? tag = null)
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
            string? tag = null)
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
        //  FRIT (Fictitious Reference Iterative Tuning) — Soma/Kaneko 2004
        //
        //  Plant model-free PID tuning. (u, y, currentPid) 데이터로 직접 PID 산출.
        //
        //  Cost:   J(θ) = Σ (y[k] - ŷ(θ)[k])²
        //          where r̃(θ) = y + C(θ)⁻¹·u  (가상 reference)
        //                ŷ(θ) = M(z)·r̃_d(θ)  (참조 모델 응답, delay 적용)
        //
        //  θ* = argmin J(θ) → 비선형 LM (MathNet). non-convex 이라 multistart 필요.
        //
        //  본 implementation 의 장단점 (정직 평가):
        //  ▣ 장점: model order 가정 없음 → 모든 축 universal (적분기 / 1차 / 2차 plant 다)
        //  ▣ 단점: non-convex cost surface — local minimum 위험. multistart 로 완화.
        //  ▣ 단점: 참조모델 M(s) 의 T_s 선택이 결과 좌우 → closed-loop BW (|T|²=0.5) 로 자동 산출.
        // ════════════════════════════════════════════════════════════════════════

        private struct FritOptResult
        {
            public double Kp, Ti, Td;
            public double KpSE, TiSE, TdSE;     // Cramér-Rao 표준오차 (NaN = 계산 실패)
            public double Cost;
            public double Tracking;             // L-curve sweep: 두 항 분리 저장
            public double RawPenalty;
            public bool Converged;
            public string Diag;
        }

        /// <summary>
        /// 역필터 1/C(z) 의 안정성 (= a₀ z² + a₁ z + a₂ 의 zero 가 단위원 내부).
        /// 불안정하면 e[k] = (1/C(z))·u[k] 가 발산 → LM 후퇴 위해 soft barrier 반환.
        /// </summary>
        private static bool IsInverseCStable(double kp, double ti, double td, double dt)
        {
            if (kp <= 0) return false;
            double tiSafe = (ti > 1e-6 && ti < 1e6) ? ti : 1e9;
            double a0 = 1.0 + dt / tiSafe + td / dt;
            double a1 = -(1.0 + 2.0 * td / dt);
            double a2 = td / dt;
            if (Math.Abs(a0) < 1e-12) return false;
            double disc = a1 * a1 - 4.0 * a0 * a2;
            if (disc >= 0)
            {
                double sq = Math.Sqrt(disc);
                double z1 = (-a1 + sq) / (2.0 * a0);
                double z2 = (-a1 - sq) / (2.0 * a0);
                return Math.Abs(z1) < 1.0 && Math.Abs(z2) < 1.0;
            }
            else
            {
                return (a2 / a0) < 1.0;
            }
        }

        /// <summary>e[k] = 1/C(z) · u[k] — backward Euler PID 의 IIR 역필터.</summary>
        private static double[] InverseCFilter(double[] u, double kp, double ti, double td, double dt)
        {
            int N = u.Length;
            double[] e = new double[N];
            double tiSafe = (ti > 1e-6 && ti < 1e6) ? ti : 1e9;
            double a0 = 1.0 + dt / tiSafe + td / dt;
            double a1 = -(1.0 + 2.0 * td / dt);
            double a2 = td / dt;
            double invKa0 = 1.0 / (kp * a0);
            e[0] = u[0] * invKa0;
            if (N > 1) e[1] = (u[1] - u[0] - kp * a1 * e[0]) * invKa0;
            for (int k = 2; k < N; k++)
                e[k] = (u[k] - u[k - 1] - kp * a1 * e[k - 1] - kp * a2 * e[k - 2]) * invKa0;
            return e;
        }

        /// <summary>
        /// 참조모델 M(z) = e⁻ˢτ_M / (1 + s·a_M)^n_M, a_M = 0.2·Ts.
        /// Tustin 1차 LP n_M 번 캐스케이드 + delay shift.
        /// </summary>
        private static double[] ApplyRefModel(double[] r, double ts, int nM, double tauM, double dt)
        {
            int N = r.Length;
            int delayN = Math.Max(0, (int)Math.Round(tauM / dt));
            double aM = 0.2 * ts;
            double beta0 = 1.0 + 2.0 * aM / dt;
            double beta1 = 1.0 - 2.0 * aM / dt;
            double invBeta0 = 1.0 / beta0;

            double[] x = new double[N];
            Array.Copy(r, x, N);
            for (int stage = 0; stage < nM; stage++)
            {
                double[] yOut = new double[N];
                double xPrev = 0, yPrev = 0;
                for (int k = 0; k < N; k++)
                {
                    yOut[k] = (x[k] + xPrev - beta1 * yPrev) * invBeta0;
                    xPrev = x[k];
                    yPrev = yOut[k];
                }
                x = yOut;
            }
            // delay shift
            double[] yDelayed = new double[N];
            for (int k = 0; k < N; k++)
                yDelayed[k] = (k < delayN) ? 0.0 : x[k - delayN];
            return yDelayed;
        }

        /// <summary>
        /// ═══════════════════════════════════════════════════════════════════════════
        /// FRIT cost — γ²-weighted tracking + unweighted H₂ penalty.
        /// ═══════════════════════════════════════════════════════════════════════════
        ///
        ///   J(θ) = Σ γ²(f_i) · |R(f_i, θ)|² / Σ γ²        (tracking, coherence-weighted)
        ///        + λ · (1/N) · Σ |C(jω_i, θ)|²            (penalty, unweighted H₂ norm)
        ///
        /// 의도된 비대칭 (각 항이 다른 출처에서 정통):
        ///   - tracking γ² weighting: Bendat-Piersol §11.4 신호대잡음비 최적
        ///     → 저주파 (signal band) anchor → Ti 도 fit 받음 → collapse 방지
        ///   - penalty unweighted: Bazanella §5.3.2 본문 그대로
        ///     → Ti/Td 양쪽 동일 weight → 한쪽 폭주 방지
        ///
        /// λ 는 외부 L-curve corner detection 으로 결정 (Hansen 1992).
        /// PID realizability 는 Skogestad Td ≤ Ti/4 사후 cap 으로 별도 처리.
        ///
        /// PID frequency response (closed-form):
        ///   |C(jω)|² = Kp² · (1 + (Td·ω - 1/(Ti·ω))²)
        ///
        /// Residual periodogram (Welch 1967, 50% overlap, Hanning):
        ///   R(f_i) = FFT(window · residual)
        ///
        /// 학계 출처:
        ///   - FRIT base: Soma-Kaneko-Fujii (2004)
        ///   - Welch periodogram: Welch (1967)
        ///   - H₂ controller norm: Bazanella-Campestrini-Eckhard (2012) §5.3.2
        ///   - L-curve: Hansen (1992) SIAM Review
        ///   - Skogestad SIMC realizability: Skogestad (2003) JPC §4.2
        ///   - u-direct 보정: Hjalmarsson (2005)
        /// </summary>
        /// <summary>Returns (tracking, rawPenalty) — L-curve sweep 용. λ 곱은 multistart 에서 적용.</summary>
        private static CostBreakdown FritCostBreakdown(double kp, double ti, double td,
            double[] u, double[]? uInject, double[] y, bool[] sat, double ts, int nM, double tauM, double dt,
            double[]? cohPerBin)
        {
            if (!IsInverseCStable(kp, ti, td, dt)) return CostBreakdown.Invalid;
            int N = u.Length;
            double[] uPid = new double[N];
            if (uInject != null && uInject.Length >= N)
                for (int k = 0; k < N; k++) uPid[k] = u[k] - uInject[k];
            else
                for (int k = 0; k < N; k++) uPid[k] = u[k];

            double[] e = InverseCFilter(uPid, kp, ti, td, dt);
            double[] rTilde = new double[N];
            for (int k = 0; k < N; k++) rTilde[k] = y[k] + e[k];
            double[] yHat = ApplyRefModel(rTilde, ts, nM, tauM, dt);
            int dropEdge = Math.Max(32, (int)Math.Round(2.0 * ts / dt));
            int kStart = dropEdge, kEnd = N - dropEdge;
            if (kEnd <= kStart) return CostBreakdown.Invalid;

            // residual 추출 (sat 제외)
            var residList = new List<double>(kEnd - kStart);
            for (int k = kStart; k < kEnd; k++)
            {
                if (sat.Length > k && sat[k]) continue;
                double rr = y[k] - yHat[k];
                if (double.IsNaN(rr) || double.IsInfinity(rr)) return CostBreakdown.Invalid;
                residList.Add(rr);
            }
            int M = residList.Count;
            if (M < 32) return CostBreakdown.Invalid;

            // ─────────────────────────────────────────────────────────────────────
            // Welch periodogram on residual (Welch 1967)
            //   SEG=256, 50% overlap, Hanning window
            //   각 segment FFT → |R(f_i)|² accumulator
            // ─────────────────────────────────────────────────────────────────────
            const int SEG = 256;
            const int HALF_SEG = SEG / 2;
            int step = SEG / 2;
            int Kseg = (M - SEG) / step + 1;
            if (Kseg < 1)
            {
                // 데이터 너무 짧음 — Parseval 등가 (시간영역 MSE), penalty=0 (정보 없음)
                double sumU = 0;
                for (int i = 0; i < M; i++) sumU += residList[i] * residList[i];
                return new CostBreakdown(sumU / M, 0.0);
            }

            // Hanning window
            var hanning = new double[SEG];
            for (int i = 0; i < SEG; i++)
                hanning[i] = 0.5 - 0.5 * Math.Cos(2.0 * Math.PI * i / (SEG - 1));

            // Per-bin residual power accumulator (Welch averaging)
            double[] residPower = new double[HALF_SEG];
            var resSamples = new Complex[SEG];
            int validSeg = 0;

            for (int sIdx = 0; sIdx < Kseg; sIdx++)
            {
                int startIdx = sIdx * step;
                if (startIdx + SEG > M) break;
                double segMean = 0;
                for (int i = 0; i < SEG; i++) segMean += residList[startIdx + i];
                segMean /= SEG;
                for (int i = 0; i < SEG; i++)
                    resSamples[i] = new Complex((residList[startIdx + i] - segMean) * hanning[i], 0);
                try { MathNet.Numerics.IntegralTransforms.Fourier.Forward(resSamples); }
                catch { continue; }
                for (int i = 0; i < HALF_SEG; i++)
                {
                    double mag = resSamples[i].Magnitude;
                    residPower[i] += mag * mag;
                }
                validSeg++;
            }
            if (validSeg < 1) return CostBreakdown.Invalid;
            for (int i = 0; i < HALF_SEG; i++) residPower[i] /= validSeg;

            // ─────────────────────────────────────────────────────────────────────
            // 분석 대역: [0.05 Hz, Nyquist] — 사용 가능한 bin 결정
            // ─────────────────────────────────────────────────────────────────────
            double binWidthHz = 1.0 / (SEG * dt);
            int loStart = Math.Max(1, (int)Math.Round(0.05 / binWidthHz));
            int hiEnd = HALF_SEG;  // Nyquist

            // ─────────────────────────────────────────────────────────────────────
            // Bin-by-bin integration — γ²-weighted tracking + unweighted H₂ penalty
            //
            //   J(θ) = Σ γ²(f_i) · |R(f_i, θ)|² / Σ γ²        (tracking, low-freq anchor)
            //        + λ · (1/N) · Σ |C(jω_i, θ)|²            (penalty, symmetric H₂)
            //
            // 의도된 비대칭:
            //   - γ² weighting on tracking (Bendat-Piersol §11.4): 신호대잡음비 최적 가중
            //     → 저주파 (γ²≈1, signal band) anchor → LM 이 Ti 도 fit 시킴 (Ti collapse 방지)
            //   - unweighted |C|² penalty (Bazanella §5.3.2): Ti/Td 양쪽 동일 weight
            //     → 어느 한쪽 폭주 방지 (이전 (1-γ²) 의 Td 과보호 문제 해소)
            //
            // 시행착오 기록:
            //   v1. γ²/(1-γ²) hybrid → Td drift (penalty 가 고주파 편향)
            //   v2. + ω² weighting (F2) → Td 과억제, Ti collapse 동시
            //   v3. γ² 도 제거 (full unweighted) → Ti collapse 더 심함 (anchor 잃음)
            //   v4 (current). γ² tracking only + unweighted penalty → balance
            //
            // λ 는 외부 L-curve corner detection 결정.
            // PID realizability 는 Skogestad Td ≤ Ti/4 사후 cap 으로 별도.
            // ─────────────────────────────────────────────────────────────────────
            bool useCoh = cohPerBin != null && cohPerBin.Length >= HALF_SEG;
            if (useCoh)
            {
                double cohSum = 0;
                for (int i = loStart; i < hiEnd; i++) cohSum += cohPerBin![i];
                if (cohSum < 1e-9) useCoh = false;
            }

            int nBins = hiEnd - loStart;
            double tracking = 0;
            double rawPenalty = 0;
            double cohWeightSum = 0;

            for (int i = loStart; i < hiEnd; i++)
            {
                double omega = 2.0 * Math.PI * i * binWidthHz;
                double cSq = ComputeCMagSquared(kp, ti, td, omega);
                double rSq = residPower[i];

                if (useCoh)
                {
                    double gammaSq = cohPerBin![i];
                    tracking += gammaSq * rSq;
                    cohWeightSum += gammaSq;
                }
                else
                {
                    tracking += rSq;   // γ² 정보 없음 → unweighted Parseval
                }
                rawPenalty += cSq;
            }

            if (useCoh)
            {
                if (cohWeightSum > 1e-9) tracking /= cohWeightSum;
            }
            else if (nBins > 0)
            {
                tracking /= nBins;
            }
            if (nBins > 0) rawPenalty /= nBins;

            return new CostBreakdown(tracking, rawPenalty);
        }

        /// <summary>
        /// Tracking 과 raw penalty 분리 — L-curve corner detection 에 필요.
        /// final cost = tracking + λ · rawPenalty.
        /// </summary>
        private readonly struct CostBreakdown
        {
            public readonly double Tracking;
            public readonly double RawPenalty;
            public bool IsInvalid => double.IsNaN(Tracking) || double.IsNaN(RawPenalty);
            public static CostBreakdown Invalid => new CostBreakdown(double.NaN, double.NaN);
            public CostBreakdown(double t, double p) { Tracking = t; RawPenalty = p; }
        }

        /// <summary>
        /// |C(jω)|² 계산 — PID controller 의 주파수 응답 magnitude squared.
        ///   C(jω) = Kp · (1 + 1/(Ti·jω) + Td·jω)
        ///   |C(jω)|² = Kp² · (1 + (Td·ω - 1/(Ti·ω))²)
        /// Bazanella penalty 의 noise sensitivity term 에 사용.
        /// </summary>
        private static double ComputeCMagSquared(double kp, double ti, double td, double omega)
        {
            if (omega <= 0) return double.PositiveInfinity;
            double imagPart = td * omega - 1.0 / (ti * omega);
            return kp * kp * (1.0 + imagPart * imagPart);
        }

        /// <summary>
        /// FRIT LM 1회 — 주어진 시드에서 LM 최적화. 비용/수렴 여부 반환.
        /// 포화 샘플은 LM residual 에서 mask.
        /// uInject 가 null 아니면 u-direct 가진 보정 (e = (1/C)·(u_actual - u_inject)).
        /// 수렴 후 Cramér-Rao 표준오차 (KpSE, TiSE, TdSE) 도 계산.
        /// </summary>
        private static FritOptResult RunFritLM(
            double[] u, double[]? uInject, double[] y, bool[] sat, double dt, double ts, int nM, double tauM,
            double[]? cohPerBin,
            double kpInit, double tiInit, double tdInit,
            double lambda = 0.0)
        {
            FritOptResult r = new FritOptResult
            {
                Kp = kpInit, Ti = tiInit, Td = tdInit,
                KpSE = double.NaN, TiSE = double.NaN, TdSE = double.NaN,
                Cost = double.NaN   // 시드 실패 시 NaN — multistart 에서 skip
            };
            int N = u.Length;

            // u_PID = u_actual - u_inject (1/C 에 들어갈 신호 — patch 부분 제거)
            double[] uPid = new double[N];
            if (uInject != null && uInject.Length >= N)
                for (int k = 0; k < N; k++) uPid[k] = u[k] - uInject[k];
            else
                for (int k = 0; k < N; k++) uPid[k] = u[k];

            int dropEdge = Math.Max(32, (int)Math.Round(2.0 * ts / dt));
            int kStart = dropEdge, kEnd = N - dropEdge;
            if (kEnd <= kStart) { r.Diag = "data too short"; return r; }

            var validIdx = new List<int>(kEnd - kStart);
            for (int k = kStart; k < kEnd; k++)
            {
                if (sat.Length > k && sat[k]) continue;
                validIdx.Add(k);
            }
            int M = validIdx.Count;
            if (M < 32) { r.Diag = $"too few non-sat samples ({M})"; return r; }

            // ─── Tikhonov-regularized LM: augmented residual vector ────────────
            //   r_aug = [ y_k - yHat(θ)_k                  ] (time-domain tracking)
            //           [ √(λ) · |C(jω_i, θ)| · √(1/N_pen) ] (freq-domain controller norm)
            //   LM 의 ‖r_aug‖² = MSE_tracking + λ · Σ |C|² / N_pen  (Bazanella §5.3.2 정통)
            //   γ²/ω weighting 제거 — 이전 hybrid form 의 비대칭 문제 해소.
            //   λ=0 이면 augmented rows 0 → 기존 동작과 동일.
            //   출처: Hansen 1992 (Tikhonov via augmented LSQ), Bazanella §5.3.2 (pure H₂)
            // ───────────────────────────────────────────────────────────────────
            const int SEG = 256;
            const int HALF_SEG = SEG / 2;
            double binWidthHz = 1.0 / (SEG * dt);
            bool useReg = lambda > 0;
            int penaltyRows = useReg ? HALF_SEG - 1 : 0;   // bin 1..127 (DC 제외)
            int totalRows = M + penaltyRows;

            // tracking row scale: 시간영역 sample 수 의 inverse → MSE 단위
            //   penalty row 와 magnitude 비교 가능하도록 √(1/M) 가중
            double trackScale = 1.0 / Math.Sqrt(M);

            // penalty row scale: Welch 적분의 √(λ·(1-γ²(ω_i))) · √(1/(HALF_SEG-1))
            //   √(1/penaltyRows) 로 평균 의미 유지 → λ 가 순수 ratio
            double penaltyNorm = penaltyRows > 0 ? 1.0 / Math.Sqrt(penaltyRows) : 1.0;

            var obsX = VB.Dense(totalRows, 0.0);
            var yObsArr = new double[totalRows];
            for (int i = 0; i < M; i++) yObsArr[i] = y[validIdx[i]] * trackScale;
            // penalty rows: target = 0 (penalty 작을수록 좋음)
            var obsY = VB.DenseOfArray(yObsArr);

            Func<MathNet.Numerics.LinearAlgebra.Vector<double>,
                 MathNet.Numerics.LinearAlgebra.Vector<double>,
                 MathNet.Numerics.LinearAlgebra.Vector<double>> model = (theta, xUnused) =>
            {
                double kp = Math.Max(1e-4, theta[0]);
                double ti = Math.Max(1e-3, Math.Min(1e6, theta[1]));
                double td = Math.Max(0, Math.Min(100, theta[2]));

                // 1/C 불안정 영역 — feasibility 위반 → LM 평가 실패 (heuristic penalty X)
                if (!IsInverseCStable(kp, ti, td, dt))
                    throw new ArithmeticException("infeasible: unstable 1/C");

                double[] e = InverseCFilter(uPid, kp, ti, td, dt);
                double[] rTilde = new double[N];
                for (int k = 0; k < N; k++) rTilde[k] = y[k] + e[k];
                double[] yHat = ApplyRefModel(rTilde, ts, nM, tauM, dt);
                var result = VB.Dense(totalRows);
                for (int i = 0; i < M; i++)
                {
                    double yh = yHat[validIdx[i]];
                    if (double.IsNaN(yh) || double.IsInfinity(yh))
                        throw new ArithmeticException("non-finite yHat");
                    result[i] = yh * trackScale;
                }
                // 주파수영역 penalty rows (Bazanella §5.3.2 pure H₂)
                //   residual_i = √(λ·|C|²/N_pen) = √λ · |C| / √N_pen
                if (useReg)
                {
                    double sqrtLam = Math.Sqrt(lambda) * penaltyNorm;
                    for (int i = 1; i < HALF_SEG; i++)
                    {
                        double omega = 2.0 * Math.PI * i * binWidthHz;
                        double cSq = ComputeCMagSquared(kp, ti, td, omega);
                        result[M + (i - 1)] = sqrtLam * Math.Sqrt(cSq);
                    }
                }
                return result;
            };

            var thetaInit = VB.DenseOfArray(new double[] { kpInit, tiInit, tdInit });
            try
            {
                var objective = MathNet.Numerics.Optimization.ObjectiveFunction.NonlinearModel(model, obsX, obsY);
                var lm = new MathNet.Numerics.Optimization.LevenbergMarquardtMinimizer(maximumIterations: 30);
                var lmResult = lm.FindMinimum(objective, thetaInit);
                var opt = lmResult.MinimizingPoint;
                r.Kp = Math.Max(1e-4, opt[0]);
                r.Ti = Math.Max(0.1, Math.Min(250, opt[1]));
                r.Td = Math.Max(0, Math.Min(10, opt[2]));
                var bd = FritCostBreakdown(r.Kp, r.Ti, r.Td, u, uInject, y, sat, ts, nM, tauM, dt, cohPerBin);
                r.Tracking = bd.Tracking;
                r.RawPenalty = bd.RawPenalty;
                r.Cost = bd.IsInvalid ? double.NaN : bd.Tracking;  // 초기 cost = tracking only (λ=0); L-curve 후 갱신
                r.Converged = (lmResult.ReasonForExit == MathNet.Numerics.Optimization.ExitCondition.Converged);
                r.Diag = r.Converged ? "converged" : "stopped";

                // Cramér-Rao SE — 수렴 후 Jacobian (finite difference) 로 계산
                try
                {
                    ComputeFritSE(model, r.Kp, r.Ti, r.Td, obsY, M,
                                  out double kpSE, out double tiSE, out double tdSE);
                    r.KpSE = kpSE;
                    r.TiSE = tiSE;
                    r.TdSE = tdSE;
                }
                catch { /* SE 계산 실패 시 NaN 유지 */ }
            }
            catch (Exception ex)
            {
                r.Diag = "LM ex: " + ex.Message;
            }
            return r;
        }

        /// <summary>
        /// Cramér-Rao 표준오차 (SE) 계산. 학계 정통.
        ///   cov(θ̂) ≈ σ²·(JᵀJ)⁻¹ (Gaussian noise 가정)
        ///   σ² = Σr²/(M-p), p=3 (Kp,Ti,Td)
        ///   J = ∂yHat/∂θ 를 finite difference 로 추정 (3 파라미터 → 6 model 평가)
        ///   SE_i = √(cov_ii)
        /// 해석: Kp = 0.5 ± 0.05 → 95% CI ≈ [0.4, 0.6]. SE / |val| > 0.5 면 신뢰 X.
        /// </summary>
        private static void ComputeFritSE(
            Func<MathNet.Numerics.LinearAlgebra.Vector<double>,
                 MathNet.Numerics.LinearAlgebra.Vector<double>,
                 MathNet.Numerics.LinearAlgebra.Vector<double>> model,
            double kp, double ti, double td,
            MathNet.Numerics.LinearAlgebra.Vector<double> obsY,
            int M,
            out double kpSE, out double tiSE, out double tdSE)
        {
            kpSE = tiSE = tdSE = double.NaN;
            if (M <= 4) return;

            var thetaStar = VB.DenseOfArray(new double[] { kp, ti, td });
            var dummy = VB.Dense(M, 0.0);

            var yStar = model(thetaStar, dummy);

            // Residual + σ² 추정
            double ssr = 0;
            for (int i = 0; i < M; i++)
            {
                double e = obsY[i] - yStar[i];
                ssr += e * e;
            }
            int dof = Math.Max(1, M - 3);
            double sigma2 = ssr / dof;
            if (sigma2 <= 0 || double.IsNaN(sigma2) || double.IsInfinity(sigma2)) return;

            // Jacobian (central finite difference)
            var J = MB.Dense(M, 3);
            double[] vals = new double[] { kp, ti, td };
            double[] deltas = new double[] {
                Math.Max(1e-5, 1e-3 * Math.Abs(kp)),
                Math.Max(1e-5, 1e-3 * Math.Abs(ti)),
                Math.Max(1e-5, 1e-3 * Math.Abs(td) + 1e-5)
            };

            for (int j = 0; j < 3; j++)
            {
                double[] tp = (double[])vals.Clone();
                double[] tm = (double[])vals.Clone();
                tp[j] += deltas[j];
                tm[j] -= deltas[j];
                var yp = model(VB.DenseOfArray(tp), dummy);
                var ym = model(VB.DenseOfArray(tm), dummy);
                double inv2d = 1.0 / (2.0 * deltas[j]);
                for (int i = 0; i < M; i++)
                    J[i, j] = (yp[i] - ym[i]) * inv2d;
            }

            // cov = σ²·(JᵀJ)⁻¹
            try
            {
                var JtJ = J.TransposeThisAndMultiply(J);
                var inv = JtJ.Inverse();
                double v0 = sigma2 * inv[0, 0];
                double v1 = sigma2 * inv[1, 1];
                double v2 = sigma2 * inv[2, 2];
                if (v0 > 0) kpSE = Math.Sqrt(v0);
                if (v1 > 0) tiSE = Math.Sqrt(v1);
                if (v2 > 0) tdSE = Math.Sqrt(v2);
            }
            catch { /* singular JᵀJ → SE 계산 불가, NaN 유지 */ }
        }

        /// <summary>
        /// FRIT multistart — 현재 PID + 보수적 + 적분기/1차 가정 시드. 가장 cost 낮은 결과 채택.
        /// non-convex cost surface 의 local minimum 위험 완화.
        /// </summary>
        /// <summary>
        /// 9-seed multistart LM + Skogestad PID realizability cap.
        ///
        /// 절차:
        ///   1. 9 seeds: 현재 PID + 8 grid corners (Kp{0.05,0.5}×Ti{1,10}×Td{0,1})
        ///   2. 각 seed LM 풀이 (penalty 없음, tracking only — Bazanella γ²-weighted residual)
        ///   3. 가장 낮은 cost 의 seed 채택
        ///   4. Skogestad realizability cap: Td ≤ Ti/4 (Skogestad 2003 §4.2)
        ///      → Td 가 Ti/4 넘으면 controller zero 가 unstable region → noise 증폭
        ///      → derivative filtering 산업 표준
        ///
        /// 학계 출처:
        ///   - Multistart LM: 표준 (Moré 1978)
        ///   - Skogestad SIMC cap: Skogestad (2003) JPC §4.2
        ///
        /// 시행착오 기록 (L-curve 폐기 이유):
        ///   - L-curve 시도: PRBS-excited FRIT 데이터에서 log T 변동(0.24) vs log P (5.16)
        ///     비율이 21:1 → 거의 수직 직선 → corner detection 이 boundary 로 끌림
        ///     → 과도한 λ (e.g., 100) → PI 구조 collapse (Ti=clamp, Td=0)
        ///   - L-curve 의 전제 (clean tracking-penalty trade-off) 우리 cost surface 에 안 맞음
        ///   - 단순화: penalty 없는 순수 FRIT + Skogestad cap 만 남김
        /// </summary>
        private static FritOptResult RunFritMultistart(
            double[] u, double[]? uInject, double[] y, bool[] sat, double dt, double ts, int nM, double tauM,
            double[]? cohPerBin,
            double kpCurr, double tiCurr, double tdCurr)
        {
            double[] kpGrid = { 0.05, 0.5 };
            double[] tiGrid = { 1.0, 10.0 };
            double[] tdGrid = { 0.0, 1.0 };

            var seeds = new List<(double kp, double ti, double td)>(9);
            seeds.Add((Math.Max(1e-3, kpCurr), Math.Max(0.1, tiCurr), Math.Max(0, tdCurr)));
            foreach (var kp in kpGrid)
                foreach (var ti in tiGrid)
                    foreach (var td in tdGrid)
                        seeds.Add((kp, ti, td));

            FritOptResult best = new FritOptResult { Cost = double.PositiveInfinity };
            foreach (var s in seeds)
            {
                var r = RunFritLM(u, uInject, y, sat, dt, ts, nM, tauM, cohPerBin, s.kp, s.ti, s.td);
                if (double.IsNaN(r.Cost) || double.IsInfinity(r.Cost)) continue;
                if (r.Cost < best.Cost) best = r;
            }
            if (double.IsInfinity(best.Cost)) return best;

            // Td 두 정통 cap 결합:
            //   1) Skogestad SIMC realizability: Td ≤ Ti/4  (Skogestad 2003 §4.2)
            //      → controller zero 가 noise 증폭 영역 들어가는 것 방지
            //   2) Closed-loop timescale: Td ≤ 1/ω_B  (Skogestad-Postlethwaite §2.4.5)
            //      → D 작용이 닫힌루프 timescale 안에서만 의미 — 그 이상은 over-reach
            //      → ω_B 는 우리가 이미 추정한 폐루프 대역폭
            //      → Ts = √(2^(1/nM)-1)/(0.2·ω_B) 역산: ω_B = √(2^(1/nM)-1)/(0.2·Ts)
            //
            // 둘 중 더 tight 한 쪽이 효력:
            //   - Ti 가 정상 (0.5~10): Ti/4 dominant (Relay→FRIT case)
            //   - Ti 가 매우 큼 (≥100, I-off): 1/ω_B dominant (FRIT-only case)
            double omegaB = Math.Sqrt(Math.Pow(2.0, 1.0 / nM) - 1.0) / (0.2 * ts);
            double tdMaxSkogestad = best.Ti * 0.25;
            double tdMaxBandwidth = 1.0 / omegaB;
            double tdMax = Math.Min(tdMaxSkogestad, tdMaxBandwidth);

            string capNote = "";
            if (best.Ti > 0 && best.Td > tdMax)
            {
                string src = tdMaxBandwidth < tdMaxSkogestad ? "1/ω_B" : "Ti/4";
                capNote = $"Td cap ({best.Td:0.00}→{tdMax:0.00} by {src}), ";
                best.Td = tdMax;
            }
            best.Diag = capNote + best.Diag;
            return best;
        }

        /// <summary>
        /// Closed-loop bandwidth 기반 Ts 추정 (Bartlett 3-bin band 평균).
        ///
        /// 정확한 수학적 절차:
        ///   1. 폐루프 전달:    T(jω) = S_yr(jω)/S_rr(jω)
        ///                      (Bendat-Piersol 2010 §6.4, H₁ cross-spectrum estimator)
        ///   2. Bartlett 평균:  |T̄(jω_i)|² = (|T(ω_{i-1})|² + |T(ω_i)|² + |T(ω_{i+1})|²) / 3
        ///                      (Bendat-Piersol §11.5, 3-bin band averaging — variance 1/3)
        ///                      "3" = 최소 non-trivial 중앙 평균 (current + 양쪽 이웃)
        ///   3. ω_B 정의:       \|T̄(jω_B)\|² = 1/2
        ///                      (Skogestad-Postlethwaite §2.4.5, closed-loop bandwidth)
        ///   4. M(s) 매칭:      \|M(jω_B)\|² = 1/(1+(0.2·Ts·ω_B)²)^nM = 1/2
        ///                      → Ts = √(2^(1/nM) - 1) / (0.2·ω_B)
        ///
        /// 전제: 적분 작용 PID, \|T\| 단조 감소, ω_B ∈ [ω_1, ω_Nyquist].
        /// 위반 시 ω_B 미발견 → -1 반환.
        /// 수치 안정성: IEEE 754 자동 (S_rr = 0 → \|T\|² = NaN → 평균에서 자동 제외).
        /// </summary>
        private static double EstimateTsFromClosedLoopBandwidth(
            double[]? S_rr, double[]? S_yr_re, double[]? S_yr_im,
            double binWidthHz, int nM,
            out string diag)
        {
            diag = "";
            if (S_rr == null || S_yr_re == null || S_yr_im == null || binWidthHz <= 0)
            {
                diag = "no spectrum";
                return -1;
            }
            int halfSeg = S_rr.Length;
            if (halfSeg < 3) { diag = "spectrum too short"; return -1; }

            // Raw |T(jω)|² per bin (IEEE 754: 0/0 = NaN, finite/0 = ∞)
            double[] tMagSqRaw = new double[halfSeg];
            for (int i = 0; i < halfSeg; i++)
            {
                double sRR = S_rr[i];
                tMagSqRaw[i] = (S_yr_re[i] * S_yr_re[i] + S_yr_im[i] * S_yr_im[i]) / (sRR * sRR);
            }

            // ω_B: Bartlett 3-bin 평균된 |T̄|² 가 0.5 이하로 떨어지는 첫 bin
            //   IsFiniteAndPositive 가 NaN/∞ 자동 제외 → degenerate bin 평균에서 skip
            int crossBin = -1;
            for (int i = 1; i < halfSeg - 1; i++)
            {
                double sum = 0; int cnt = 0;
                for (int j = i - 1; j <= i + 1; j++)
                {
                    if (IsFiniteAndPositive(tMagSqRaw[j])) { sum += tMagSqRaw[j]; cnt++; }
                }
                if (cnt == 0) continue;
                double tBarSq = sum / cnt;
                if (tBarSq <= 0.5) { crossBin = i; break; }
            }
            if (crossBin < 0) { diag = "no |T̄|²=0.5 crossing in band"; return -1; }

            double omegaB = 2.0 * Math.PI * crossBin * binWidthHz;
            double k = Math.Sqrt(Math.Pow(2.0, 1.0 / nM) - 1.0);
            double ts = k / (0.2 * omegaB);

            diag = $"ω_B={omegaB:0.000}rad/s (bin {crossBin}, 3-bin avg)";
            return ts;
        }

        private static bool IsFiniteAndPositive(double v)
            => !double.IsNaN(v) && !double.IsInfinity(v) && v > 0.0;

        /// <summary>
        /// nM ∈ {2,3,4} sweep (Ts 는 nM 별로 closed-loop bandwidth 에서 산출).
        ///   nM = 2 plant only, 3 = plant + actuator lag, 4 = cascaded.
        ///   각 nM 에서 Ts(nM) = √(2^(1/nM)-1)/(0.2·ω_B) → M(s) bandwidth = ω_B 일정.
        ///   Bazanella per-bin H₂ cost 최저 nM 채택.
        /// </summary>
        private static FritOptResult RunFritBandwidthSweep(
            double[] u, double[]? uInject, double[] y, bool[] sat, double dt, double tauM,
            double[]? cohPerBin,
            double[]? S_rr, double[]? S_yr_re, double[]? S_yr_im, double binWidthHz,
            double kpCurr, double tiCurr, double tdCurr,
            out double tsBest, out int nMBest, out string bwDiag)
        {
            int[] nMGrid = { 2, 3, 4 };

            FritOptResult best = new FritOptResult { Cost = double.PositiveInfinity };
            tsBest = -1;
            nMBest = nMGrid[0];
            bwDiag = "";

            foreach (int nM in nMGrid)
            {
                double tsEst = EstimateTsFromClosedLoopBandwidth(
                    S_rr, S_yr_re, S_yr_im, binWidthHz, nM, out string diag);
                if (tsEst <= 0) { bwDiag = diag; continue; }

                double tsClamped = Math.Max(3.0 * dt, tsEst);
                tsClamped = Math.Max(tsClamped, tauM);

                var r = RunFritMultistart(u, uInject, y, sat, dt, tsClamped, nM, tauM,
                                          cohPerBin, kpCurr, tiCurr, tdCurr);
                if (double.IsInfinity(r.Cost) || double.IsNaN(r.Cost)) continue;
                if (r.Cost < best.Cost)
                {
                    best = r;
                    tsBest = tsClamped;
                    nMBest = nM;
                    bwDiag = diag;
                }
            }
            return best;
        }

    }
}
