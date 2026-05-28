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
using System.Numerics;                       // Complex (Two-stage 의 G(z) 극점 추출)
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

            // 적응형 진폭 제거 — 사용자 슬라이더 (Excite Amp) 값 그대로 사용.
            // PRBS 가 ±A 로 강제 bounded 라 자동 boost 없어도 안전.

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
            public readonly List<double> R = new List<double>();          // 가진 신호 (PRBS, IV-ARX 의 instrument)
            public readonly List<bool>   Saturated = new List<bool>();    // 이 샘플이 포화 중인지 (가중치용)

            // ── 포화 회복 추적 ──
            // 포화 끝난 직후 1/C(z) IIR 역필터 state 가 회복하는 데 ~2초 (TransientTailSamples) 소요.
            // 그 동안 계산되는 e[k] 가 오염됨 → 해당 인덱스는 FRIT 가중치에서 down-weight.
            public int SaturatedCount;
            public int SamplesSinceLastSat;   // 마지막 포화 이후 경과 샘플 수
            public int EffectiveValidCount;   // transient tail 밖에 있는 깨끗한 샘플 누적

            // 사전 진단 상태 (Diagnosing 단계, 2-phase)
            //   Phase 0 (0-3s): 가진 OFF — limit cycle / 지속 포화 검출
            //   Phase 1 (3-6s): 작은 perturbation — "이미 완벽 PID" 검출 (tracking_ratio < 0.05)
            public int    DiagPhase;        // 0 또는 1
            public double DiagT;            // 진단 누적 시간 (초)
            public int    DiagSampleCount;
            public double DiagUMax, DiagUMin;
            public int    DiagSatCount;     // |u| ≥ 임계 카운트
            public int    DiagSignChanges;  // u 부호 변환 횟수
            public double DiagPrevU;        // 직전 u (부호 변환 검출용)

            // Phase 1 (perturbation test) 용:
            public double DiagYBaseline;    // Phase 0 의 y 평균 (perturbation 기준)
            public double DiagYBaselineSum; // Phase 0 의 y 합 (mean 계산용)
            public int    DiagYBaselineCnt;
            public double DiagYSum;         // Phase 1 의 (y - baseline) 합
            public double DiagYSumSq;       // Phase 1 의 (y - baseline)^2 합
            public int    DiagYCountP1;     // Phase 1 샘플 수

            public double LastU;                    // 마지막 제어 출력 (참고용)
            public double NaturalYStd;              // 가진 전 자연 변동 (참고용)

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
            public int    PrbsBitTicks = 4;         // 현재 bit duration (틱). 초기값 = 0.1초.
            public int    TicksSinceSpectralCheck;  // 마지막 spectral check 후 경과

            // Hybrid 가진 — u-direct 보조 amp (headroom-based)
            public double UAmpDyn;                  // 현재 u-direct base amplitude (dyn 조정)

            // Information-driven termination state
            //   매 6초 max(S) 측정 → 연속 3 windows < ε 이면 well-tuned 종료
            public int    WellTunedConsecutiveCount;
            public double LastMaxSensitivity;        // 가장 최근 측정된 max(S) (status 표시용)

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
                Saturated.Clear();
                SaturatedCount = 0;
                SamplesSinceLastSat = int.MaxValue / 2;   // 시작 시 "이미 충분히 오래 깨끗" 상태
                EffectiveValidCount = 0;
                DiagPhase = 0;
                DiagT = 0;
                DiagSampleCount = 0;
                DiagUMax = DiagUMin = 0;
                DiagSatCount = 0;
                DiagSignChanges = 0;
                DiagPrevU = 0;
                DiagYBaseline = 0;
                DiagYBaselineSum = 0;
                DiagYBaselineCnt = 0;
                DiagYSum = 0;
                DiagYSumSq = 0;
                DiagYCountP1 = 0;
                LastU = 0;
                NaturalYStd = 0;
                // PRBS 초기화: state = 1 (어떤 non-zero seed 든 OK, 결정론적)
                PrbsState = 1;
                PrbsTicksInBit = 0;
                PrbsCurrentValue = 1.0;   // 첫 bit = state & 1 = 1 → +A
                AmpDyn = 0;               // StartRecording 에서 _s.ExciteAmp 으로 설정.
                UAmpDyn = 0;              // StartRecording 에서 초기화
                TicksSinceAmpAdjust = 0;
                PrbsHpfInPrev = 0;
                PrbsHpfOutPrev = 0;
                PrbsBitTicks = 4;         // 초기 = 0.1초 bit (broadband)
                TicksSinceSpectralCheck = 0;
                WellTunedConsecutiveCount = 0;
                LastMaxSensitivity = 1.0;  // 초기 unknown
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

        // u-direct excitation 용 y baseline.
        // Diagnose Phase 0 (가진 OFF 3초) 에서 측정된 y 평균. 수집 동안 plant 가 base 에서
        // 너무 벗어나면 (= 비행기 자세 무너지면) amp 줄여서 안전 확보.
        private double _recordingYBaseline;

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
                // u 는 plant 가 실제로 받는 값 = clip 된 값. 안전을 위해 명시적 clamp.
                // 이렇게 하면 measured u = actual plant input → IV-ARX 가 포화 데이터에서도 unbiased.
                double uRaw = c.LastControlVariable;
                double u = Math.Max(-1.0, Math.Min(1.0, uRaw));
                double y = c.LastProcessVariable;
                _sess.LastU = u;

                // 포화 추적 (telemetry/UI 용). 회귀에서는 제외 안 함 — clamped u 가 plant 입력이라 unbiased.
                bool saturated = Math.Abs(uRaw) >= _s.SaturationThreshold;
                if (saturated)
                {
                    _sess.SaturatedCount++;
                    _sess.SamplesSinceLastSat = 0;
                }
                else
                {
                    _sess.SamplesSinceLastSat++;
                }

                // 적응형 진폭 제거 — 사용자 슬라이더 (Excite Amp) 값 그대로 사용.
                // PRBS 는 진폭 ±A 로 강제 bounded 라 adaptive 보호 불필요.

                _sess.U.Add(u);
                _sess.Y.Add(y);
                _sess.R.Add(_lastExciteValue);   // PRBS reference — IV-ARX 의 instrument
                _sess.Saturated.Add(saturated);

                // 포화 데이터도 IV-ARX 회귀에 포함 (clamped u = actual plant input → unbiased).
                // EffectiveValidCount = 총 샘플 수.
                _sess.EffectiveValidCount = _sess.U.Count;

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
                    _sess.LastMessage =
                        $"Collecting... N={_sess.U.Count} ({_sess.T:0.0}s/{MAX_COLLECT_SEC:0.0}s), " +
                        $"max|S|={_sess.LastMaxSensitivity:0.000}, well-tuned count={_sess.WellTunedConsecutiveCount}/{WELL_TUNED_CONSEC_REQUIRED}, " +
                        $"sat={_sess.SaturatedCount}, amp={_sess.AmpDyn:0.000}/u={_sess.UAmpDyn:0.000}, bit={_sess.PrbsBitTicks} / 수집중";
                }

                // 종료 결정
                if (_autoState == AutoTuneState.Recording)
                {
                    bool wellTuned = _sess.WellTunedConsecutiveCount >= WELL_TUNED_CONSEC_REQUIRED;
                    bool timeout = _sess.T >= MAX_COLLECT_SEC;

                    if (wellTuned)
                    {
                        // controller 가 거의 완벽 → FRIT 의미 없음
                        StopRecording();
                        _autoState = AutoTuneState.Done;
                        _sess.HasResult = true;
                        _sess.Kp = this._focus.Pid.kP.Us;
                        _sess.Ti = this._focus.Pid.kI.Us;
                        _sess.Td = this._focus.Pid.kD.Us;
                        _sess.LastMessage =
                            $"✓ Well-tuned (max|S|={_sess.LastMaxSensitivity:0.000} < {WELL_TUNED_S_THRESHOLD} for {WELL_TUNED_CONSEC_REQUIRED} windows). " +
                            $"Controller rejects all bands → closed-loop identifiability limit. " +
                            $"No improvement possible. / 이미 잘 튜닝됨.";
                    }
                    else if (timeout)
                    {
                        // 60초 도달 — 정보 풍부 (S 큼) → FRIT 실행
                        StopRecording();
                        _autoState = AutoTuneState.Computing;
                        _sess.LastMessage = $"Collection {MAX_COLLECT_SEC}s complete (max|S|={_sess.LastMaxSensitivity:0.000}), analyzing... / 수집 완료, 분석 중";
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
            // Hybrid 가진 초기 amp:
            //   SP_amp = 사용자 슬라이더 (사용자 의도 반영, sat-aware adaptive 가 자동 조정)
            //   u_amp = 보수적 0.03 (headroom envelope 안에서 추가 안전)
            _sess.AmpDyn = Math.Max(AMP_DYN_MIN, Math.Min(AMP_DYN_MAX, _s.ExciteAmp));
            _sess.UAmpDyn = 0.03;
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

            // SP-direct PRBS 가진. 진폭은 사용자 슬라이더 (Excite Amp) 값 그대로.
            // 슬라이더가 0 이면 default 0.3 사용 (사용자 첫 시도 보호).
            if (_s.ExciteAmp <= 0.01f) _s.ExciteAmp = 0.3f;
            _s.ExciteEnabled = true;
            _s.ExciteWave = WaveType.MultiSine;  // 코드 상 wave type, 실제는 PRBS

            _autoState = AutoTuneState.Recording;
            StartRecording();
            _sess.LastMessage = $"Recording (PRBS amp={_s.ExciteAmp:0.00}) / 녹화 중 (PRBS 가진)";
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
            double[] r = _sess.R.Count == _sess.U.Count ? _sess.R.ToArray() : null;
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

            // ── FRIT (Fictitious Reference Iterative Tuning) — Soma/Kaneko 2004 ──
            // Model-free PID tuning. (u, y, currentPid) 로 직접 PID 산출.
            //   Cost: J(θ) = Σ (y - M(z)·r̃(θ))², r̃(θ) = y + C(θ)⁻¹·u
            //   LM 최적화, 27-grid multistart (Kp/Ti/Td log-spaced).
            //   Ts 는 사용자 슬라이더 (SettlingTimeTs) 직접 사용 — sweep 없음.
            double tauM = Math.Max(dt, (double)_s.ModelDelayTau);
            double userTs = Math.Max(3.0 * dt, Math.Min(5.0, _s.SettlingTimeTs));
            int nM = 2;

            FritOptResult fr = RunFritMultistart(u, y, sat, dt, userTs, nM, tauM, kp0, ti0, td0);

            if (double.IsInfinity(fr.Cost) || double.IsNaN(fr.Cost))
            {
                _autoState = AutoTuneState.Failed;
                _sess.LastMessage = "FRIT failed (no valid seed converged) / FRIT 실패";
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
            _sess.Kp = kpFinal; _sess.Ti = tiFinal; _sess.Td = tdFinal;
            _sess.KpSE = 0; _sess.TiSE = 0; _sess.TdSE = 0;
            _sess.FitRmse = Math.Sqrt(Math.Max(0, fr.Cost));

            _autoState = AutoTuneState.Done;
            _sess.LastMessage =
                $"Done | FRIT (28 seeds, Ts={userTs:0.000}s) → " +
                $"Kp={kpFinal:0.000} Ti={tiFinal:0.0} Td={tdFinal:0.00} " +
                $"(cost={fr.Cost:0.0000}, {fr.Diag})";
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
        // OnDiagnoseTick — Auto Tune 직후 2-phase 사전 진단 (총 6초)
        // ════════════════════════════════════════════════════════════════════════
        //
        // Phase 0 (0-3s, 가진 OFF): 현재 PID 가 *튜닝 가능 상태* 인지 확인.
        //   판정:
        //     1. Limit cycle: satRate > 40%, crossRate > 0.5/s, uSwing > 1.6
        //     2. 지속 포화: satRate > 40% (진동 적음)
        //     3. 살짝 포화: satRate 15~40% → 경고 후 phase 1 진행
        //     4. 정상: satRate < 15% → phase 1 진행
        //   Phase 0 의 y 평균 = baseline (phase 1 의 perturbation 기준)
        //
        // Phase 1 (3-6s, 작은 PRBS perturbation): 현재 PID 가 *너무 완벽한지* 확인.
        //   작은 amplitude (=0.05) PRBS 적용 → y_std 측정 → tracking_ratio = y_std/amp
        //     - ratio < 0.05  : 완벽 추종 → "정보 부족, 튜닝 의미 없음" 종료
        //     - 0.05 ≤ < 0.3  : 잘 작동, 정상 진행
        //     - ratio ≥ 0.3   : 약한 PID, 정보 풍부, 정상 진행
        //
        // 학계: closed-loop identifiability — 강한 controller 일수록 tracking error 작아
        //   plant 정보 적음 (Forssell-Ljung 1999). FRIT 도 동일 한계.
        // ════════════════════════════════════════════════════════════════════════
        private const double DIAG_PHASE0_DUR = 3.0;
        private const double DIAG_PHASE1_DUR = 3.0;
        private const double DIAG_PERTURB_AMP = 0.05;
        private const double DIAG_WELL_TUNED_THRESHOLD = 0.05;  // tracking_ratio 임계: 완벽 추종

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

            if (_sess.DiagPhase == 0)
            {
                // ── Phase 0: 가진 OFF, |u| 통계 + y baseline 누적 ──
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
                _sess.LastMessage = $"Diag P0... {_sess.DiagT:0.0}s/{DIAG_PHASE0_DUR:0.0}s (uPeak={uPeakSoFar:0.00}) / 진단 P0";

                if (_sess.DiagT < DIAG_PHASE0_DUR) return;

                // Phase 0 판정
                double satRate = (double)_sess.DiagSatCount / Math.Max(1, _sess.DiagSampleCount);
                double uSwing = _sess.DiagUMax - _sess.DiagUMin;
                double uPeak = Math.Max(Math.Abs(_sess.DiagUMax), Math.Abs(_sess.DiagUMin));
                double crossRate = _sess.DiagSignChanges / DIAG_PHASE0_DUR;

                if (satRate > 0.40 && crossRate > 0.5 && uSwing > 1.6)
                {
                    _autoState = AutoTuneState.Failed;
                    _sess.LastMessage =
                        $"⚠ Limit cycle (u={_sess.DiagUMin:0.00}~{_sess.DiagUMax:0.00}, " +
                        $"{crossRate:0.0}/s, sat={satRate:P0}). " +
                        $"Kp 과대 / Ti 과소 (windup) / Td 과대 의심. 게인 낮춰 재시도.";
                    return;
                }
                if (satRate > 0.40)
                {
                    _autoState = AutoTuneState.Failed;
                    _sess.LastMessage =
                        $"⚠ 지속 포화 (sat={satRate:P0}, uPeak={uPeak:0.00}). " +
                        $"Kp/Ki 과대 또는 SP 가 액추에이터 한계 초과 의심.";
                    return;
                }

                // Phase 1 진입 준비
                _sess.DiagPhase = 1;
                _sess.DiagYBaseline = _sess.DiagYBaselineSum / Math.Max(1, _sess.DiagYBaselineCnt);
                _sess.DiagT = 0;  // Phase 1 timer reset
                _sess.LastMessage = $"Diag P0 OK (sat={satRate:P0}). Phase 1 시작 (perturbation test)";
                return;
            }

            // ── Phase 1: 작은 PRBS perturbation + y 변동 측정 ──
            // PRBS bit 진행 (LFSR 그대로 사용)
            _sess.PrbsTicksInBit++;
            if (_sess.PrbsTicksInBit >= PRBS_BIT_TICKS)
            {
                int newBit = ((_sess.PrbsState >> PRBS_TAP1) ^ (_sess.PrbsState >> PRBS_TAP2)) & 1;
                _sess.PrbsState = ((_sess.PrbsState << 1) | newBit) & PRBS_MASK;
                _sess.PrbsCurrentValue = (newBit == 1) ? 1.0 : -1.0;
                _sess.PrbsTicksInBit = 0;
            }
            double perturb = DIAG_PERTURB_AMP * _sess.PrbsCurrentValue;
            // SP 에 작은 perturbation 주입 (base 가 있어야 적용)
            if (_hasBaseSetPointAdjust)
            {
                try { this._focus.SetPointAdjust.Us = _baseSetPointAdjust + (float)perturb; } catch { }
            }

            // y 변동 누적 (baseline 차분)
            double yDiff = y - _sess.DiagYBaseline;
            _sess.DiagYSum += yDiff;
            _sess.DiagYSumSq += yDiff * yDiff;
            _sess.DiagYCountP1++;

            _sess.DiagT += dt;
            _sess.LastMessage = $"Diag P1... {_sess.DiagT:0.0}s/{DIAG_PHASE1_DUR:0.0}s (perturb test) / 진단 P1";

            if (_sess.DiagT < DIAG_PHASE1_DUR) return;

            // Phase 1 판정
            // SetPoint 복원
            if (_hasBaseSetPointAdjust)
            {
                try { this._focus.SetPointAdjust.Us = _baseSetPointAdjust; } catch { }
            }

            int n = _sess.DiagYCountP1;
            double yMean = _sess.DiagYSum / Math.Max(1, n);
            double yVar = _sess.DiagYSumSq / Math.Max(1, n) - yMean * yMean;
            double yStd = (yVar > 0) ? Math.Sqrt(yVar) : 0;
            double trackingRatio = yStd / DIAG_PERTURB_AMP;

            if (trackingRatio < DIAG_WELL_TUNED_THRESHOLD)
            {
                // 이미 완벽 추종 — closed-loop identifiability limit. 정보 부족.
                _autoState = AutoTuneState.Done;
                _sess.HasResult = true;
                _sess.Kp = this._focus.Pid.kP.Us;
                _sess.Ti = this._focus.Pid.kI.Us;
                _sess.Td = this._focus.Pid.kD.Us;
                _sess.LastMessage =
                    $"✓ Already well-tuned (tracking ratio={trackingRatio:0.000} < {DIAG_WELL_TUNED_THRESHOLD}). " +
                    $"Current PID rejects perturbation perfectly — closed-loop identifiability limit reached. " +
                    $"No improvement possible without weaker C. " +
                    $"/ 이미 잘 튜닝됨 — 정보 부족. 더 튜닝 불가.";
                return;
            }

            // 정상 진행 — Recording 단계로
            // Phase 0 의 y baseline 을 recording 동안 사용 (u-direct safety)
            _recordingYBaseline = _sess.DiagYBaseline;
            _sess.LastMessage = $"Diag OK (tracking={trackingRatio:0.000}). 녹화 시작.";
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
        /// <summary>현재 틱 가진값 (telemetry).</summary>
        private double _lastExciteValue = 0.0;

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
        // SP-direct amp 범위 (사용자 슬라이더 ExciteAmp 의 동적 조정)
        private const double AMP_DYN_MIN = 0.01;
        private const double AMP_DYN_MAX = 1.0;
        // u-direct amp 범위 (보조 가진, headroom 의 fraction)
        private const double U_AMP_MIN = 0.005;
        private const double U_AMP_MAX = 0.3;
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
        //   3 band: low (0.05-0.5Hz), mid (0.5-2Hz), high (2-5Hz)
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
        /// Sensitivity-aware spectral monitor — y/r 비율로 sensitivity 분석.
        ///
        /// 학계 (Forssell-Ljung 1999, Skogestad-Postlethwaite ch.2):
        ///   T(jω) = y/r = closed-loop transfer (= CG/(1+CG))
        ///   S(jω) = 1 - T(jω) = sensitivity (= 1/(1+CG))
        ///
        ///   |S| 큰 band = controller 가 reject 못 함 = 정보 풍부 → 그 band 자극
        ///   |S| 모든 band 에서 작음 = controller 가 모든 band 잘 잡음 = well-tuned
        ///
        /// 두 역할:
        ///   1) 부족한 band (= sensitivity 큰 band) 식별 → PRBS bit_ticks 조정
        ///   2) max sensitivity 반환 → termination 결정 (well-tuned 감지)
        /// </summary>
        /// <returns>max(|S(f)|) over all bands, 또는 -1 if invalid</returns>
        private double UpdatePrbsBitTicksFromSpectrum()
        {
            int N = _sess.Y.Count;
            if (N < SPECTRAL_FFT_LEN) return -1;
            if (_sess.R.Count < SPECTRAL_FFT_LEN) return -1;

            double dt = Time.fixedDeltaTime;
            if (dt <= 0) dt = 0.025;

            int start = N - SPECTRAL_FFT_LEN;
            var ySamples = new Complex[SPECTRAL_FFT_LEN];
            var rSamples = new Complex[SPECTRAL_FFT_LEN];

            double yMean = 0, rMean = 0;
            for (int i = 0; i < SPECTRAL_FFT_LEN; i++)
            {
                yMean += _sess.Y[start + i];
                rMean += _sess.R[start + i];
            }
            yMean /= SPECTRAL_FFT_LEN;
            rMean /= SPECTRAL_FFT_LEN;
            for (int i = 0; i < SPECTRAL_FFT_LEN; i++)
            {
                ySamples[i] = new Complex(_sess.Y[start + i] - yMean, 0);
                rSamples[i] = new Complex(_sess.R[start + i] - rMean, 0);
            }

            try
            {
                MathNet.Numerics.IntegralTransforms.Fourier.Forward(ySamples);
                MathNet.Numerics.IntegralTransforms.Fourier.Forward(rSamples);
            }
            catch { return -1; }

            double binWidthHz = 1.0 / (SPECTRAL_FFT_LEN * dt);
            int loStart = Math.Max(1, (int)Math.Round(0.05 / binWidthHz));
            int loEnd = (int)Math.Round(0.5 / binWidthHz);
            int midEnd = (int)Math.Round(2.0 / binWidthHz);
            int hiEnd = Math.Min(SPECTRAL_FFT_LEN / 2, (int)Math.Round(5.0 / binWidthHz));

            // 각 band 의 평균 |T(f)|
            double tLo = ComputeAvgTMag(ySamples, rSamples, loStart, loEnd);
            double tMid = ComputeAvgTMag(ySamples, rSamples, loEnd, midEnd);
            double tHi = ComputeAvgTMag(ySamples, rSamples, midEnd, hiEnd);

            // Sensitivity |S| = |1 - T|. T 가 1 에 가까우면 S ≈ 0 (well tracked).
            double sLo = Math.Abs(1.0 - tLo);
            double sMid = Math.Abs(1.0 - tMid);
            double sHi = Math.Abs(1.0 - tHi);

            // 부족 band 식별 = 가장 sensitivity 큰 band (= 정보 풍부 = 자극할 가치 큼)
            int newBitTicks;
            if (sLo >= sMid && sLo >= sHi)      newBitTicks = 64;  // low S 큼 → low band 자극
            else if (sMid >= sHi)               newBitTicks = 16;  // mid
            else                                newBitTicks = 4;   // high

            _sess.PrbsBitTicks = newBitTicks;

            // Termination metric: max(S) over all bands
            return Math.Max(sLo, Math.Max(sMid, sHi));
        }

        private static double ComputeAvgTMag(Complex[] y, Complex[] r, int binStart, int binEnd)
        {
            double sum = 0;
            int cnt = 0;
            for (int i = binStart; i < binEnd; i++)
            {
                double rMag = r[i].Magnitude;
                if (rMag < 1e-9) continue;  // r 거의 0 인 bin skip
                double yMag = y[i].Magnitude;
                sum += yMag / rMag;
                cnt++;
            }
            return (cnt > 0) ? sum / cnt : 0;
        }

        private void ApplyExcitation(float dt)
        {
            _lastExciteValue = 0.0;

            if (!_s.ExciteEnabled || _s.ExciteWave == WaveType.Off || !_hasBaseSetPointAdjust)
            {
                FritExcitationInjector.Clear(this._focus);
                try { this._focus.SetPointAdjust.Us = _baseSetPointAdjust; } catch { }
                return;
            }

            double t = _sess.T;
            double x = 0.0;

            // Sine/Chirp 는 수동 실험용으로 보존 (SP-direct 방식 유지)
            // MultiSine (=PRBS) 만 AutoTune 의 표준 — u-direct injection.
            switch (_s.ExciteWave)
            {
                case WaveType.Sine:
                    {
                        double ampS = Math.Max(0.0, _s.ExciteAmp);
                        double w = 2.0 * Math.PI * Math.Max(0.01, _s.ExciteFreqHz);
                        x = ampS * Math.Sin(w * t);
                        FritExcitationInjector.Clear(this._focus);
                        _lastExciteValue = x;
                        try { this._focus.SetPointAdjust.Us = _baseSetPointAdjust + (float)x; } catch { }
                        return;
                    }
                case WaveType.Chirp:
                    {
                        double ampC = Math.Max(0.0, _s.ExciteAmp);
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
                        x = ampC * Math.Sin(phase);
                        FritExcitationInjector.Clear(this._focus);
                        _lastExciteValue = x;
                        try { this._focus.SetPointAdjust.Us = _baseSetPointAdjust + (float)x; } catch { }
                        return;
                    }
            }

            // ── MultiSine = Hybrid PRBS (SP-direct + u-direct) ──
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

            // FRIT instrument = SP inject (main 신호). u-direct 는 보조 plant 자극용.
            _lastExciteValue = spInject;
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
        /// 수집 중 중간 식별로 SE/τ_1 비율 반환 — SE-게이트 (수집 종료 조건) 용.
        /// 너무 짧거나 식별 실패 시 +∞ 반환 → 수집 계속.
        /// </summary>
        private double QuickIdSeRatio()
        {
            int n = _sess.U.Count;
            if (n < 128) return double.PositiveInfinity;
            double dt = Time.fixedDeltaTime;
            if (dt <= 0) dt = 0.02;

            double[] y = _sess.Y.ToArray();
            double[] r = _sess.R.ToArray();

            // Two-stage 의 SE — 현재 PID 사용
            double kp = this._focus.Pid.kP.Us;
            double ti = this._focus.Pid.kI.Us;
            double td = this._focus.Pid.kD.Us;

            PlantModel m = IdentifyPlantTwoStage(y, r, dt, 0, kp, ti, td);
            if (!m.Valid || m.Tau1 <= 1e-6 || double.IsNaN(m.TauSE)) return double.PositiveInfinity;
            return m.TauSE / m.Tau1;
        }

        /// <summary>
        /// (u, y, r) 데이터에 IV-ARX/OE (Refined Instrumental Variables) 로 plant G 식별.
        ///
        /// 모델 가정 (OE: Output Error, 더 현실적):
        ///   y[k] = G(z) u[k] + v[k]     where v ~ output noise (colored when reflected to ARX form)
        ///
        /// 전략 (RIV; Young 1980, Söderström-Stoica §8):
        ///   Stage 1: Plain ARX OLS — 초기 추정 (a₁⁰, a₂⁰, b⁰)
        ///   Stage 2+: RIV iteration
        ///     - 현재 추정으로 y_sim 시뮬레이션 (noise-free plant output)
        ///     - IV matrix Z = [y_sim[k-1], y_sim[k-2], r[k-1-δ]]
        ///       · y_sim 은 시뮬레이션 → noise 와 무상관 ✓
        ///       · r 은 사용자 가진 → noise 와 무상관 ✓
        ///     - (Z^T X) θ = Z^T y 풀이
        ///     - 수렴 (Δθ 작음) 까지 2~3회
        ///   점근적으로 OE-PEM 과 동일 efficiency (Young 정리).
        ///
        /// 입력 r 이 null 이면 ARX OLS 로 폴백.
        /// </summary>
        private static PlantModel IdentifyPlantArx(double[] u, double[] y, double[] r, bool[] sat, double dt, double theta)
        {
            PlantModel m = new PlantModel { Theta = theta };
            int N = Math.Min(u.Length, Math.Min(y.Length, sat.Length));
            if (N < 64) { m.Diagnosis = "data too short"; return m; }
            bool useIv = (r != null && r.Length >= N);

            int delayN = Math.Max(0, (int)Math.Round(theta / dt));
            int kStart = 2 + delayN;
            if (kStart >= N - 4) { m.Diagnosis = "delay > data"; return m; }

            double yStd = StdDev(y);
            if (yStd < 1e-4) { m.Diagnosis = "y barely moves — excitation too weak?"; return m; }

            // Detrend
            double[] yd = new double[N]; Array.Copy(y, yd, N); Detrend(yd);
            double[] ud = new double[N]; Array.Copy(u, ud, N); Detrend(ud);
            double[] rd = null;
            if (useIv) { rd = new double[N]; Array.Copy(r, rd, N); Detrend(rd); }

            // ── Stage 1: ARX OLS (initial estimate) ──
            // regressors X = [y[k-1], y[k-2], u[k-1-δ]]
            // 포화 샘플도 포함: measured u = clipped value = actual plant input → unbiased.
            // saturation 은 information loss 만 만들고 bias 안 만듦 (linear-in-params 회귀).
            double s11 = 0, s12 = 0, s13 = 0, s22 = 0, s23 = 0, s33 = 0;
            double t1 = 0, t2 = 0, t3 = 0;
            int count = 0;
            for (int k = kStart; k < N; k++)
            {
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

            double M11 = s22 * s33 - s23 * s23;
            double M12 = s12 * s33 - s23 * s13;
            double M13 = s12 * s23 - s22 * s13;
            double det = s11 * M11 - s12 * M12 + s13 * M13;
            if (Math.Abs(det) < 1e-12) { m.Diagnosis = "regressors collinear"; return m; }

            double a1 = (t1 * M11 - s12 * (t2 * s33 - s23 * t3) + s13 * (t2 * s23 - s22 * t3)) / det;
            double a2 = (s11 * (t2 * s33 - s23 * t3) - t1 * (s12 * s33 - s23 * s13) + s13 * (s12 * t3 - t2 * s13)) / det;
            double b  = (s11 * (s22 * t3 - s23 * t2) - s12 * (s12 * t3 - s13 * t2) + t1 * (s12 * s23 - s22 * s13)) / det;
            if (double.IsNaN(a1) || double.IsNaN(a2) || double.IsNaN(b))
            { m.Diagnosis = "NaN in initial ARX fit"; return m; }

            // ── Stage 2+: RIV refinement (편향 제거) ──
            int rivIters = 0;
            if (useIv)
            {
                const int MAX_RIV_ITER = 3;
                const double RIV_TOL = 0.001;
                double[] ySim = new double[N];

                for (int rivIter = 0; rivIter < MAX_RIV_ITER; rivIter++)
                {
                    // 안정성 가드: 현재 추정이 unstable 이면 simulation 이 발산해서 IV 가 오염.
                    // 특성다항식 root |z| < 1 인지 확인 후 simulate.
                    double discCheck = a1 * a1 + 4.0 * a2;
                    double zMaxAbs;
                    if (discCheck >= 0)
                    {
                        double sqC = Math.Sqrt(discCheck);
                        zMaxAbs = Math.Max(Math.Abs((a1 + sqC) / 2.0), Math.Abs((a1 - sqC) / 2.0));
                    }
                    else
                    {
                        zMaxAbs = Math.Sqrt(Math.Max(0, -a2));
                    }
                    if (zMaxAbs > 1.0)
                    {
                        // ARX initial 이 이미 unstable → RIV iteration 의미 없음, ARX 결과 유지.
                        break;
                    }

                    // y_sim: noise-free simulation with current estimate.
                    // y_sim[k] = a₁ y_sim[k-1] + a₂ y_sim[k-2] + b u_d[k-1-δ]
                    ySim[0] = yd[0];
                    ySim[1] = yd[1];
                    for (int k = 2; k < N; k++)
                    {
                        double ud1 = (k - 1 - delayN >= 0) ? ud[k - 1 - delayN] : 0;
                        ySim[k] = a1 * ySim[k - 1] + a2 * ySim[k - 2] + b * ud1;
                    }

                    // IV regressor Z = [y_sim[k-1], y_sim[k-2], r[k-1-δ]]
                    // (Z^T X) θ = Z^T y. 3x3 시스템.
                    double zx11 = 0, zx12 = 0, zx13 = 0;
                    double zx21 = 0, zx22 = 0, zx23 = 0;
                    double zx31 = 0, zx32 = 0, zx33 = 0;
                    double zy1 = 0, zy2 = 0, zy3 = 0;
                    int rivCount = 0;
                    for (int k = kStart; k < N; k++)
                    {
                        double yt = yd[k];
                        double y1 = yd[k - 1], y2 = yd[k - 2], u1 = ud[k - 1 - delayN];
                        double ys1 = ySim[k - 1], ys2 = ySim[k - 2];
                        double r1 = rd[k - 1 - delayN];

                        zx11 += ys1 * y1; zx12 += ys1 * y2; zx13 += ys1 * u1;
                        zx21 += ys2 * y1; zx22 += ys2 * y2; zx23 += ys2 * u1;
                        zx31 += r1  * y1; zx32 += r1  * y2; zx33 += r1  * u1;
                        zy1 += ys1 * yt; zy2 += ys2 * yt; zy3 += r1 * yt;
                        rivCount++;
                    }
                    if (rivCount < 32) break;

                    // 3x3 시스템 (Z^T X) θ = (Z^T y) Cramer 풀이
                    double Md = zx11 * (zx22 * zx33 - zx23 * zx32)
                              - zx12 * (zx21 * zx33 - zx23 * zx31)
                              + zx13 * (zx21 * zx32 - zx22 * zx31);
                    if (Math.Abs(Md) < 1e-12) break;

                    double a1n = ( zy1 * (zx22 * zx33 - zx23 * zx32)
                                 - zx12 * (zy2 * zx33 - zx23 * zy3)
                                 + zx13 * (zy2 * zx32 - zx22 * zy3)) / Md;
                    double a2n = ( zx11 * (zy2 * zx33 - zx23 * zy3)
                                 - zy1  * (zx21 * zx33 - zx23 * zx31)
                                 + zx13 * (zx21 * zy3 - zy2 * zx31)) / Md;
                    double bn  = ( zx11 * (zx22 * zy3 - zy2 * zx32)
                                 - zx12 * (zx21 * zy3 - zy2 * zx31)
                                 + zy1  * (zx21 * zx32 - zx22 * zx31)) / Md;
                    if (double.IsNaN(a1n) || double.IsNaN(a2n) || double.IsNaN(bn)) break;

                    double change = Math.Abs(a1n - a1) + Math.Abs(a2n - a2) + Math.Abs(bn - b) / Math.Max(1e-6, Math.Abs(b));
                    a1 = a1n; a2 = a2n; b = bn;
                    rivIters = rivIter + 1;
                    if (change < RIV_TOL) break;
                }
            }

            // 잔차 RMSE (최종 estimate 기준) — 포화 샘플도 포함.
            double sqResid = 0;
            int cResid = 0;
            for (int k = kStart; k < N; k++)
            {
                double pred = a1 * yd[k - 1] + a2 * yd[k - 2] + b * ud[k - 1 - delayN];
                double e = yd[k] - pred;
                sqResid += e * e;
                cResid++;
            }
            m.FitRmse = Math.Sqrt(sqResid / Math.Max(1, cResid));

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

            // 적분기 판정 — 통계적 기준 (휴리스틱 임계값 사용 안 함):
            //   진짜 적분기는 1 - a₁ - a₂ = 0 (DC 게인 ∞).
            //   추정치의 SE 보다 작으면 "noise 와 구분 불가" → 적분기.
            //   SE(1-a₁-a₂) ≈ √(σ²·(M_11 + 2·M_12 + M_22)/det) — Fisher info 기반.
            double denom = 1.0 - a1 - a2;
            double seDenom = Math.Sqrt(sigma2 * Math.Abs(M11 + 2.0 * M12 + s11 * s33 - s13 * s13) / Math.Max(1e-12, Math.Abs(det)));
            m.HasIntegrator = Math.Abs(denom) < 2.0 * seDenom;  // 2σ 통계적 판정

            // 불안정 추정 거부 — SIMC physical: stable plant 면 |z| < 1.
            //   적분기 plant 는 |z|=1 근방. 노이즈/수치오차로 |z| 가 1 을 약간 넘는 건 흔함.
            //   |z| ∈ (1, 2] 는 적분기로 간주 (아래 cap 로직이 처리), |z| > 2 만 truly unstable.
            if (zSlow > 2.0)
            { m.Diagnosis = $"slow pole |z|={zSlow:0.000} > 2.0 (truly unstable)"; return m; }
            if (zSlow <= 0)
            { m.Diagnosis = $"slow pole |z|={zSlow:0.000} ≤ 0 (invalid)"; return m; }
            // |z| > 1 → 사실상 적분기 (noise 가 stable 극을 unit circle 밖으로 밀어낸 경우)
            if (zSlow > 1.0) m.HasIntegrator = true;

            // |z| ≥ 1 노이즈 → 1 직전으로 cap (적분기 표현). cap 자체는 수치 보호 (1/ln(1) = ∞).
            double zForTau = Math.Min(zSlow, 1.0 - 1e-6);
            m.Tau1 = -dt / Math.Log(zForTau);

            // τ_2 (rate damping) 동일 방식 — cap 만 수치적 보호.
            double zFastClamped = Math.Max(1e-6, Math.Min(zFast, 1.0 - 1e-6));
            m.Tau2 = -dt / Math.Log(zFastClamped);

            // DC gain K 계산:
            //   일반 plant: K = b / (1 - a₁ - a₂)    (denom: DC gain 의 역수)
            //   적분기 plant: K_i = b / dt           (rate gain, denom ≈ 0)
            if (m.HasIntegrator)
            {
                m.K = b / Math.Max(1e-6, dt);
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
            string method = useIv ? $"IV-ARX (RIV ×{rivIters})" : "ARX OLS";
            m.Diagnosis = m.HasIntegrator
                ? $"{method}: integrator, τ_other={m.Tau2:0.000}s, K={m.K:0.000}"
                : (m.Tau2 > 0.01
                    ? $"{method}: 2nd-order τ_1={m.Tau1:0.000}s τ_2={m.Tau2:0.000}s K={m.K:0.000}"
                    : $"{method}: 1st-order τ_p={m.Tau1:0.000}s K={m.K:0.000}");
            return m;
        }

        /// <summary>다항식 곱셈 — a[0]+a[1]z⁻¹+... × b[0]+b[1]z⁻¹+... 결과 c.</summary>
        private static double[] PolyMult(double[] a, double[] b)
        {
            double[] c = new double[a.Length + b.Length - 1];
            for (int i = 0; i < a.Length; i++)
                for (int j = 0; j < b.Length; j++)
                    c[i + j] += a[i] * b[j];
            return c;
        }

        /// <summary>
        /// 다항식 p[0]+p[1]z⁻¹+...+p[n]z⁻ⁿ 을 복소 z 에서 평가. (z⁻¹ = 1/z 사용)
        /// </summary>
        private static Complex PolyEvalAtZ(double[] coeffs, Complex z)
        {
            Complex zInv = Complex.One / z;
            Complex acc = Complex.Zero;
            Complex pow = Complex.One;
            for (int i = 0; i < coeffs.Length; i++)
            {
                acc += coeffs[i] * pow;
                pow *= zInv;
            }
            return acc;
        }

        /// <summary>
        /// Two-stage method (Van den Hof &amp; Schrama 1995) — closed-loop identification.
        ///
        /// ▣ Why
        ///   직접 (u, y) 식별은 controller 가 약/강에 따라 plant 추정이 변동 (model order
        ///   misspecification 때문). reference signal r 이 외부신호라는 점을 활용해
        ///   *closed-loop transfer* T(z)=y/r 를 먼저 식별 → 알려진 C(z) 로 G(z) 역산.
        ///   → 같은 plant 에 대해 controller 무관 같은 G → iteration 수렴.
        ///
        /// ▣ Stage 1: T(z) ARX(2,2) — r 이 외부 신호 → OLS unbiased (IV 불필요)
        ///   y[k] = a₁·y[k-1] + a₂·y[k-2] + b₀·r[k-1-δ] + b₁·r[k-2-δ] + ε
        ///   → T(z) = (b₀·z⁻¹ + b₁·z⁻²) / (1 - a₁·z⁻¹ - a₂·z⁻²)
        ///
        /// ▣ Stage 2: G(z) = T(z) / [C(z)·(1-T(z))] — closed-form
        ///   C(z) = K_p·(c₀ + c₁z⁻¹ + c₂z⁻²) / (1 - z⁻¹)   (backward Euler PID)
        ///   대입 정리:  G(z) = N_T(z)·D_C(z) / [N_C(z)·(D_T(z)-N_T(z))]
        ///   분자/분모 차수: N_G 차수 3, D_G 차수 4. dominant 2 pole = 진짜 plant pole.
        ///   나머지는 C 의 zero 와 cancel — N_G(pole) ≈ 0 으로 검출.
        /// </summary>
        private static PlantModel IdentifyPlantTwoStage(
            double[] y, double[] r, double dt, double theta,
            double kp, double ti, double td)
        {
            PlantModel m = new PlantModel { Theta = theta };
            int N = Math.Min(y.Length, r.Length);
            if (N < 64) { m.Diagnosis = "data too short"; return m; }

            int delayN = Math.Max(0, (int)Math.Round(theta / dt));
            int kStart = 2 + delayN;
            if (kStart >= N - 4) { m.Diagnosis = "delay > data"; return m; }

            double yStd = StdDev(y);
            if (yStd < 1e-4) { m.Diagnosis = "y barely moves — excitation too weak"; return m; }

            double[] yd = new double[N]; Array.Copy(y, yd, N); Detrend(yd);
            double[] rd = new double[N]; Array.Copy(r, rd, N); Detrend(rd);

            // ── Stage 1: T(z) ARX(3,3) OLS ──
            // 6×6 symmetric normal eq. r 은 외부 → noise ⟂ r → unbiased.
            //   적분기 plant + PID controller 의 closed-loop T 는 차수 3:
            //   T = CG/(1+CG), C ~ poly(2)/(1-z⁻¹), G_int ~ z⁻¹/(1-z⁻¹)
            //   → numerator/denominator 모두 3차 → ARX(3,3) 가 minimum.
            //   ARX(2,2) under-modeling 시 PID 강도에 따라 dominant mode 변동 → 수렴 깨짐.
            const int NA = 3, NB = 3;
            const int NP = NA + NB;  // 6 params
            int kStartReg = Math.Max(kStart, NA + NB + delayN);
            if (kStartReg >= N - 4) { m.Diagnosis = "delay/order > data"; return m; }

            double[,] sMat = new double[NP, NP];
            double[] tVec = new double[NP];
            int count = 0;
            for (int k = kStartReg; k < N; k++)
            {
                double yt = yd[k];
                double[] reg = {
                    yd[k-1], yd[k-2], yd[k-3],
                    rd[k-1-delayN], rd[k-2-delayN], rd[k-3-delayN]
                };
                for (int i = 0; i < NP; i++)
                {
                    tVec[i] += yt * reg[i];
                    for (int j = i; j < NP; j++)
                        sMat[i, j] += reg[i] * reg[j];
                }
                count++;
            }
            for (int i = 0; i < NP; i++)
                for (int j = 0; j < i; j++)
                    sMat[i, j] = sMat[j, i];
            if (count < 64) { m.Diagnosis = $"too few samples ({count})"; return m; }

            var Mreg = MB.DenseOfArray(sMat);
            var rhs = VB.DenseOfArray(tVec);

            MathNet.Numerics.LinearAlgebra.Vector<double> sol;
            try { sol = Mreg.Solve(rhs); }
            catch { m.Diagnosis = "T regression singular"; return m; }

            double aT1 = sol[0], aT2 = sol[1], aT3 = sol[2];
            double bT0 = sol[3], bT1 = sol[4], bT2 = sol[5];
            if (double.IsNaN(aT1) || double.IsNaN(aT2) || double.IsNaN(aT3)
                || double.IsNaN(bT0) || double.IsNaN(bT1) || double.IsNaN(bT2))
            { m.Diagnosis = "NaN in T fit"; return m; }

            // T 식별 잔차 (SE-게이트 용)
            double sqResid = 0; int cResid = 0;
            for (int k = kStartReg; k < N; k++)
            {
                double pred = aT1 * yd[k - 1] + aT2 * yd[k - 2] + aT3 * yd[k - 3]
                            + bT0 * rd[k - 1 - delayN] + bT1 * rd[k - 2 - delayN] + bT2 * rd[k - 3 - delayN];
                double e = yd[k] - pred;
                sqResid += e * e; cResid++;
            }
            m.FitRmse = Math.Sqrt(sqResid / Math.Max(1, cResid));

            // ── Stage 2: G(z) = N_G/D_G via polynomial arithmetic ──
            // C(z) = K_p · (c₀ + c₁ z⁻¹ + c₂ z⁻²) / (1 - z⁻¹)
            double tiSafe = (ti > 1e-6 && ti < 1e6) ? ti : 1e9;  // Ti off (==250) → 거의 무한
            double c0 = 1.0 + dt / tiSafe + td / dt;
            double c1Pid = -(1.0 + 2.0 * td / dt);
            double c2Pid = td / dt;

            double[] N_C = { kp * c0, kp * c1Pid, kp * c2Pid };
            double[] N_T = { 0.0, bT0, bT1, bT2 };
            double[] D_T = { 1.0, -aT1, -aT2, -aT3 };
            double[] DT_minus_NT = { 1.0, -aT1 - bT0, -aT2 - bT1, -aT3 - bT2 };
            double[] D_C = { 1.0, -1.0 };

            double[] N_G = PolyMult(N_T, D_C);              // 5 coeffs (z⁻⁰..z⁻⁴)
            double[] D_G = PolyMult(N_C, DT_minus_NT);      // 6 coeffs (z⁻⁰..z⁻⁵)

            double d0 = D_G[0];
            if (Math.Abs(d0) < 1e-15)
            { m.Diagnosis = "D_G leading coeff ≈ 0 (PID K_p ≈ 0?)"; return m; }

            // ── G(z) 의 극점: D_G(z⁻¹) = 0 의 z root, 5×5 companion matrix ──
            double cf1 = D_G[1]/d0, cf2 = D_G[2]/d0, cf3 = D_G[3]/d0, cf4 = D_G[4]/d0, cf5 = D_G[5]/d0;
            var Mc = MB.DenseOfArray(new double[,] {
                { -cf1, -cf2, -cf3, -cf4, -cf5 },
                {  1.0,  0.0,  0.0,  0.0,  0.0 },
                {  0.0,  1.0,  0.0,  0.0,  0.0 },
                {  0.0,  0.0,  1.0,  0.0,  0.0 },
                {  0.0,  0.0,  0.0,  1.0,  0.0 }
            });
            var eigs = Mc.Evd().EigenValues;

            // 5 root: spurious (N_G(z_pole)≈0, C zero 와 cancel) 제외 후 dominant 2 채택.
            var realPoles = new List<Complex>();
            double ngScale = Math.Max(1e-9, Math.Abs(bT0) + Math.Abs(bT1) + Math.Abs(bT2));
            for (int i = 0; i < 5; i++)
            {
                Complex p = eigs[i];
                Complex ngVal = PolyEvalAtZ(N_G, p);
                if (ngVal.Magnitude > 1e-4 * ngScale)
                    realPoles.Add(p);
            }
            if (realPoles.Count == 0)
                for (int i = 0; i < 5; i++) realPoles.Add(eigs[i]);

            realPoles.Sort((a, b) => b.Magnitude.CompareTo(a.Magnitude));

            Complex p1 = realPoles[0];
            Complex p2 = (realPoles.Count > 1) ? realPoles[1] : p1;
            double mag1 = p1.Magnitude;
            double mag2 = p2.Magnitude;

            if (mag1 > 2.0) { m.Diagnosis = $"G pole |z|={mag1:0.000} > 2 (unstable)"; return m; }
            if (mag1 <= 0) { m.Diagnosis = $"G pole |z|={mag1:0.000} invalid"; return m; }

            // ── DC gain K — multi-frequency Bode slope detection ──
            // 적분기 plant: |G(jω)| ∝ 1/ω → log slope = -1 → K_i = ω·|G(jω)|
            // stable plant: |G(jω)| → K (constant at low ω) → log slope ≈ 0 → K = |G|
            // 두 ω 에서 |G| 측정해서 slope 로 plant type 판정.
            double omega1 = 0.05, omega2 = 0.5;  // rad/s, well below typical FTD plant BW
            Complex z1 = Complex.FromPolarCoordinates(1.0, omega1 * dt);
            Complex z2 = Complex.FromPolarCoordinates(1.0, omega2 * dt);
            double gMag1 = (PolyEvalAtZ(N_G, z1) / PolyEvalAtZ(D_G, z1)).Magnitude;
            double gMag2 = (PolyEvalAtZ(N_G, z2) / PolyEvalAtZ(D_G, z2)).Magnitude;
            double slope = (gMag1 > 1e-12 && gMag2 > 1e-12)
                ? Math.Log(gMag2 / gMag1) / Math.Log(omega2 / omega1)
                : 0;

            bool integratorBySlope = slope < -0.5;  // slope -1 이면 정확히 적분기, -0.5 보수적 cutoff
            bool integratorByPole = (mag1 > 0.95 && Math.Abs(p1.Real - 1.0) < 0.1 && Math.Abs(p1.Imaginary) < 0.05);
            m.HasIntegrator = integratorBySlope || integratorByPole;
            if (mag1 > 1.0) m.HasIntegrator = true;

            if (m.HasIntegrator)
                m.K = omega1 * gMag1;  // K_i = ω·|G(jω)| (integrator: |G|·ω 가 상수)
            else
                m.K = gMag1;  // K = |G(low ω)| (stable: constant at DC)

            if (double.IsNaN(m.K) || double.IsInfinity(m.K) || m.K < 1e-9)
            { m.Diagnosis = $"K calc failed ({m.K})"; return m; }

            // Time constants
            double magForTau = Math.Min(mag1, 1.0 - 1e-6);
            m.Tau1 = -dt / Math.Log(magForTau);
            double mag2Clamped = Math.Max(1e-6, Math.Min(mag2, 1.0 - 1e-6));
            m.Tau2 = -dt / Math.Log(mag2Clamped);

            // ── SE 근사 (proxy) ──
            // T 의 dominant pole 로부터 τ SE 근사. ARX(3,3) 의 정확한 SE 는 복잡하므로 conservative.
            double sigma2 = m.FitRmse * m.FitRmse;
            double seA1 = 0;
            try {
                var Minv = Mreg.Inverse();
                seA1 = Math.Sqrt(Math.Abs(sigma2 * Minv[0, 0]));
            } catch { }
            // τ_1 SE 근사: ARX 의 dominant pole 의 magnitude SE → τ chain rule
            double dtau_dz = (mag1 > 1e-6 && mag1 < 1.0)
                ? dt / (mag1 * Math.Log(mag1) * Math.Log(mag1)) : 0;
            m.TauSE = Math.Abs(dtau_dz * seA1);

            m.Valid = true;
            string poleStr = (Math.Abs(p1.Imaginary) < 1e-4)
                ? $"|z|={mag1:0.000}"
                : $"|z|={mag1:0.000}∠{Math.Atan2(p1.Imaginary, p1.Real) * 180 / Math.PI:0}°";
            m.Diagnosis = m.HasIntegrator
                ? $"Two-stage ARX(3,3): integrator (slope={slope:0.00}), τ_2={m.Tau2:0.000}s K_i={m.K:0.000}"
                : $"Two-stage ARX(3,3): τ_1={m.Tau1:0.000}s τ_2={m.Tau2:0.000}s K={m.K:0.000} ({poleStr}, slope={slope:0.00})";
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

            // FTD slider 한계 (mandatory, not heuristic):
            //   FTD 의 Integral time slider 는 250 까지 (표현상 "off"). Td 는 10 까지.
            //   이건 게임 UI 의 강제 bound 라 적용 안 하면 슬라이더가 무시.
            if (r.Ti > 250.0) r.Ti = 250.0;
            if (r.Ti < 0.1) r.Ti = 0.1;
            if (r.Td < 0) r.Td = 0;
            if (r.Td > 10.0) r.Td = 10.0;

            return r;
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
        //  ▣ 단점: 참조모델 M(s) 의 T_s 선택이 결과 좌우 → Ts sweep 으로 자동 선정.
        // ════════════════════════════════════════════════════════════════════════

        private struct FritOptResult
        {
            public double Kp, Ti, Td;
            public double Cost;
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

        /// <summary>FRIT 비용 — mean square (y - ŷ). 포화 샘플은 cost 에서 제외.</summary>
        private static double FritCostEval(double kp, double ti, double td,
            double[] u, double[] y, bool[] sat, double ts, int nM, double tauM, double dt)
        {
            if (!IsInverseCStable(kp, ti, td, dt)) return 1e12;
            int N = u.Length;
            double[] e = InverseCFilter(u, kp, ti, td, dt);
            double[] rTilde = new double[N];
            for (int k = 0; k < N; k++) rTilde[k] = y[k] + e[k];
            double[] yHat = ApplyRefModel(rTilde, ts, nM, tauM, dt);
            int dropEdge = Math.Max(32, (int)Math.Round(2.0 * ts / dt));
            int kStart = dropEdge, kEnd = N - dropEdge;
            if (kEnd <= kStart) return 1e12;
            double sum = 0;
            int cnt = 0;
            for (int k = kStart; k < kEnd; k++)
            {
                // 포화 샘플 제외 — FTD 의 anti-windup 가 작동하지만 우리 1/C(z) inverse 가 LTI 가정.
                //   포화 영역은 PID 가 nonlinear (I clamped). cost 에 systematic error.
                if (sat != null && sat.Length > k && sat[k]) continue;
                double rr = y[k] - yHat[k];
                if (double.IsNaN(rr) || double.IsInfinity(rr)) return 1e12;
                sum += rr * rr;
                cnt++;
            }
            return (cnt > 0) ? sum / cnt : 1e12;
        }

        /// <summary>
        /// FRIT LM 1회 — 주어진 시드에서 LM 최적화. 비용/수렴 여부 반환.
        /// 포화 샘플은 LM residual 에서 mask (포화 동안 PID 가 nonlinear → 회귀 오염).
        /// </summary>
        private static FritOptResult RunFritLM(
            double[] u, double[] y, bool[] sat, double dt, double ts, int nM, double tauM,
            double kpInit, double tiInit, double tdInit)
        {
            FritOptResult r = new FritOptResult { Kp = kpInit, Ti = tiInit, Td = tdInit, Cost = 1e12 };
            int N = u.Length;
            int dropEdge = Math.Max(32, (int)Math.Round(2.0 * ts / dt));
            int kStart = dropEdge, kEnd = N - dropEdge;
            if (kEnd <= kStart) { r.Diag = "data too short"; return r; }

            // 포화 mask 적용한 인덱스 리스트
            var validIdx = new List<int>(kEnd - kStart);
            for (int k = kStart; k < kEnd; k++)
            {
                if (sat != null && sat.Length > k && sat[k]) continue;
                validIdx.Add(k);
            }
            int M = validIdx.Count;
            if (M < 32) { r.Diag = $"too few non-sat samples ({M})"; return r; }

            var obsX = VB.Dense(M, 0.0);
            var yObsArr = new double[M];
            for (int i = 0; i < M; i++) yObsArr[i] = y[validIdx[i]];
            var obsY = VB.DenseOfArray(yObsArr);

            Func<MathNet.Numerics.LinearAlgebra.Vector<double>,
                 MathNet.Numerics.LinearAlgebra.Vector<double>,
                 MathNet.Numerics.LinearAlgebra.Vector<double>> model = (theta, xUnused) =>
            {
                double kp = Math.Max(1e-4, theta[0]);
                double ti = Math.Max(1e-3, Math.Min(1e6, theta[1]));
                double td = Math.Max(0, Math.Min(100, theta[2]));

                if (!IsInverseCStable(kp, ti, td, dt))
                    return VB.Dense(M, 1e3);

                double[] e = InverseCFilter(u, kp, ti, td, dt);
                double[] rTilde = new double[N];
                for (int k = 0; k < N; k++) rTilde[k] = y[k] + e[k];
                double[] yHat = ApplyRefModel(rTilde, ts, nM, tauM, dt);
                var result = VB.Dense(M);
                for (int i = 0; i < M; i++)
                {
                    double yh = yHat[validIdx[i]];
                    if (double.IsNaN(yh) || double.IsInfinity(yh)) yh = 1e3;
                    result[i] = yh;
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
                r.Cost = FritCostEval(r.Kp, r.Ti, r.Td, u, y, sat, ts, nM, tauM, dt);
                r.Converged = (lmResult.ReasonForExit == MathNet.Numerics.Optimization.ExitCondition.Converged);
                r.Diag = r.Converged ? "converged" : "stopped";
            }
            catch (Exception ex)
            {
                r.Diag = "LM ex: " + ex.Message;
            }
            return r;
        }

        /// <summary>
        /// FRIT multistart — 현재 PID + 보수적 + 적분기/1차 가정 시드. 가장 cost 낮은 결과 채택.
        /// non-convex cost surface 의 local minimum 위험 완화.
        /// </summary>
        /// <summary>
        /// 27-grid multistart — PID 슬라이더 범위에서 log-spaced grid + 현재 PID = 28 seeds.
        ///   Kp ∈ {0.01, 0.1, 1.0}  (3 값, log-spaced)
        ///   Ti ∈ {1, 10, 100}      (3 값, log-spaced)
        ///   Td ∈ {0, 0.1, 1.0}     (3 값)
        ///   + 현재 PID 1 시드
        /// 학계: Levenberg-Marquardt 의 local minimum 함정 회피 — multistart 가 표준.
        /// </summary>
        private static FritOptResult RunFritMultistart(
            double[] u, double[] y, bool[] sat, double dt, double ts, int nM, double tauM,
            double kpCurr, double tiCurr, double tdCurr)
        {
            double[] kpGrid = { 0.01, 0.1, 1.0 };
            double[] tiGrid = { 1.0, 10.0, 100.0 };
            double[] tdGrid = { 0.0, 0.1, 1.0 };

            var seeds = new List<(double kp, double ti, double td)>(28);
            // 현재 PID 첫 시드
            seeds.Add((Math.Max(1e-3, kpCurr), Math.Max(0.1, tiCurr), Math.Max(0, tdCurr)));
            // 27 grid
            foreach (var kp in kpGrid)
                foreach (var ti in tiGrid)
                    foreach (var td in tdGrid)
                        seeds.Add((kp, ti, td));

            FritOptResult best = new FritOptResult { Cost = double.PositiveInfinity };
            foreach (var s in seeds)
            {
                var r = RunFritLM(u, y, sat, dt, ts, nM, tauM, s.kp, s.ti, s.td);
                if (r.Cost < best.Cost) best = r;
            }
            return best;
        }

        /// <summary>
        /// Ts sweep — log-spaced 후보들에 multistart FRIT 실행, safety check 통과한 가장 공격적 후보 채택.
        /// safety: Td/Ti &lt; 0.3, Td/dt &lt; 10 (D 진동 방지 — 산업 표준).
        /// 모두 fail 시 fallback (가장 cost 낮은, 보수적 선호).
        /// </summary>
        private FritOptResult RunFritTsSweep(double[] u, double[] y, bool[] sat, double dt, double tauM,
            double kpCurr, double tiCurr, double tdCurr, out double tsBest, out int safeCount)
        {
            int nM = 2;
            int[] dtMults = { 3, 5, 7, 10, 15, 20, 30, 50, 80, 200 };
            FritOptResult best = new FritOptResult { Cost = double.PositiveInfinity };
            FritOptResult fallback = new FritOptResult { Cost = double.PositiveInfinity };
            tsBest = 0;
            double tsFallback = 0;
            safeCount = 0;

            foreach (int m in dtMults)
            {
                double ts = Math.Max(3.0 * dt, Math.Min(5.0, m * dt));
                ts = Math.Max(ts, tauM);

                var r = RunFritMultistart(u, y, sat, dt, ts, nM, tauM, kpCurr, tiCurr, tdCurr);
                if (double.IsInfinity(r.Cost) || double.IsNaN(r.Cost)) continue;

                bool safe = (r.Td / Math.Max(r.Ti, 1e-6) < 0.3) && (r.Td / dt < 10);
                if (safe)
                {
                    safeCount++;
                    if (best.Cost == double.PositiveInfinity || r.Cost < best.Cost)
                    {
                        best = r;
                        tsBest = ts;
                    }
                }
                if (r.Cost < fallback.Cost)
                {
                    fallback = r;
                    tsFallback = ts;
                }
            }

            if (best.Cost == double.PositiveInfinity)
            {
                tsBest = tsFallback;
                return fallback;
            }
            return best;
        }

    }
}
