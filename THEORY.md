# PIDSupporter — 이론 & 학습 가이드

이 문서는 **공부용**. 한 번 읽으면 이 모드 안에서 무슨 일이 벌어지는지 다 파악할 수 있게 작성. 각 절은 **직관 → 수학 → 학계 출처 → 코드 위치** 순서.

---

## 0. 한 문장 요약

> 비행 중인 함체에 작은 무작위 신호를 넣어서 응답을 측정하고, 그 데이터로 PID 게인을 역산하는 모드. 알고리즘은 **FRIT** (Soma-Kaneko 2004) + **Welch coherence** (Bendat-Piersol 2010) + **multistart LM** 조합.

---

## 1. 우선 PID 가 뭔지부터

PID 제어기는 오차 e = (목표 - 현재값) 을 받아 제어신호 u 를 만드는 함수:

```
u(t) = Kp · e(t) + Ki · ∫e dt + Kd · de/dt
     = Kp · ( e + (1/Ti)·∫e dt + Td·de/dt )
```

- **Kp**: 비례 — 오차가 크면 강하게
- **Ti**: 적분 시정수 — 작을수록 누적 오차 빨리 잡음 (저주파 외란 제거)
- **Td**: 미분 시정수 — 클수록 변화율에 민감 (overshoot 방지)

**전달함수 형식**:

```
C(s) = Kp · ( 1 + 1/(Ti·s) + Td·s )
```

이 모드의 목적: **Kp, Ti, Td 를 데이터로부터 알아내기**.

---

## 2. 폐루프 시스템 식별이 왜 어려운가

### 기본 구조

```
        r           e          u           y
   ──→ (+) ──→ [ C(s) ] ──→ [ G(s) ] ──→ ┬──→
        ↑                                 │
        └─────────────────────────────────┘
```

- r: 목표값 (setpoint)
- C(s): 우리가 찾고 싶은 PID 제어기
- G(s): plant (비행기/배 등 물리)
- y: 출력 (실제값)

### 두 가지 ID 방식

**Open-loop ID** (쉬움): 
- C(s) 를 떼어내고 u 를 직접 정해서 G(s) 만 식별
- 위험: 제어 없으니 비행기 추락

**Closed-loop ID** (이 모드): 
- C(s) 가 살아있는 상태로 식별
- 비행기 안 추락 ✓
- **하지만** u 와 y 가 c-G loop 으로 얽혀 있어서 수학적으로 까다로움

### 핵심 어려움 두 개

**(a) 식별성 (identifiability)**:
- C 가 강하면 y 거의 안 흔들림 → 정보 없음
- C 가 약하면 잘 흔들리지만 비행기 불안정

**(b) Closed-loop 상관**:
- y 에 노이즈 있음 → u 가 그 노이즈에 반응 → u 와 노이즈 상관
- Open-loop 가정 ID 는 편향됨 (bias)

→ 이 두 문제를 **FRIT** 가 우회.

---

## 3. FRIT — Fictitious Reference Iterative Tuning

**출처**: Soma, S., Kaneko, O., Fujii, T. (2004) "A new method of controller parameter tuning based on input-output data – Fictitious Reference Iterative Tuning", *IFAC Proceedings*.

### 아이디어

**핵심 트릭**: G(s) 를 모델링하지 않고 PID 만 직접 찾는다.

데이터 (u, y, 현재 PID 게인 C₀) 가 있을 때:
- **가상 참조신호**: r̃(θ) = y + C(θ)⁻¹ · u
- 이건 "만약 PID 가 θ 였다면, 이 (u,y) 를 만든 reference 는 무엇이었을까?"
- 우리가 원하는 응답은 **참조 모델** M(s) 와 같아야 함:
  $$y = M(s) · r̃(θ)$$
  이게 정확히 만족되는 θ 가 desired PID.

### 비용 함수

```
J(θ) = || y - M(s)·r̃(θ) ||²
     = || y - M(s)·(y + C(θ)⁻¹·u) ||²
```

이걸 최소화하는 θ = (Kp, Ti, Td) 가 답.

### 참조 모델 M(s)

원하는 폐루프 응답 모양:

$$M(s) = \frac{e^{-s \tau_M}}{(1 + a_M \cdot s)^{n_M}}, \quad a_M = 0.2 \cdot T_s$$

- **τ_M**: 순수 지연 (dead-time). FTD 에선 ≈ 1 tick.
- **T_s**: 목표 정착시간. 작을수록 빠른 응답.
- **n_M**: 모델 차수 (2~4 sweep).
- 5·a_M = T_s 가 "5 시정수 = settling" 경험칙에서 옴.

### 왜 잘 작동하나

- G(s) 모델링 불필요 (모델 오차 자유)
- 폐루프 데이터 그대로 쓰니까 비행 중에도 안전
- 식별성: C(θ) 가 바뀌면서 r̃ 도 같이 바뀌어서 information 가 비선형적으로 들어옴

**코드 위치**: `FritTuningTab.cs` → `FritCostEval`, `RunFritLM`, `ApplyRefModel`, `InverseCFilter`

---

## 4. 가진 (excitation) — 어떻게 자극하나

식별하려면 신호를 흔들어야 함. 너무 약하면 정보 X, 너무 세면 비행기 X.

### PRBS — Pseudo-Random Binary Sequence

- LFSR (Linear Feedback Shift Register) 로 만드는 ±1 시퀀스
- 다항식 x¹⁰ + x⁷ + 1, period 2¹⁰ - 1 = 1023
- 광대역 (broadband) 자극 — 화이트노이즈와 비슷한 스펙트럼

**왜 PRBS 인가**:
- 결정론적 → 재현 가능
- 양극단 (±A) 만 → 같은 분산 대비 piecewise constant 신호 중 SNR 최대
- 학계 표준 (Söderström-Stoica §5.2)

### Hybrid 가진

**SP-direct** (메인): r 에 PRBS 더함 → 제어기 통해 plant 자극.

**u-direct** (additive perturbation): u 에 직접 PRBS 더함.
- C 가 강해서 SP-direct 만으론 plant 가 안 흔들릴 때 보완
- **headroom-bounded**: |u_inject| ≤ γ · (1 - |u_C|) — 액추에이터 포화 안 시키게
- 출처: Hjalmarsson (2005) "From experiment design to closed-loop control"

**Adaptive bit duration**:
- bit 길이 = bit_ticks × dt
- 짧음 → 고주파 자극 우세, 김 → 저주파 우세
- Welch 분석으로 "가장 정보 부족한 band" 식별 → 그 band 자극 강화

**HPF drift 제거**:
- 적분기 plant + finite-window PRBS → 누적 DC bias → 비행기 천천히 drift
- fc ≈ 0.01 Hz HPF → DC 만 제거, 동역학은 유지
- 출처: Ljung §13.5

**Saturation-aware amplitude**:
- 매 2초 sat rate 측정 → 목표 10-25% 유지하게 amp 조정
- 너무 낮음 → amp ↑, 너무 높음 → amp ↓
- 이론적 근거: 학계의 "input design with power constraint" 정통

**코드 위치**: `RecordingTick`, `ApplyExcitation`, `UpdateAdaptiveAmp`, `UpdateSpectralBitTicks`

---

## 5. Welch periodogram + Coherence — 데이터 품질 측정

매 6초 마다 누적 데이터를 분석:

### Welch periodogram

- 데이터를 256-sample segment 로 자름, 50% overlap
- 각 segment 에 Hanning window 적용 → FFT
- 여러 segment 평균 → 분산 감소
- 출처: Welch (1967) "The use of fast Fourier transform for the estimation of power spectra"

### Cross-spectrum

$$S_{yr}(f) = \langle Y(f) \cdot R^*(f) \rangle$$

y 와 r 사이의 주파수별 상관 (복소수, 진폭 + 위상).

### Coherence γ²(f)

$$\gamma^2(f) = \frac{|S_{yr}(f)|^2}{S_{yy}(f) \cdot S_{rr}(f)} \in [0, 1]$$

**해석**:
- γ² → 1: y 가 r 에 선형적으로 깨끗하게 반응 (식별 OK)
- γ² → 0: 노이즈가 dominate (식별 불가)
- 보통 > 0.7 이면 신뢰 가능

출처: Bendat & Piersol (2010) *Random Data*, §11.

### Sensitivity |S(jω)|

폐루프 sensitivity function:

$$S(j\omega) = \frac{1}{1 + C(j\omega) \cdot G(j\omega)} = 1 - T(j\omega)$$

T(j\omega) = S_yr/S_rr 로 추정 → |S| = |1 - T|.

**해석**:
- |S| 낮음 = controller 가 그 주파수에서 강함 (제어 잘 됨)
- |S| 높음 = controller 약함 (정보 풍부)
- 식별 관점에선 **|S| 큰 band 가 학습 기회**

3 band 로 평균:
- Low: 0.05–0.5 Hz (적분, 외란)
- Mid: 0.5–2 Hz (주요 동역학)
- High: 2–5 Hz (Td 영향)

**코드 위치**: `MeasureSensitivityAndCoherence`, `ComputeBandWelchSensitivity`

---

## 6. Sensitivity-weighted Cost

기본 FRIT cost 는 시간 영역 sum of squares. 그런데 모든 주파수가 동일하게 신뢰 가능한 건 아님 — γ² 낮은 대역은 노이즈만 fit 함.

### Band 별 가중 평균

Residual 을 FFT 해서 3 band 의 에너지 측정:

```
cost = (γ²_lo · E_lo + γ²_mid · E_mid + γ²_hi · E_hi) / (γ²_lo + γ²_mid + γ²_hi)
```

γ² 가 낮은 band 의 cost 영향력이 자동으로 줄어듦 → noise fitting 방지.

### Parseval 정리 등가성

시간 영역 sum of squares = 주파수 영역 sum of squares (Parseval). 그러니 band 별 합치는 게 수학적으로 의미 있음.

### 학계 출처

- Bendat-Piersol (2010): 표준 frequency-weighted MSE
- 더 공격적 weight: Fisher information ratio `γ²/(1-γ²)` (Ljung §6.3) — 이건 미적용 (현재 γ² 직접 사용)

**코드 위치**: `FritCostEval` (Welch periodogram on residual)

---

## 7. LM 최적화 + Multistart

비용 J(θ) 가 비선형 + 비볼록 (non-convex) → local minimum 위험.

### Levenberg-Marquardt

Gauss-Newton + gradient descent 의 하이브리드. Math.NET 의 `LevenbergMarquardtMinimizer` 사용.

각 LM 한 번에 30 iter, residual 길이 = 비포화 sample 수.

### Multistart 시드

9 시드:
- 1: 현재 PID 값
- 8: 2×2×2 grid corners 
  - Kp ∈ {0.05, 0.5}
  - Ti ∈ {1.0, 10.0}
  - Td ∈ {0.0, 1.0}

각 시드에서 LM 돌리고 cost 최저 채택. 학계 표준 (Söderström-Stoica §7.3).

**코드 위치**: `RunFritLM`, `RunFritMultistart`

---

## 8. nM × Ts Grid Sweep

참조 모델 M(s) 의 두 메타파라미터:
- **n_M**: 차수 (2, 3, 4)
- **T_s**: 정착시간 (0.1, 0.3, 1.0, 3.0, 10.0 초)

직관:
- nM = 2: plant 만 (적분기 없는 1차 + actuator 없음 가정)
- nM = 3: plant + actuator first-order lag
- nM = 4: cascaded (예: roll → pitch → yaw)
- T_s 작음: 빠른 응답 강요 (Td 커질 수 있음)
- T_s 큼: 부드러운 응답 (Td 작음)

### Sweep

```
for nM in {2, 3, 4}:
  for Ts in {0.1, 0.3, 1.0, 3.0, 10.0}:
    for seed in 9 seeds:
      run LM, record cost
  
pick (nM, Ts, θ) with lowest cost
```

총 = 3 × 5 × 9 = **135 LM evaluations ≈ 30 초**.

학계 정통: model order selection via cross-validation cost (Söderström-Stoica §8.4).

**코드 위치**: `RunFritFullSweep`

---

## 9. Cramér-Rao 표준오차 (SE)

LM 이 θ̂ 를 줬는데 — 얼마나 믿을 수 있나?

### Fisher Information

$$I(\theta) = \frac{1}{\sigma^2} J^T J$$

J = ∂yHat/∂θ (Jacobian), σ² = residual variance.

### Cramér-Rao Lower Bound

어떤 unbiased estimator 든 분산 ≥ I⁻¹. 우리는 등호 가까이라 가정:

$$\text{cov}(\hat\theta) \approx \sigma^2 \cdot (J^T J)^{-1}$$

$$\text{SE}_i = \sqrt{\text{cov}_{ii}}$$

### 95% 신뢰구간

Gaussian 가정에서 95% CI ≈ θ̂ ± 2·SE.

### UI 표시

```
Kp = 0.5000  ±0.0200  (4%)             ← 좋음
Ti = 5.20    ±2.10   (40%)  [low conf] ← 의심
Td = 0.30    ±0.45   (150%) [uncertain] ← 신뢰 X
```

### 학계 출처

- Ljung (1999) *System Identification: Theory for the User*, §9.4
- Söderström-Stoica (1989) *System Identification*, §7.4

**코드 위치**: `ComputeFritSE`

---

## 10. Iterative Tuning Pattern (IFT)

### 발견된 패턴

User 가 직접 본 흐름:
1. **Round 1**: 약한 PID → S_lo 높음 → 저주파 자극 우세 → Kp/Ti 정확, Td 발산
2. (수동) Td 줄임 → 적용
3. **Round 2**: 나은 PID → S_hi 높음 → 고주파 자극 우세 → Td 정확

### 이게 왜 일어나나

- Td 는 고주파에서만 영향 (Td·s 항)
- Round 1 의 약한 PID 는 고주파 자극 못 함 → Td 정보 X → LM 이 자유롭게 둠
- Round 2 의 나은 PID 는 고주파 자극 만들어줌 → Td 정확 식별

### 학계 출처

이게 정확히 **IFT (Iterative Feedback Tuning)** — Hjalmarsson (1994):
> "Iterative feedback tuning—an overview" — *Int. J. Adaptive Control and Signal Processing*

각 라운드는 "현재 약한 곳" 을 자극하고, 학습 누적은 다음 라운드의 시드로.

### 현재 구현

User 가 수동으로 반복. SE 표시로 "이 게인 못 믿음" 알려줌 → User 가 판단해서 재실행.

자동화 (1 클릭으로 반복) 는 미구현 — 추후 가능.

---

## 11. Bode 적분 정리 — 절대 못 이기는 법칙

### 정리

안정 plant + RHP zero 없음 가정:

$$\int_0^\infty \log|S(j\omega)| \, d\omega = 0$$

번역: **|S| 어디서든 내리면 다른 곳에서 반드시 올라옴**. 모든 ω 에서 |S| < 1 인 controller 는 물리적으로 불가능.

### 좋은 PID 의 |S| 모양

```
|S(jω)|
 │
1│ - - - - - - - - - - ╱──────  ← S_hi ≈ 1 (대역폭 밖)
 │              ╱─────
 │      ╱──────
0│ ──                          ← S_lo 낮음 (제어 강함)
 └─────────────────────→ ω
```

- 저주파 (S_lo 낮음): 제어 잘 됨 ✓
- 중주파 (S_mid 중간): trade-off zone
- 고주파 (S_hi ≈ 1): 대역폭 밖, 어쩔 수 없음 (= 정상)

User 가 본 "S_lo, S_mid 낮고 S_hi 높은 PID 가 좋은 PID" 라는 관찰은 **Bode 정리의 직접 결과**.

### 만약 S_hi 도 낮추려면?

- Td 매우 크게 (D 가 고주파 강하게)
- 결과: 센서 노이즈 증폭, 액추에이터 burnout, phase margin 잃음
- = 실제로는 나쁜 PID

### 학계 출처

Bode, H. (1945) *Network Analysis and Feedback Amplifier Design*, §11.5. 그 후 Doyle-Francis-Tannenbaum (1992) *Feedback Control Theory* 에서 현대화.

---

## 12. 전체 파이프라인 (요약)

```
[Auto Tune 클릭]
  │
  ├─ Phase 0 (3s, 가진 OFF):
  │    · |u|, sign change rate, sat rate 측정
  │    · Limit cycle / 지속 포화 → 실패 종료
  │    · y baseline 누적
  │
  ├─ Recording (최대 60s):
  │    매 틱:
  │      · Hybrid PRBS (SP-direct + u-direct headroom)
  │      · HPF DC 제거
  │      · (u, y, r, uInject, sat) 기록
  │    매 2초:
  │      · Sat rate 측정 → amp 자동 조정 (target 10-25%)
  │    매 6초:
  │      · Welch + Coherence + |S| 측정
  │      · 부족 band 자극 강화 (bit_ticks 조정)
  │      · LastCohLo/Mid/Hi 저장
  │
  ├─ FRIT Sweep:
  │    for nM ∈ {2, 3, 4}:
  │      for Ts ∈ {0.1, 0.3, 1.0, 3.0, 10.0}:
  │        9-seed multistart LM
  │          · 비용 = band-weighted Welch on residual
  │          · SE = √diag(σ²·(JᵀJ)⁻¹)
  │    pick (nM, Ts, θ) with lowest cost
  │
  └─ Result:
       · _s.SettlingTimeTs ← best Ts (슬라이더 반영)
       · _s.ModelOrderNm   ← best nM (슬라이더 반영)
       · UI: Kp ± KpSE, Ti ± TiSE, Td ± TdSE
       · [uncertain] flag if SE/|val| > 50%
```

---

## 13. 코드 위치 빠른 색인

| 기능 | 위치 |
|------|------|
| Auto Tune 진입점 | `FritTuningTab.cs` → `AutoTuneCompute` |
| Compute 버튼 | `ComputeNow` |
| 진단 phase | `OnDiagnoseTick` |
| 데이터 수집 + 가진 | `RecordingTick`, `ApplyExcitation` |
| Sat-aware amp | `UpdateAdaptiveAmp` |
| Welch + Coherence | `MeasureSensitivityAndCoherence`, `ComputeBandWelchSensitivity` |
| FRIT cost (band-weighted) | `FritCostEval` |
| Inverse C filter | `InverseCFilter` |
| Reference model M(s) | `ApplyRefModel` |
| LM 1회 + SE | `RunFritLM`, `ComputeFritSE` |
| Multistart | `RunFritMultistart` |
| Full sweep | `RunFritFullSweep` |
| u-direct 가진 injection | `VariableControllerOutputPatch.cs` (Harmony) |

---

## 14. 학계 참고문헌

- **Bendat & Piersol (2010)**. *Random Data: Analysis and Measurement Procedures*, 4th ed. Wiley. — Welch, coherence, sensitivity.
- **Bode (1945)**. *Network Analysis and Feedback Amplifier Design*. Van Nostrand. — Sensitivity integral.
- **Doyle, Francis, Tannenbaum (1992)**. *Feedback Control Theory*. Macmillan. — 현대 sensitivity 분석.
- **Forssell & Ljung (1999)**. "Closed-loop identification revisited", *Automatica* 35(7). — 폐루프 식별성.
- **Hjalmarsson (1994)**. "Iterative feedback tuning—an overview", *Int. J. Adapt. Control Signal Process*. — IFT.
- **Hjalmarsson (2005)**. "From experiment design to closed-loop control", *Automatica* 41. — Headroom-bounded input.
- **Ljung (1999)**. *System Identification: Theory for the User*, 2nd ed. Prentice Hall. — 표준 교재.
- **Söderström & Stoica (1989)**. *System Identification*. Prentice Hall. — Multistart, model order.
- **Soma, Kaneko, Fujii (2004)**. "A new method of controller parameter tuning based on input-output data – FRIT", *IFAC Proc.* — FRIT 원논문.
- **Welch (1967)**. "The use of fast Fourier transform for the estimation of power spectra", *IEEE Trans. Audio Electroacoustics*. — Welch periodogram.

---

## 15. 더 발전시킬 수 있는 부분

- **Iterative auto-tune** (자동 반복): User 의 manual round 2 를 자동화 — Hjalmarsson IFT 정통.
- **Optimal input design** (D-optimal): 현재 PRBS → information matrix 최대화하는 input 으로.
- **Residual whitening test**: Ljung-Box 등 — 모델이 진짜 맞는지 통계적 검정.
- **Gain/Phase margin 계산**: Robust stability 보장.
- **Multi-axis joint ID**: Roll + Pitch 동시 식별 (cross-coupling 명시 모델링).
