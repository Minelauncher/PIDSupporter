# PIDSupporter — 이론 & 학습 가이드

이 문서는 **공부용**. 한 번 읽으면 이 모드 안에서 무슨 일이 벌어지는지 다 파악할 수 있게 작성. 각 절은 **직관 → 수학 → 학계 출처 → 코드 위치** 순서.

---

## 0. 한 문장 요약

> 비행 중인 함체에 작은 무작위 신호를 넣어서 응답을 측정하고, 그 데이터로 PID 게인을 역산하는 모드. 두 정통 알고리즘 제공:
>
> 1. **FRIT** (Soma-Kaneko 2004) + **γ²-weighted tracking cost** (Bendat-Piersol) + **closed-loop bandwidth Ts** (Skogestad-Postlethwaite) + **9-seed multistart LM** + **Skogestad Td realizability cap** (Skogestad 2003) — 정밀 식별
> 2. **Relay Feedback** (Åström-Hägglund 1984) + **Ziegler-Nichols** (1942) — 빠른 baseline tuning, 산업 표준 #1

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

### 이산 시간 IIR 구현 (backward Euler)

FRIT cost J(θ) 평가하려면 r̃(θ) 를 시간영역에서 계산. C(s) 와 M(s) 둘 다 IIR 로 이산화 필요.

**1/C(z) 유도**:

연속시간 C(s) 를 분수로 정리:

$$C(s) = K_p \cdot \frac{T_i T_d s^2 + T_i s + 1}{T_i s}$$

backward Euler 치환 $s \to (1 - z^{-1})/dt$ 후:

$$\frac{1}{C(z^{-1})} = \frac{1}{K_p} \cdot \frac{T_i \cdot dt \cdot (1 - z^{-1})}{a_0 + a_1 z^{-1} + a_2 z^{-2}}$$

where:

$$a_0 = 1 + \frac{dt}{T_i} + \frac{T_d}{dt}, \quad a_1 = -\left(1 + \frac{2 T_d}{dt}\right), \quad a_2 = \frac{T_d}{dt}$$

차분 방정식 ($e[k] = (1/C(z)) \cdot u[k]$):

$$a_0 \cdot e[k] + a_1 \cdot e[k-1] + a_2 \cdot e[k-2] = \frac{1}{K_p} \cdot (u[k] - u[k-1])$$

→ $e[k]$ 에 대해 풀어:

$$e[k] = \frac{1}{a_0} \left( \frac{u[k] - u[k-1]}{K_p} - a_1 \cdot e[k-1] - a_2 \cdot e[k-2] \right)$$

**1/C(z) 안정성**:

1/C(z) 의 poles = $a_0 z^2 + a_1 z + a_2 = 0$ 의 root. 단위원 안에 있어야 안정.

- disc = $a_1^2 - 4 a_0 a_2 \ge 0$ (실수 root): 둘 다 $|z| < 1$ 확인
- disc < 0 (복소 conjugate pair): $|z|^2 = a_2 / a_0 < 1$ 확인

불안정 (Kp, Ti, Td) 조합 → LM 평가 실패 → multistart 에서 시드 skip (휴리스틱 penalty 없이 NaN 반환).

**M(z) 이산화**:

$M(s) = e^{-s \tau_M} / (1 + a \cdot s)^{n_M}$, where $a = 0.2 \cdot T_s$.

각 first-order pole $1/(1 + a s)$ 를 backward Euler 로:

$$\frac{1}{1 + a s} \to \frac{1 - \alpha}{1 - \alpha z^{-1}}, \quad \alpha = \frac{a}{a + dt}$$

= 표준 first-order IIR low-pass:

$$y[k] = \alpha \cdot y[k-1] + (1 - \alpha) \cdot u[k]$$

$n_M$ 번 cascade 해서 M(z) 완성. 순수 지연 $e^{-s \tau_M}$ 는 $\lfloor \tau_M / dt \rfloor$ 번 $z^{-1}$ 곱하기 (FTD 에서 보통 1 tick).

**코드 위치**: `FritTuningTab.cs` → `FritCostBreakdown`, `RunFritLM`, `ApplyRefModel`, `InverseCFilter`, `IsInverseCStable`

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

## 6. γ²-weighted Cost (per-bin Welch)

기본 FRIT cost 는 시간 영역 sum of squares. 그런데 모든 주파수가 동일하게 신뢰 가능한 건 아님 — γ² 낮은 대역은 노이즈만 fit 함.

### 현재 cost form

residual r(k) = y(k) - M(s)·r̃(θ)(k) 를 Welch FFT 로 per-bin power 추정 (|R(f_i)|²) 후 γ² 가중합:

```
J(θ) = Σ γ²(f_i) · |R(f_i, θ)|²  /  Σ γ²(f_i)
```

- γ²(f_i) ≈ 1 (signal band, 보통 저주파): tracking 항이 강하게 fit 강요 → Ti / Kp 같은 저주파 파라미터 정확
- γ²(f_i) ≈ 0 (noise band, 보통 고주파): tracking 항이 자동으로 noise fitting 무시 → robust
- 정규화 Σγ² → 가중평균 → 데이터 magnitude 와 무관

### Parseval 정리 등가성

시간 영역 sum of squares = 주파수 영역 sum of squares (Parseval). 그러니 per-bin 합 계산이 수학적으로 의미 있음 (시간영역 MSE 의 일반화).

### 시행착오 기록 (penalty 항)

세션 중에 controller-energy penalty 항을 시도:

1. **per-bin (1-γ²)·|C|² penalty** (Bazanella §5.3.2): residual 과 controller 분리, 고주파 |C| 억제로 Td drift 차단 의도. 결과: noise band penalty 가 고주파 편향 → Td 과보호하면서 Ti 무방어.
2. **ω² weighting (F2)** (Bazanella §5.3.3): Ti/Td 양방향 절제 시도. 결과: Td 과억제 (Td=0 폭살).
3. **Tikhonov 정통 L-curve λ 선택** (Hansen 1992): augmented LM 의 residual vector 에 penalty rows 증강 + λ 자동 결정. 결과: PRBS-excited FRIT 데이터의 cost surface 가 log T 변동(0.24) vs log P (5.16) = 21:1 → 거의 수직 직선 → corner detection 이 boundary 로 끌려서 PI 구조 collapse 야기.

→ 결론: **penalty 항 전부 폐기**. tracking only + 사후 Skogestad cap (§9) 가 가장 정통 + 실용 균형.

### 학계 출처

- Bendat-Piersol (2010) §11.4: 표준 frequency-weighted MSE
- Fisher information ratio `γ²/(1-γ²)` (Ljung §6.3) — 더 공격적이지만 noise band 에서 0/0 불안정 → 미적용

**코드 위치**: `FritCostBreakdown` (Welch periodogram on residual + γ² weighting)

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

## 8. Closed-loop Bandwidth Ts + nM Sweep

참조 모델 M(s) 의 두 메타파라미터:
- **n_M**: 차수 (2, 3, 4) — sweep
- **T_s**: 정착시간 — **데이터에서 자동 산출** (sweep 안 함)

### 왜 Ts sweep 폐기

초기 구현: Ts ∈ {0.1, 0.3, 1.0, 3.0, 10.0} × nM × 9 seeds = 135 LM.

발견된 문제: cost(θ; Ts) 가 Ts 축 cross-comparison 시 구조적 bias 발생.
- Ts → ∞ 면 M(s) bandwidth ≈ 0 → residual ≈ y → tracking θ-flat → penalty 최소 trivial solution 으로 polarization
- Ts → dt 면 M(s) bandwidth ≈ Nyquist → 비현실
- Sweep 의 winner 가 데이터의 실제 plant 특성 무관하게 cost 의 구조적 minimum 으로 끌림

→ 해결: Ts 를 **데이터에서 직접 산출** (sweep 제거).

### Closed-loop bandwidth ω_B (Skogestad-Postlethwaite §2.4.5)

폐루프 전달 함수 T(jω) = S_yr(jω) / S_rr(jω) (Bendat-Piersol §6.4 H₁ estimator).

ω_B = first ω where |T(jω)|² ≤ 1/2.

이 정의는 closed-loop bandwidth 의 표준 정의 (3dB 점). 데이터 → ω_B 직접 추정.

### Bartlett 3-bin band averaging

|T(jω_i)|² 의 Welch 추정은 noise variance ~ (1-γ²)/(K·γ²) 가짐. variance 줄이려고:

```
|T̄(jω_i)|² = ( |T(ω_{i-1})|² + |T(ω_i)|² + |T(ω_{i+1})|² ) / 3
```

3-bin 평균으로 noise variance 1/3 감소 (Bendat-Piersol §11.5). "3" = 최소 non-trivial 중앙 평균 (current + 양쪽 이웃) — 임의 숫자 아님.

ω_B = first ω where |T̄(jω)|² ≤ 0.5.

### M(s) bandwidth matching → Ts 공식

|M(jω_B)|² = 1/(1 + (0.2·Ts·ω_B)²)^nM = 1/2 풀어서:

$$T_s = \frac{\sqrt{2^{1/n_M} - 1}}{0.2 \cdot \omega_B}$$

각 nM 별로 다른 Ts. M(s) bandwidth = ω_B 매칭하도록.

### Sweep (nM 만)

```
for nM in {2, 3, 4}:
  Ts(nM) = √(2^(1/nM)-1) / (0.2·ω_B)   ← 데이터에서
  9-seed multistart LM with this Ts, nM
  record cost
  
pick nM with lowest cost
```

총 = 3 × 9 = **27 LM evaluations ≈ 5 초** (이전 135 의 1/5).

### 전제 (정직 보고)

- 적분 작용 있는 PID (Ti < ∞) → |T(0)| ≈ 1 (closed-loop DC tracking 보장)
- |T(jω)| 단조 감소 (resonance peak 없음)
- ω_B 가 관측 band 안 [ω_1, ω_Nyquist] (= 1/(SEG·dt) ~ fs/2)

위반 시 ω_B 미발견 → 메시지 "increase starting PID gain" 으로 사용자 안내. 폴백 없음.

학계 정통: Skogestad-Postlethwaite (2005) §2.4.5 + Bendat-Piersol (2010) §11.5.

**코드 위치**: `EstimateTsFromClosedLoopBandwidth`, `RunFritBandwidthSweep`

---

## 9. Skogestad Td Cap (Realizability + Timescale)

### |C(jω)|² closed form

cap 분석 + cost penalty 분석에 PID 의 frequency response 가 필요. 유도:

$$C(j\omega) = K_p \left( 1 + \frac{1}{j \omega T_i} + j \omega T_d \right) = K_p + j K_p \left( \omega T_d - \frac{1}{\omega T_i} \right)$$

magnitude squared:

$$|C(j\omega)|^2 = K_p^2 \left( 1 + \left( \omega T_d - \frac{1}{\omega T_i} \right)^2 \right)$$

특성:
- 저주파 (ω → 0): $|C|^2 \approx K_p^2 / (\omega T_i)^2 \to \infty$ — integrator 영역
- 고주파 (ω → ∞): $|C|^2 \approx K_p^2 (\omega T_d)^2 \to \infty$ — derivative 영역
- 최소 at $\omega^* = 1/\sqrt{T_i T_d}$ where $|C|^2 = K_p^2$

이 닫힌형이 §6 의 cost surface 분석 (Td drift 원인) + 본 절 cap 설계의 기반.

### 왜 cap 필요한가

LM 이 tracking 만 최적화하면 **Td drift** 가 흔함:
- Td 는 미분 작용 — 고주파 noise 에 민감
- cost surface 가 Td 축으로 종종 평평 (위 |C|² 닫힌형에서 ω 작을 때 Td 항 미미) → noise 따라 drift → Td 과대
- 사용자 경험 + 학계 일치: Kp/Ti 는 저주파 dominant 라 안정, Td 만 drift 빈번

→ 사후 cap 으로 차단.

### 두 정통 cap 결합

$$T_d \le \min\left( \frac{T_i}{4}, \; \frac{1}{\omega_B} \right)$$

**(1) Skogestad SIMC realizability** (Skogestad 2003 §4.2):
- Td > Ti/4 면 controller zero 가 unstable region → noise 증폭
- derivative filtering 의 산업 표준 한계
- Ti 가 정상 범위 (0.5~10) 일 때 binding

**(2) Closed-loop timescale matching** (Skogestad-Postlethwaite §2.4.5):
- 1/ω_B = 닫힌루프 timescale
- Td > 1/ω_B 는 "D 작용이 폐루프 보다 느림" — 물리적으로 over-reach
- Ti 가 매우 큼 (≥100, I-off case) 일 때 binding

### 어느 cap 이 효력 발휘하나

| 케이스 | Ti | ω_B | Ti/4 | 1/ω_B | 효력 cap |
|--------|-----|-----|------|-------|---------|
| Relay → FRIT (정상 Ti) | 0.7 | 5.9 | 0.175 | 0.17 | 비슷 (두 정통 일치) |
| FRIT only, Ti=250 (I-off) | 250 | 4.9 | 62.5 | 0.20 | **1/ω_B** (Ti/4 무력) |
| 함선 롤 (느린 응답) | 5 | 1.0 | 1.25 | 1.0 | **1/ω_B** |
| 빠른 비행기 | 5 | 10 | 1.25 | 0.10 | **1/ω_B** |
| 보수 PID (작은 Ti) | 1 | 4 | 0.25 | 0.25 | 비슷 |

### 왜 Td 만 cap, Kp/Ti 안 cap

- Kp / Ti 는 저주파 dominant → 데이터 fit 이 strong 하게 결정 → drift 적음
- Td 만 cost surface 평평 → noise sensitive
- Skogestad cap 자체가 noise filtering 정신 — Td 만 제약

### 학계 출처

- Skogestad (2003) "Simple analytic rules for model reduction and PID controller tuning", *J. Process Control* 13, §4.2 — SIMC realizability
- Skogestad & Postlethwaite (2005) *Multivariable Feedback Control* §2.4.5 — closed-loop bandwidth

**코드 위치**: `RunFritMultistart` 의 사후 cap 블록

---

## 10. Quick Tune — Relay Feedback (Åström-Hägglund + ZN)

FRIT 와 다른 정통 접근. 산업에서 가장 많이 쓰이는 PID auto-tuning. 두 핵심 논문:

- **Åström, Hägglund (1984)**. "Automatic tuning of simple regulators with specifications on phase and amplitude margins". *Automatica* 20(5), 645-651 — relay feedback 측정법
- **Ziegler, Nichols (1942)**. "Optimum settings for automatic controllers". *ASME Trans.* 64, 759-768 — (K_c, T_c) → PID 공식

### 기본 원리

```
원래:   r → PID → u → plant → y
                 ↑___________|

Quick:  r → ±h relay → u → plant → y
                      ↑___________|
```

PID 를 일시적으로 ±h relay 로 교체:
- y > SP → u = -h
- y < SP → u = +h

→ plant dynamics 에 따라 y 가 SP 주변에서 **자연 진동 (limit cycle)** 형성.

### Limit cycle 의 수학 — Describing Function

비선형 relay 를 sinusoidal 입력 $A \sin(\omega t)$ 에 대한 **fundamental harmonic** 으로 근사 (높은 harmonic 무시):

$$\text{relay output}_1(t) \approx \frac{4h}{\pi} \sin(\omega t)$$

→ 등가 게인 (describing function):

$$N(A) = \frac{4h}{\pi A}$$

여기서 A 는 limit cycle 의 y 진폭.

### Limit Cycle 조건

closed-loop 에서 진동이 유지되려면 loop transfer 가 -1 점 지나야 (Nyquist 임계):

$$N(A) \cdot G(j\omega) = -1$$

이는 두 조건으로 분해:

$$|G(j\omega)| \cdot N(A) = 1, \qquad \angle G(j\omega) = -180°$$

위상이 -180° 가 되는 주파수 = **critical frequency ω_c**. 그때:

$$K_c = \frac{1}{|G(j\omega_c)|} = N(A) = \frac{4h}{\pi A}$$

→ **A (진폭) 측정만으로 K_c 직접 산출**. plant 모델링 없이.

T_c = 진동 주기 = $2\pi / \omega_c$.

### Ziegler-Nichols PID 공식 (1942)

(K_c, T_c) 측정 후:

$$K_p = 0.6 \cdot K_c, \qquad T_i = 0.5 \cdot T_c, \qquad T_d = 0.125 \cdot T_c$$

이 비율은 ZN 의 경험적 최적화 — **1/4 amplitude decay** 목표 (한 cycle 마다 진동 진폭이 25% 로 감소). 학계 비판: damping ratio ζ ≈ 0.21 이라 약간 진동적.

### Hysteresis ε (noise robustness)

순수 relay 는 noise 가 있으면 SP 근처에서 fast switching → 부정확. hysteresis 로 noise 거부:

- y > SP + ε → u = -h
- y < SP - ε → u = +h
- 그 사이 → u 이전 값 유지

이때 describing function 이 복소수로 변함:

$$N(A, \epsilon) = \frac{4h}{\pi A} \sqrt{1 - (\epsilon/A)^2} - j \frac{4h\epsilon}{\pi A^2}$$

복소 게인 → critical frequency 가 약간 이동. ε ≪ A 면 순수 relay 와 근사.

### 안전성 비교 (open-loop ZN vs relay)

기존 ZN (1942 원본) 은 Kp 를 키우면서 진동 한계 측정 → **Kp 한계 도달 시 plant 발산 위험**.

Åström-Hägglund 의 relay 변형은 u 가 ±h 로 한정 → **plant 가 안정한 limit cycle 형성** → 발산 없음. 이게 산업 표준 #1 인 이유.

| 측면 | Open-loop ZN | Relay (closed-loop) |
|------|--------------|---------------------|
| u 범위 | 무제한 (Kp 키우는 도중) | ±h 한정 |
| 함체 거동 | Kp 한계 도달 시 발산 | 안정한 limit cycle |
| 위험 | 높음 | 낮음 |

### Quick Tune 의 실제 단계 (이 모드)

1. **Diagnose (3s)**: 가진 OFF, 현재 PID 의 limit cycle 가능성 확인 (포화, sign change 등)
2. **Warm-up (~2 cycles)**: relay 활성화 → 초기 transient 무시 (PID → relay 전환 충격)
3. **Measure (≥3 cycles)**: A, T 측정 후 평균 (variance 감소)
4. **Compute**: $K_c = 4h/(\pi A)$, $T_c = T$ → ZN 공식 → Kp/Ti/Td

전체 ~30 초.

### Quick Tune 의 한계 (정직 평가)

- **ZN 공식은 보편적 최적 아님**: 1/4 decay 가 너무 진동적이라는 비판 (Skogestad SIMC 등 더 보수적 변형 존재). 다만 baseline 으로는 충분.
- **단일 주파수만 식별**: critical point ($\omega_c$, $K_c$) 두 정보. 전체 frequency response 미식별.
- **함체 진동**: FRIT 보다 크게 흔들림 (limit cycle 진폭 A 가 visible). 작은 함체에 부담.
- **Td = 0.125·T_c 일률**: 플랜트별 적정 Td 와 다를 수 있음.

### FRIT vs Relay 비교

| 측면 | FRIT | Quick Tune (Relay) |
|------|------|---------------------|
| 데이터 방식 | broadband PRBS | limit cycle 단일 주파수 |
| 식별 정보 | 전체 frequency response (Welch) | $(K_c, T_c)$ 두 점 |
| 모델 가정 | 참조 모델 M(s) 필요 | 없음 (등가 게인만) |
| 시간 | ~60s | ~30s |
| 안전성 | 약한 가진, 안전 | 함체 큰 진동 |
| 결과 정밀도 | 높음 (다변량 LM + multistart) | 보수 (ZN 일률 공식) |
| 적용 | 정밀 tuning | 빠른 baseline |

**권장 워크플로**: Quick Tune 으로 빠른 baseline → Apply → FRIT 로 정밀화. 두 알고리즘 장점 결합. 사용자 도메인 지식 + 데이터 기반 식별 + 산업 표준 안전.

**코드 위치**: `FritTuningTab.cs` → `QuickTuneNow` (진입점), `OnQuickTuneTick` (상태 머신), `ComputeRelayTune` (K_c, T_c → ZN), `RelayOutputInjector` (`VariableControllerOutputPatch.cs`)

---

## 11. Cramér-Rao 표준오차 (SE)

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

## 12. Iterative Tuning Pattern (IFT)

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

## 13. Bode 적분 정리 — 절대 못 이기는 법칙

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

## 14. 전체 파이프라인 (요약)

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
  │      · LastCohLo/Mid/Hi + per-bin S_rr/S_yr 저장
  │
  ├─ Compute (수집 완료 후):
  │    · Closed-loop bandwidth ω_B from |T̄|² (Bartlett 3-bin)
  │    · for nM ∈ {2, 3, 4}:
  │        Ts(nM) = √(2^(1/nM)-1) / (0.2·ω_B)   ← 데이터에서 자동
  │        9-seed multistart LM
  │          · 비용 = γ²-weighted Welch on residual
  │          · SE = √diag(σ²·(JᵀJ)⁻¹)
  │      pick nM with lowest cost
  │    · Skogestad cap: Td ≤ min(Ti/4, 1/ω_B)
  │
  └─ Result:
       · _s.SettlingTimeTs ← derived Ts (슬라이더 반영)
       · _s.ModelOrderNm   ← best nM (슬라이더 반영)
       · UI: Kp ± KpSE, Ti ± TiSE, Td ± TdSE
       · [uncertain] flag if SE/|val| > 50%
       · cap 활성 시 메시지 "Td cap (X→Y by Ti/4 or 1/ω_B)"
```

---

## 15. 코드 위치 빠른 색인

| 기능 | 위치 |
|------|------|
| **FRIT (Auto Tune)** | |
| Auto Tune 진입점 | `FritTuningTab.cs` → `AutoTuneCompute` |
| Compute 버튼 | `ComputeNow` |
| 진단 phase | `OnDiagnoseTick` |
| 데이터 수집 + 가진 | `OnUiFixed` 의 recording 분기, `ApplyExcitation` |
| Welch + Coherence | `UpdatePrbsBitTicksFromSpectrum` (per-bin S_rr/S_yr 저장 포함) |
| 폐루프 BW Ts 추정 | `EstimateTsFromClosedLoopBandwidth` |
| nM sweep + Skogestad cap | `RunFritBandwidthSweep`, `RunFritMultistart` |
| FRIT cost (γ²-weighted) | `FritCostBreakdown` |
| Inverse C filter | `InverseCFilter` |
| Reference model M(s) | `ApplyRefModel` |
| LM 1회 + SE | `RunFritLM`, `ComputeFritSE` |
| u-direct 가진 injection | `VariableControllerOutputPatch.cs` (Harmony) |
| **Quick Tune (Relay)** | |
| Quick Tune 진입점 | `FritTuningTab.cs` → `QuickTuneNow` |
| Relay 상태 머신 (warm-up + measure) | `OnQuickTuneTick` |
| K_c, T_c → ZN PID 계산 | `ComputeRelayTune` |
| Relay 출력 injection (PID 일시 교체) | `RelayOutputInjector` in `VariableControllerOutputPatch.cs` |

---

## 16. 학계 참고문헌

- **Åström, Hägglund (1984)**. "Automatic tuning of simple regulators with specifications on phase and amplitude margins", *Automatica* 20(5), 645-651. — Relay feedback auto-tuning, describing function 분석.
- **Bendat & Piersol (2010)**. *Random Data: Analysis and Measurement Procedures*, 4th ed. Wiley. — Welch, coherence, sensitivity.
- **Bode (1945)**. *Network Analysis and Feedback Amplifier Design*. Van Nostrand. — Sensitivity integral.
- **Doyle, Francis, Tannenbaum (1992)**. *Feedback Control Theory*. Macmillan. — 현대 sensitivity 분석.
- **Forssell & Ljung (1999)**. "Closed-loop identification revisited", *Automatica* 35(7). — 폐루프 식별성.
- **Hjalmarsson (1994)**. "Iterative feedback tuning—an overview", *Int. J. Adapt. Control Signal Process*. — IFT.
- **Hjalmarsson (2005)**. "From experiment design to closed-loop control", *Automatica* 41. — Headroom-bounded input.
- **Ljung (1999)**. *System Identification: Theory for the User*, 2nd ed. Prentice Hall. — 표준 교재.
- **Skogestad (2003)**. "Simple analytic rules for model reduction and PID controller tuning", *J. Process Control* 13. — SIMC PID realizability cap (Td ≤ Ti/4).
- **Skogestad & Postlethwaite (2005)**. *Multivariable Feedback Control: Analysis and Design*, 2nd ed. Wiley. — Closed-loop bandwidth 정의 (|T|²=0.5), 1/ω_B timescale.
- **Söderström & Stoica (1989)**. *System Identification*. Prentice Hall. — Multistart, model order.
- **Soma, Kaneko, Fujii (2004)**. "A new method of controller parameter tuning based on input-output data – FRIT", *IFAC Proc.* — FRIT 원논문.
- **Welch (1967)**. "The use of fast Fourier transform for the estimation of power spectra", *IEEE Trans. Audio Electroacoustics*. — Welch periodogram.
- **Ziegler, Nichols (1942)**. "Optimum settings for automatic controllers", *ASME Trans.* 64, 759-768. — (K_c, T_c) → PID 게인 공식.

---

## 17. 더 발전시킬 수 있는 부분

- **Iterative auto-tune** (자동 반복): User 의 manual round 2 를 자동화 — Hjalmarsson IFT 정통.
- **Optimal input design** (D-optimal): 현재 PRBS → information matrix 최대화하는 input 으로.
- **Residual whitening test**: Ljung-Box 등 — 모델이 진짜 맞는지 통계적 검정.
- **Gain/Phase margin 계산**: Robust stability 보장.
- **Multi-axis joint ID**: Roll + Pitch 동시 식별 (cross-coupling 명시 모델링).
