# PID 자동 튜너 — 이론 및 구현

---

## 1. 개요

이 모드는 **Hybrid PRBS 가진 + Welch/Coherence sensitivity 분석 + FRIT (28-seed multistart) PID 식별** 으로 PID 자동 튜닝을 수행합니다. 모든 단계가 학계 표준 + 명확한 수학적 근거.

**핵심 파이프라인**:

```
[Auto Tune]
  → Diagnose Phase 0 (3s, 가진 OFF)
       · Limit cycle / 지속 포화 검출 → fail
       · y baseline 측정

  → Diagnose Phase 1 (3s, 작은 perturbation amp=0.05)
       · tracking_ratio 측정 (정보 표시만, 자동 종료 X)

  → Recording (open-ended, 최대 60s):
       매 틱:
         · Hybrid PRBS: SP-direct (메인) + u-direct (보조, headroom-bounded)
         · 데이터 (u_actual, y, r=spInject, uInject, sat) 기록
       매 2초 (80 ticks):
         · Sat-aware amp adaptive (sat rate ~10-25% target)
       매 6초 (240 ticks):
         · Welch periodogram + Cross-spectrum + Coherence
         · 3 band 의 |S(f)| + γ²(f) 측정
         · 부족 band (S 큰 곳) → PRBS bit_ticks 조정
       60s 도달 → 종료

  → FRIT 식별:
       · 28 seeds = 27-grid + 현재 PID
       · LM 30 iter 각 seed → cost 최저 채택
       · u-direct 보정: e = (1/C)·(u_actual - u_inject)

  → Apply
```

**학계 근거 요약**:

| 컴포넌트 | 학계 출처 |
|---|---|
| FRIT | Soma/Kaneko 2004 |
| Levenberg-Marquardt | Marquardt 1963 |
| Sensitivity function | Skogestad/Postlethwaite *Multivariable Feedback Control* |
| Closed-loop identifiability | Forssell/Ljung 1999 |
| Additive perturbation (u-direct) | Söderström/Stoica §8.5 |
| Welch periodogram | Welch 1967 |
| Coherence γ²(f) | Bendat/Piersol 2010 *Random Data* |
| Input amplitude constraint | Hjalmarsson 2005 |
| PRBS | Ljung *System Identification* §13 |
| FTD PID 이산화 (backward Euler) | Ai.dll PidStandardForm 디컴파일 검증 |

---

## 2. PID 제어기

ISA PID:
$$u(t) = K_p \left[ e(t) + \frac{1}{T_i} \int e \, d\tau + T_d \frac{de}{dt} \right]$$

### FTD 의 이산화 (디컴파일 검증)

- 적분: `I += e·dt` (backward Euler)
- 미분: `(e - e_prev) / dt` (backward difference)
- Output: `u_pre_clip = Kp·(e + I/Ti + Td·de/dt)`
- Anti-windup: `I` clamp 후 `u = clip(u_pre_clip, ±1)`

### z-domain 표현

$$C(z) = K_p \cdot \frac{a_0 + a_1 z^{-1} + a_2 z^{-2}}{1 - z^{-1}}$$

$$a_0 = 1 + \frac{dt}{T_i} + \frac{T_d}{dt}, \quad a_1 = -\left(1 + \frac{2 T_d}{dt}\right), \quad a_2 = \frac{T_d}{dt}$$

### FTD 슬라이더 범위

| 파라미터 | 범위 | 기본값 |
|---|---|---|
| `K_p` | 0 ~ 1 | 0.05 |
| `T_i` | 0 ~ 250 | 250 (=off) |
| `T_d` | 0 ~ 100 | 0.3 |

---

## 3. 폐루프 식별의 본질적 한계

### Closed-loop identifiability theorem (Forssell-Ljung 1999)

```
T(jω) = CG/(1+CG)        ← closed-loop transfer
S(jω) = 1/(1+CG) = 1-T   ← sensitivity
```

- |S(jω)| 큼 = controller 가 그 freq 에서 reject 못 함 = **정보 풍부**
- |S(jω)| ≈ 0 = controller 가 완벽 reject = **정보 없음**
- |S(jω)| → 0 모든 freq = 완벽 PID = *수학적으로 식별 불가능*

→ 강한 PID 일수록 식별 어려움. 어떤 식별 방법도 우회 불가능.

---

## 4. 가진 (Excitation) — Hybrid PRBS

### 4.1 PRBS (Pseudo-Random Binary Sequence)

10-bit LFSR (`x^10 + x^7 + 1`, period 1023). 매 `PrbsBitTicks` 마다 새 ±1 비트.

`bit_ticks ↔ 자극 band`:
- `bit=4` (0.1s) → high band (0-5Hz emphasis)
- `bit=16` (0.4s) → mid band (0-1.25Hz)
- `bit=64` (1.6s) → low band (0-0.3Hz)

매 6초 spectral monitor 가 `|S|` 가장 큰 band 로 자동 조정.

### 4.2 HPF (DC drift cancellation)

1차 IIR HPF (fc ≈ 0.01 Hz, α=0.9984): 적분기 plant 의 finite-window PRBS DC bias 제거. fast PRBS dynamics 유지.

### 4.3 Hybrid 주입 — SP-direct + u-direct

**같은 PRBS 신호를 양쪽 inject**:

```
PRBS bit (±1) → HPF → prbsHpf
                       ↓
        ┌──────────────┼──────────────┐
        ↓                              ↓
   SP_inject = SP_amp · prbsHpf    u_inject = u_amp · prbsHpf
                                        ↓ headroom bound
   focus.SetPointAdjust.Us           clamp(u_inject, ±γ·(1-|u_C|))
   = base + SP_inject               (γ=0.5)
                                        ↓
                                   FritExcitationInjector.Set
                                   → patch 가 u_PID 에 더함
```

**근거**:
- SP-direct: FRIT cost 와 자연 호환 (Soma/Kaneko 2004)
- u-direct: Söderström-Stoica §8.5 additive perturbation
- Headroom-bounded: Hjalmarsson 2005 amplitude constraint
- 같은 PRBS 신호: 단순 + 두 경로 phase 일치

### 4.4 Saturation-aware adaptive amplitude

매 80 틱 (~2초):

```
sat rate > 30% → amp × 0.7 (비선형 distortion 회피)
sat rate < 10% → amp × 1.4 (정보량 ↑)
10-30%        → 유지
```

`SP_amp ∈ [0.01, 1.0]`, `u_amp ∈ [0.005, 0.3]`. 목표 sat rate ~10-25% (Hjalmarsson 2005).

---

## 5. 데이터 수집 — Welch + Coherence Spectral Monitor

### 5.1 Welch periodogram (학계 정통)

**Welch 1967, Bendat-Piersol 2010**:

```
[모든 데이터 사용]
SEG_LEN = 256, step = 128 (50% overlap)
K = (N - 256) / 128 + 1   ← 시간 흐를수록 K 증가

[각 segment]
  Hanning window 적용 (spectral leakage 차단)
  FFT (y, r 둘 다)

[Cross-spectrum 누적]
  S_yy[f] += |Y_k(f)|²
  S_rr[f] += |R_k(f)|²
  S_yr[f] += Y_k(f) · conj(R_k(f))   ← complex

[K segments 평균]
  S_yy /= K, S_rr /= K, S_yr /= K
```

**효과**: noise variance ∝ 1/K (60초 수집 시 K=17 → 4-5배 감소).

### 5.2 Transfer function + Coherence

```
T(f) = S_yr(f) / S_rr(f)               ← complex transfer
γ²(f) = |S_yr(f)|² / (S_yy(f)·S_rr(f))  ← coherence ∈ [0, 1]
S(f) = |1 - T(f)|                       ← sensitivity
```

**Coherence γ² 의 의미**:
- ≈ 1: y, r 의 그 freq linear relationship 강함 → |S| 측정 *신뢰 가능*
- ≈ 0: noise dominate → |S| 측정 *무의미*

→ Coherence 가 *numerical artifact 자동 차단*. 신호 약한 bin 은 자연히 가중치 낮음.

### 5.3 Band 평균 (coherence-weighted)

```
|S|_band = Σ |S(f)| · γ²(f) / Σ γ²(f)   ← 신뢰 bin 의 weight 큼
```

3 band:
- low: 0.05-0.5 Hz
- mid: 0.5-2 Hz
- high: 2-5 Hz

### 5.4 Adaptive PRBS bit_ticks

가장 큰 `|S|_band` → 그 band 자극하는 bit_ticks 선택:
- `S_lo` 가장 큼 → bit_ticks = 64 (low 자극)
- `S_mid` 가장 큼 → bit_ticks = 16 (mid 자극)
- `S_hi` 가장 큼 → bit_ticks = 4 (high 자극)

### 5.5 Open-ended termination

```
Hard timeout: T ≥ 60초 → 종료, FRIT 실행
```

자동 well-tuned 감지는 *비활성화* (`tracking_ratio` 가 *진짜 well-tuned* 와 *plant 조용* 구분 못 함). UI bars (max|S|, γ²) 로 *사용자가 직접 판단*.

### 5.6 Saturation 처리

`measured u = post-clip clamp`:
```csharp
double u = clamp(c.LastControlVariable, ±1);
```

- u_actual = plant 실제 입력 (linear regression unbiased)
- FRIT cost: saturation 샘플 weight 0 (anti-windup → PID nonlinear 영역 제외)

---

## 6. FRIT 식별 (with u-direct 보정)

### 6.1 Cost function (u-direct compensated)

$$\tilde{r}_k(\theta) = y_k + C(\theta)^{-1} \cdot u_{PID,k}$$

$$u_{PID,k} = u_{actual,k} - u_{inject,k} \quad \text{(patch 가 더한 부분 제거)}$$

$$\hat{y}_k(\theta) = M(z) \cdot \tilde{r}_{k - \delta}(\theta)$$

$$J(\theta) = \sum_k [y_k - \hat{y}_k(\theta)]^2$$

**왜 보정?**: u-direct 가진의 `u_inject` 가 cost 식에 들어가면 *trivial solution 위험*. 명시적으로 빼서 *PID 만의 fictitious reference* 계산. Söderström-Stoica §8.5.

### 6.2 1/C(z) 안정성 체크

```
disc = a₁² - 4 a₀ a₂
실근:  stable ⇔ |z₁| < 1 ∧ |z₂| < 1
복소근: stable ⇔ a₂/a₀ < 1
```

불안정 시 soft barrier (residual=1e3) → LM 후퇴.

### 6.3 IIR 역필터

$$e[k] = \frac{u_{PID}[k] - u_{PID}[k-1] - K_p a_1 e[k-1] - K_p a_2 e[k-2]}{K_p a_0}$$

### 6.4 참조 모델 M(z) — Tustin

$$M(s) = \frac{e^{-s \tau_M}}{(1 + s \cdot 0.2 T_s)^{n_M}}, \quad n_M = 2$$

`T_s` 는 *사용자 슬라이더* (SettlingTimeTs).

### 6.5 27-Grid Multistart

```
Kp ∈ {0.01, 0.1, 1.0}
Ti ∈ {1, 10, 100}
Td ∈ {0, 0.1, 1.0}
→ 3×3×3 = 27 grid + 현재 PID 1개 = 28 seeds
```

각 seed 에서 LM 30 iter → cost 최저 결과 채택.

학계: LM 의 local minimum 함정 회피 — multistart 가 표준 (Bjorck 1996).

### 6.6 결과 후처리

- FTD slider 단위 반올림 (Kp 0.001, Ti/Td 0.1)
- Hard cap: Kp ≤ 1.0, Ti ≤ 250, Td ≤ 10

### 6.7 ComputeNow ↔ AutoTuneCompute 통일

`Compute (FRIT)` 버튼과 `Auto Tune` 둘 다 *동일 28-seed multistart + u-direct 보정* 사용. 결과 일관성 보장.

---

## 7. 사전 진단 — 2-phase (총 6초)

### Phase 0 (0~3s, 가진 OFF)

|u| max/min, saturation count, sign changes 누적. y baseline 계산.

판정:
- `satRate > 40% AND crossRate > 0.5/s AND uSwing > 1.6` → Limit cycle fail
- `satRate > 40%` → 지속 포화 fail
- 그 외 → Phase 1 진입

### Phase 1 (3~6s, 작은 perturbation amp=0.05)

`tracking_ratio = y_std / amp` 측정. *결과 메시지에만 표시* (자동 종료 X).

이유: `tracking_ratio` 가 *완벽 추종* 과 *plant 조용* 구분 못 함. baseline 비교 + SNR 검사 추가 전까지 비활성.

---

## 8. 포화 처리

§5.6 의 saturation 처리:
- `measured u = clamp(LastControlVariable, ±1)` → post-clip → actual plant input
- linear-in-params 회귀에서 unbiased
- FRIT cost: saturation 샘플 weight 0 (anti-windup → nonlinear)

100% saturated 면 `Var(u) = 0` → b 식별 불가능. 코드의 데이터 부족 경고가 자동 catch.

---

## 9. FTD 특이사항 (단순화)

### 환경
- `dt = 0.025s` (40Hz)
- Nyquist ≈ 20Hz
- PRBS bit duration: 4-64 틱

### u-direct injection
`VariableControllerOutputPatch` (Harmony postfix on `NewMeasurement`) 가 PID 출력에 perturbation 더하고 `LastControlVariable` sync.

### 축별 분리 기능 — 제거됨
이전 버전의 *다른 축 SP 고정* + *피치 고도 유지* 기능 제거. 이유:
- FTD UI 가 *single PID window* — 다중 축 metadata 유지 어려움
- Cross-coupling 포함 데이터가 *실제 비행 환경 plant* 식별 → 더 robust PID
- 코드 복잡도 ~400 라인 감소

→ AI 가 다른 축 자유 제어, coupling 영향은 FRIT cost 의 small noise term 으로 흡수.

### Validate
단일 축 (focus) 만 측정. 10초 수집 후 yStd 표시.

---

## 10. 알려진 한계

1. **완벽 PID 식별 불가능** — closed-loop identifiability limit. UI 의 |S| + γ² 로 사용자가 직접 판단.

2. **FRIT non-convex** — LM 의 local minimum. 28-seed multistart 완화하지만 완전 회피 X.

3. **참조 모델 M 차수** — `n_M = 2` 고정. 3차 이상 plant 면 model error.

4. **u-direct headroom γ trade-off** — γ=0.5 가 안전 vs 정보. 작은 γ 안전하지만 plant 자극 약함.

5. **Ts 사용자 결정** — 자동 sweep 없음.

6. **D 항 noise sensitivity** — Td 가 결과마다 변동. 사용자가 cap 조정.

7. **Cross-coupling 영향** — 다른 축 자유 제어 → 약간 noise 증가. *실제 비행 환경* 매치 효과.

8. **`tracking_ratio` 의 ambiguity** — well-tuned 자동 감지 비활성 (baseline + SNR fix 후 부활 가능).

---

## 11. UI 의 사용자 metric

수집 중 표시:

- **|S| Low/Mid/High**: 각 band sensitivity (coherence-weighted)
- **Coherence Low/Mid/High**: 그 band 측정 신뢰도 (γ² ∈ [0,1])
- **Saturation rate**: target 10-25%
- **SP amp / u amp**: adaptive 진폭
- **Collection time**: 0-60s
- **Well-tuned count**: max(|S|) < 0.1 연속 횟수 (정보용)

사용자가 *실시간 데이터 quality* 판단 가능.

---

## 12. 참고문헌

- **FRIT**: Soma, S., Kaneko, O., & Fujii, T. (2004). *A new method of controller parameter tuning based on input-output data — FRIT*. IFAC.
- **Closed-loop identification**: Forssell, U., & Ljung, L. (1999). *Closed-loop identification revisited*. Automatica 35(7).
- **Sensitivity / Loop-shaping**: Skogestad, S., & Postlethwaite, I. (2005). *Multivariable Feedback Control*. Wiley.
- **Additive perturbation**: Söderström, T., & Stoica, P. (1989). *System Identification*. Prentice Hall. §8.5.
- **Welch periodogram**: Welch, P.D. (1967). *The use of fast Fourier transform for the estimation of power spectra*. IEEE Transactions on Audio and Electroacoustics.
- **Coherence / Cross-spectrum**: Bendat, J.S., & Piersol, A.G. (2010). *Random Data: Analysis and Measurement Procedures*. Wiley. Ch. 9-10.
- **Input design**: Hjalmarsson, H. (2005). *From experiment design to closed-loop control*. Automatica 41(3).
- **PRBS / System ID**: Ljung, L. (1999). *System Identification: Theory for the User*. Prentice Hall.
- **Levenberg-Marquardt**: Marquardt, D.W. (1963). *An algorithm for least-squares estimation of nonlinear parameters*. SIAM Journal.
- **Multistart in LM**: Björck, Å. (1996). *Numerical Methods for Least Squares Problems*. SIAM.
- **Tustin bilinear**: Oppenheim, A.V., & Schafer, R.W. *Discrete-Time Signal Processing*. Pearson.
- **MathNet LM 구현**: `MathNet.Numerics.Optimization.LevenbergMarquardtMinimizer`
