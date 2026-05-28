# PID 자동 튜너 — 이론 및 구현

---

## 1. 개요

이 모드는 **Hybrid PRBS 가진 + Information-driven 수집 + FRIT 식별** 으로 PID 자동 튜닝을 수행합니다. 모든 단계가 학계 표준 + 명확한 수학적 근거.

**핵심 파이프라인**:

```
[Auto Tune]
  → Diagnose Phase 0 (3s, 가진 OFF)
       · Limit cycle / 지속 포화 검출 → fail
       · y baseline 측정 (다음 phase 기준)

  → Diagnose Phase 1 (3s, 작은 perturbation amp=0.05)
       · tracking_ratio = y_std/amp 측정
       · < 0.05 → "Already well-tuned" 즉시 종료, FRIT skip

  → Recording (open-ended, 최대 60s):
       매 틱:
         · Hybrid PRBS: SP 와 u 양쪽에 같은 PRBS·HPF 주입
         · u-direct: headroom-bounded (γ·(1-|u_C|))
         · 데이터 (u, y, r, sat) 기록
       매 2초 (80 ticks):
         · Sat-aware amp adaptive (sat rate ~10-25% target)
       매 6초 (240 ticks):
         · y/r FFT → 3 band sensitivity |S(f)|
         · 부족 band (S 큰 곳) → PRBS bit_ticks 조정
         · max(S) < 0.1 for 3 consecutive windows → well-tuned 종료

  → FRIT 식별 (timeout 도달 시):
       · 28 seeds = 27-grid (Kp/Ti/Td log-spaced) + 현재 PID
       · LM 30 iter 각 시드 → cost 최저 채택
       · 사용자 슬라이더 Ts 그대로 (sweep 없음)

  → Apply
```

**학계 근거 요약**:

| 컴포넌트 | 학계 출처 |
|---|---|
| FRIT | Soma/Kaneko 2004 *Fictitious Reference Iterative Tuning* |
| Levenberg-Marquardt | Levenberg 1944, Marquardt 1963 |
| Sensitivity function | Skogestad/Postlethwaite *Multivariable Feedback Control* |
| Closed-loop identifiability | Forssell/Ljung 1999 *Closed-loop identification revisited* |
| Additive perturbation (u-direct) | Söderström/Stoica §8.5 |
| Input amplitude constraint | Hjalmarsson 2005 *From experiment design to closed-loop control* |
| PRBS | Ljung *System Identification* §13 |
| FTD PID 이산화 분석 | backward Euler (Ai.dll PidStandardForm 디컴파일 검증) |

---

## 2. PID 제어기

표준 ISA PID:

$$u(t) = K_p \left[ e(t) + \frac{1}{T_i} \int e \, d\tau + T_d \frac{de}{dt} \right]$$

### FTD 의 이산화 (Ai.dll 디컴파일로 검증)

- 적분: `I += e·dt` (backward Euler)
- 미분: `(e - e_prev) / dt` (backward difference)
- Output: `u_pre_clip = Kp·(e + I/Ti + Td·de/dt)`
- Anti-windup: `I` clamp 후 `u = clip(u_pre_clip, ±1)`

→ 우리 backward Euler 가정과 완벽 일치 ✓

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

**결론**: 강한 PID 일수록 식별 어려움. 이건 fundamental — 어떤 식별 방법도 우회 불가능.

→ 우리 mod 의 *"이미 잘 튜닝됨" 종료 logic* 의 정확한 근거.

---

## 4. 가진 (Excitation) — Hybrid PRBS

### 4.1 PRBS (Pseudo-Random Binary Sequence)

10-bit LFSR (Linear Feedback Shift Register):
- 다항식: `x^10 + x^7 + 1` (maximum length sequence, period 2^10-1 = 1023)
- 매 `PrbsBitTicks` 마다 새 비트 (±1)
- broadband 스펙트럼 (학계 표준, Ljung §13)

`PrbsBitTicks` 는 spectral monitor 가 동적 조정:
- 4 (=0.1초): high-freq emphasis
- 16 (=0.4초): mid-freq
- 64 (=1.6초): low-freq

### 4.2 HPF (DC drift cancellation)

PRBS 의 short-window 평균이 정확히 0 아님 → 적분기 plant 에서 누적 drift.

1차 IIR HPF:
$$y[n] = \alpha \cdot (y[n-1] + x[n] - x[n-1]), \quad \alpha = 0.9984$$

`fc ≈ 0.01 Hz`. PRBS bit (0.1s) 내 decay 무시 가능, 장기 DC bias 만 제거.

### 4.3 SP-direct 가진 (Main, FRIT 호환)

$$\text{SetPointAdjust} = \text{base} + \text{SP}_\text{amp} \cdot \text{prbsHpf}$$

- FRIT cost 식 `r̃ = y + C^{-1}·u` 와 자연 호환
- `r = SP_inject` 가 FRIT 의 reference 신호

### 4.4 u-direct 가진 (보조, Controller 우회)

$$u_\text{actual} = u_\text{PID} + u_\text{inject}$$

Harmony patch (`VariableControllerOutputPatch`) 가 `VariableControllerMaster.NewMeasurement` postfix 에서 `__result` 와 `LastControlVariable` 둘 다 동기화.

**Headroom-bounded amp** (Hjalmarsson 2005):

$$u_\text{inject} = \text{clamp}(u_\text{amp} \cdot \text{prbsHpf}, \pm \gamma (1 - |u_C|))$$

- γ = 0.5
- PID 가 한가할 때 (`u_C` 작음) → headroom 큼 → 강한 자극
- PID 가 바쁠 때 (`u_C` 큼) → headroom 작음 → 안전 자제
- saturation 자동 회피

**왜 hybrid?**:
- SP only: controller 가 r 을 filter → spectrum controller-dependent
- u only: FRIT cost 식과 부정합 (trivial solution 위험)
- Hybrid: SP 가 FRIT main 정보, u-direct 가 controller 우회 plant 자극 보조

### 4.5 Saturation-aware adaptive amplitude

매 80 틱 (~2초) 마다:

```
sat rate > 30% → amp × 0.7 (감소, 비선형 distortion 회피)
sat rate < 10% → amp × 1.4 (적극 증가, 정보량 ↑)
10-30%        → 유지
```

목표 sat rate ~10-25% — *plant 자극 최대 + 비선형 영역 최소* (Hjalmarsson 2005).

SP_amp 와 u_amp 둘 다 같은 sat rate 기반 조정.

---

## 5. 데이터 수집 — Information-driven termination

### 5.1 Spectral monitor (매 6초)

```
FFT_LEN = 256 (~6.4초 window)
SPECTRAL_INTERVAL = 240 ticks (~6초)
```

매 6초 마다:
1. `y_recent`, `r_recent` 의 FFT
2. 3 band 의 `|T(f)| = |Y(f)|/|R(f)|` 평균 계산:
   - low: 0.05 ~ 0.5 Hz
   - mid: 0.5 ~ 2 Hz
   - high: 2 ~ 5 Hz
3. `|S(f)| = |1 - T(f)|` 각 band
4. **부족 band 식별** (가장 |S| 큰 band) → PRBS bit_ticks 조정:
   - S_lo 가장 큼 → bit_ticks = 64
   - S_mid 가장 큼 → bit_ticks = 16
   - S_hi 가장 큼 → bit_ticks = 4
5. `max(S) = max(S_lo, S_mid, S_hi)` 반환

### 5.2 Open-ended termination

```
if max(S) < 0.1:    WellTunedConsecutiveCount++
else:               WellTunedConsecutiveCount = 0

종료 조건 A: WellTunedConsecutiveCount >= 3 (= 18초 동안 consistent)
    → "Well-tuned" 종료, FRIT skip
    → closed-loop identifiability limit 도달

종료 조건 B: T >= 60초 (hard timeout)
    → 정보 충분, FRIT 실행
```

ε = 0.1 의 의미: 모든 band 에서 `|T| ∈ [0.9, 1.1]` = 90%+ tracking. 거의 완벽 PID.

K = 3 의 의미: 18초 동안 consistent → noise 가 아닌 진짜 saturation.

### 5.3 Saturation 처리

`measured u` = post-clip clamp:
```csharp
double u = Math.Max(-1.0, Math.Min(1.0, c.LastControlVariable));
```

→ `u_actual` = plant 실제 입력. Linear-in-params 회귀에서 *bias 없음*.

FRIT cost 에서 saturation 샘플은 weight 0 (cost 합에서 제외) — anti-windup 으로 PID 가 nonlinear 동작 영역.

---

## 6. FRIT 식별

### 6.1 Cost function

$$J(\theta) = \sum_k [y_k - \hat{y}_k(\theta)]^2$$

$$\tilde{r}_k(\theta) = y_k + C(\theta)^{-1} u_k \quad \text{(가상 reference)}$$

$$\hat{y}_k(\theta) = M(z) \cdot \tilde{r}_{k - \delta}(\theta)$$

### 6.2 1/C(z) 안정성 체크

PID 의 분자 quadratic `a₀ z² + a₁ z + a₂` 의 zero 가 단위원 외부면 역필터 발산. LM 매 evaluation 시작에 체크:

```
disc = a₁² - 4 a₀ a₂
실근:    stable ⇔ |z₁| < 1 ∧ |z₂| < 1
복소근:  stable ⇔ a₂/a₀ < 1
```

불안정 시 soft barrier (`residual = 1e3`) → LM 후퇴.

### 6.3 IIR 역필터 `e = (1/C)·u`

$$e[k] = \frac{u[k] - u[k-1] - K_p a_1 e[k-1] - K_p a_2 e[k-2]}{K_p a_0}$$

### 6.4 참조 모델 M(z) — Tustin 이산화

$$M(s) = \frac{e^{-s \tau_M}}{(1 + s \cdot 0.2 T_s)^{n_M}}, \quad n_M = 2$$

1차 LP filter Tustin:
$$y[k] = \frac{x[k] + x[k-1] - \beta_1 y[k-1]}{\beta_0}$$

$$\beta_0 = 1 + \frac{2 \cdot 0.2 T_s}{dt}, \quad \beta_1 = 1 - \frac{2 \cdot 0.2 T_s}{dt}$$

`n_M = 2` 회 캐스케이드 + 지연 `delayN = round(τ_M/dt)` 적용.

`T_s` 는 *사용자 슬라이더* (`SettlingTimeTs`) — 자동 sweep 없음 (단순화).

### 6.5 27-grid Multistart

LM 의 local minimum 함정 회피:

```
Kp ∈ {0.01, 0.1, 1.0}    (3 값, log-spaced)
Ti ∈ {1, 10, 100}         (3 값, log-spaced)
Td ∈ {0, 0.1, 1.0}        (3 값)
→ 3×3×3 = 27 grid + 현재 PID 1개 = 28 seeds
```

각 seed 에서 LM 30 iter (MathNet `LevenbergMarquardtMinimizer`) → cost 최저 결과 채택.

`27 × 30 iter × FD Jacobian ≈ 14초` (50ms × 28 LM 평균).

### 6.6 결과 후처리

- FTD slider 단위 반올림: Kp 0.001, Ti/Td 0.1
- Hard cap: Kp ≤ 1.0, Ti ≤ 250, Td ≤ 10

---

## 7. 사전 진단 — 2-phase (총 6초)

### Phase 0 (0~3s, 가진 OFF)

- |u| max/min, saturation count, sign changes 누적
- y baseline 계산 (Phase 1 기준)

**판정**:
- Limit cycle: `satRate > 40% AND crossRate > 0.5/s AND uSwing > 1.6` → fail
- 지속 포화: `satRate > 40% AND 진동 적음` → fail
- 정상: Phase 1 진입

### Phase 1 (3~6s, 작은 PRBS perturbation amp=0.05)

- `y_std` 측정 (baseline 대비)
- `tracking_ratio = y_std / amp`

**판정**:
- `tracking_ratio < 0.05` → "Already well-tuned" 즉시 종료
- 그 외 → Recording 진입

학계: Phase 1 의 perturbation test 는 *active probing* — 가진 OFF 만으로는 "이미 완벽 PID" 와 "그냥 조용한 상태" 구분 불가.

---

## 8. FTD 특이사항

### 축별 SP/PV 구조
- `SetPointAdjust`: 외부 SP offset 주입 (가진용)
- `FakeSetPoint`: AI 의 SP 를 외부 값으로 강제 (축 고정용)

### 축 분리 (Axis Fixture)
튜닝 중 다른 축은 `FakeSetPoint = 현재 PV` 로 고정.

### 피치 고도 유지
비행기는 피치로 고도 제어 → 피치 SP 고정하면 고도 drift. Hover 축 PV 로 고도 오차 측정, 피치 SP 에 실시간 offset 주입.

### u-direct injection
`VariableControllerOutputPatch` (Harmony postfix on `NewMeasurement`) 가 PID 출력에 perturbation 더하고 `LastControlVariable` sync.

### 환경
- `dt = 0.025s` (40Hz)
- Nyquist ≈ 20Hz
- PRBS bit duration: 4-64 틱 (0.1-1.6초)

---

## 9. 알려진 한계

1. **완벽 PID 식별 불가능** — closed-loop identifiability limit (Forssell-Ljung 1999). 우리 well-tuned termination 으로 처리.

2. **FRIT non-convex** — LM 의 local minimum 위험. 28-seed multistart 로 완화하지만 완전 회피 X.

3. **참조 모델 M 차수 가정** — `n_M = 2` 고정. 3차 이상 plant 면 model error.

4. **u-direct headroom 의 trade-off** — γ=0.5 가 안전 vs 정보. 작은 γ 면 안전하지만 plant 자극 약함.

5. **Ts 사용자 결정** — sweep 제거. 사용자가 적절한 Ts 슬라이더 설정 필요.

6. **D 항 식별 noise sensitivity** — Td 가 결과마다 변동 가능. 사용자가 보고 cap 조정 필요.

---

## 10. 참고문헌

- **FRIT 원본**: Soma, S., Kaneko, O., & Fujii, T. (2004). *A new method of controller parameter tuning based on input-output data — Fictitious Reference Iterative Tuning (FRIT)*. IFAC Workshop on Adaptation and Learning in Control and Signal Processing.

- **Closed-loop identification**: Forssell, U., & Ljung, L. (1999). *Closed-loop identification revisited*. Automatica 35(7), 1215-1241.

- **Sensitivity function**: Skogestad, S., & Postlethwaite, I. (2005). *Multivariable Feedback Control* (2nd ed.). Wiley.

- **Additive perturbation**: Söderström, T., & Stoica, P. (1989). *System Identification*. Prentice Hall. §8.5.

- **Input design**: Hjalmarsson, H. (2005). *From experiment design to closed-loop control*. Automatica 41(3), 393-438.

- **PRBS / System ID**: Ljung, L. (1999). *System Identification: Theory for the User* (2nd ed.). Prentice Hall. §13.

- **Levenberg-Marquardt**: Marquardt, D.W. (1963). *An algorithm for least-squares estimation of nonlinear parameters*. Journal of the Society for Industrial and Applied Mathematics 11(2).

- **Tustin bilinear**: Oppenheim, A.V., & Schafer, R.W. *Discrete-Time Signal Processing* (3rd ed.). Pearson. Ch. 7.

- **MathNet LM 구현**: `MathNet.Numerics.Optimization.LevenbergMarquardtMinimizer`
