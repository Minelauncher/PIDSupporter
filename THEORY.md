# PID 자동 튜너 — 이론 및 구현

---

## 1. 개요

이 모드는 **IV-ARX/OE (Refined Instrumental Variables) plant identification + SIMC PID design** 으로 자동 튜닝을 수행합니다. 모든 단계가 학계 표준 + 명확한 수학적 근거에서 출발했고, 임의 휴리스틱 상수 사용 안 함.

**파이프라인**:

```
[Auto Tune]
  → 사전 진단 3초 (가진 OFF, |u| 만 관찰)
       · Limit cycle 또는 지속 포화 → 즉시 fail + Kp 권장값
  → 데이터 수집
       · PRBS 가진 (HPF 통과) 을 SetPoint 에 주입
       · u-target adaptive amplitude — controller 강도 무관 일정한 plant excitation
       · u 는 ±1 clamp 후 기록 (post-clip → actual plant input)
       · SE-게이트 종료: SE(τ_1)/τ_1 < 0.2 또는 N ≥ MinSamples×4
  → IV-ARX/OE (RIV) plant 식별 (closed-loop unbiased)
       · y[k] = a₁·y[k-1] + a₂·y[k-2] + b·u[k-1-δ]
       · 극점 z₁, z₂ → 연속 시정수 τ_1, τ_2, DC 게인 K
       · 적분기 감지 (|1-a₁-a₂| < 2·SE 통계적 기준)
  → SIMC PID 계산
       · 1차 plant: PI
       · 2차 plant: PID (Td = τ_2)
       · 적분기 plant: PI for integrator (Ti = 4(τ_c+θ))
  → Apply
```

**수학적 기반과 한계 (명시)**:

- **PRBS 가진** — Ljung "System Identification" §13. 10-bit LFSR (x¹⁰ + x⁷ + 1, period 2¹⁰-1=1023), bit duration 4 ticks. ±A 이진, broadband 스펙트럼. 임의값 0개.
- **PRBS HPF** — α=0.9984 (fc≈0.01Hz), 1차 IIR. 적분기 plant 의 finite-window DC bias 만 제거, fast PRBS dynamics 영향 무시 가능.
- **u-target adaptive amplitude** — Hjalmarsson 2005 "input design with power constraint". 매 ~2초 마다 u_std 측정 → r amplitude 조정 → controller 무관 일정한 Fisher info.
- **IV-ARX/OE (RIV)** — Young 1980 Refined IV, Söderström-Stoica §8.6 (closed-loop IV with reference signal as instrument). 점근적으로 OE-PEM 과 동일 efficiency.
- **SIMC formula** — Skogestad 2003. 1차/2차/적분기 plant 별 closed-form.
- **적분기 판정** — `|1-a₁-a₂| < 2·SE_denom` 2σ 통계적 기준 (Fisher info 기반 SE).
- **Plant pole reject** — `|z| > 2.0` 만 truly unstable. 1.0 ~ 2.0 은 적분기로 cap (RIV finite-N noise 가 unit circle 밖으로 미는 케이스 흡수).
- **UI 강제 bound** — FTD 슬라이더 한계 (Ti ≤ 250, Td ≤ 10).

---

## 2. PID 제어기

PID 제어기는 **오차** (목표와 현재의 차이) 를 보고 보정 출력을 냅니다.

$$e(t) = \mathrm{setpoint} - \mathrm{current\ value}$$

$$u(t) = K_p \left[ e(t) + \frac{1}{T_i} \int_0^t e(\tau)\,d\tau + T_d \frac{de(t)}{dt} \right]$$

**ISA 형식**: `K_p` 가 세 항 모두에 곱해짐. `K_p` 를 줄이면 P, I, D 가 동시에 약해짐.

### FTD 값 범위

| 파라미터 | 범위 | 기본값 |
|-----------|------|---------|
| `K_p` | 0 ~ 1 | 0.05 |
| `T_i` | 0 ~ 250 | 250 (=off) |
| `T_d` | 0 ~ 100 | 0.3 |

FTD 의 PID 출력은 `[-1, 1]` 로 clip 되어 액추에이터로 보내짐.

---

## 3. 폐루프 식별의 세 가지 문제

폐루프 (PID 작동 중) 에서 plant 식별의 본질적 어려움:

### 3.1 Bias (consistency)

폐루프에서 `u = C(z)·(r-y)` 라 `u` 와 noise `v` 가 *상관*. 단순 ARX OLS 회귀:

```
y[k] = a₁·y[k-1] + a₂·y[k-2] + b·u[k-1-δ] + v[k]
```

는 `u` 의 noise correlation 때문에 *systematic bias*. 큰 N 으로도 옳은 답에 안 수렴.

**해결**: IV-ARX/OE (RIV) — instrument `Z` 가 noise 와 무상관이면 unbiased.

### 3.2 Variance / SNR

같은 estimation method 라도 *데이터의 정보량* 에 따라 SE 가 다름. 폐루프에서:

```
u = T(z)·r,  T = C/(1+CG)  (complementary sensitivity)
```

- weak C → |T| 작음 → 같은 r 에서 u 작음 → plant 자극 부족
- 식별 variance ∝ σ²_noise / (N · Var(u))

→ controller 강도가 SE 에 직접 영향. weak C 면 SE 폭발 가능.

**해결**: u-target adaptive amplitude — `r` 을 자동 scale 해서 u_std 를 사용자 설정값에 맞춤 → Fisher info 가 controller 무관.

### 3.3 Saturation

u 가 ±1 rail 에 hit → clipping 발생 → nonlinearity.

**핵심 관찰**: measured u (clipped value) = actual plant input. 회귀식이 linear-in-params:

```
y[k] = a₁·y[k-1] + a₂·y[k-2] + b·u_actual[k-1-δ] + v[k]
```

`u_actual` 자체가 nonlinear (saturated) 신호여도, regressor 로 measured value 를 쓰면 LS/IV 가 unbiased. saturation 이 *데이터에 묻혀있어서* model misspecification 아님.

**한계**: Fisher info ∝ Var(u). 100% saturated 면 `Var(u)=0` → b 식별 불가능. 비포화 비율이 어느 정도 있어야 식별 가능. SE 가 이걸 자동 reflect.

**해결**: saturation 샘플도 회귀에 그대로 포함. u 측정 시 명시적 clamp 로 post-clip 보장.

---

## 4. 가진 (Excitation) — PRBS + HPF + adaptive amplitude

### 4.1 PRBS (Pseudo-Random Binary Sequence)

10-bit LFSR (Linear Feedback Shift Register):

```
state ← 1  (any non-zero seed)
매 PRBS_BIT_TICKS 틱 (=4틱, 0.1s @ dt=0.025):
    new_bit = (state >> 9) XOR (state >> 6)  ← 다항식 x¹⁰ + x⁷ + 1
    state = ((state << 1) | new_bit) & 0x3FF
    PRBS_value = (new_bit == 1) ? +1 : -1
```

**성질** (Ljung §13):
- Maximum length: 2¹⁰ - 1 = 1023 비트 (102.3초 @ dt=0.025·4틱)
- Broadband spectrum: 0 ~ fs/(2·PRBS_BIT_TICKS) = ~5Hz (FTD plant 일반 범위 cover)
- 평균 1/1023 ≈ 0 (DC 거의 zero)
- White noise 와 spectral 성질 유사하지만 deterministic + bounded

**왜 PRBS?**
- ±A bounded → 과한 진폭 risk 없음 (vs Gaussian noise)
- broadband → 모든 plant mode 활성화 (vs single sine)
- deterministic → 재현 가능 (vs random noise)

### 4.2 PRBS HPF (drift cancellation)

PRBS 의 짧은 window 평균이 정확히 0 은 아님 (period 가 1023 비트). 적분기 plant 에서 finite-window DC bias 가 누적되면 비행기가 천천히 drift.

해결: 가진 신호에 매우 저주파 HPF 적용:

```
y[n] = α·(y[n-1] + x[n] - x[n-1]),   α = exp(-2π·fc·dt)
```

`fc = 0.01Hz`, `dt = 0.025` → α = 0.9984.

**성질**:
- HPF 시정수 1/(2π·fc) ≈ 16초. PRBS bit (0.1초) 내 decay = exp(-0.1/16) ≈ 0.994 → 거의 무손실
- 장기 DC bias 만 제거. fast PRBS dynamics 유지
- IV 조건 영향 없음: HPF 통과한 `r` 도 *외부 신호* → noise 와 무상관 ✓

### 4.3 u-target adaptive amplitude

**문제**: PRBS 의 ±A 진폭이 고정이면 controller 강도에 따라 plant 자극이 변동.
- weak C: |T(z)| 작음 → u 거의 안 움직임 → 정보 부족
- strong C: u 크게 움직임 → saturation 위험

**해결**: 사용자가 설정하는 `ExciteAmp` 의 의미를 *u (plant input) 의 target std* 로 재해석. 매 K 틱마다 측정된 u_std 로 가진 amplitude 자동 조정.

알고리즘 (매 80 틱 ≈ 2초):

```
u_recent ← last 80 samples of measured U
u_std ← std(u_recent)
target ← _s.ExciteAmp
ratio ← clamp(target / u_std, 0.5, 2.0)   ← 단계당 최대 2배 변동
AmpDyn ← clamp(AmpDyn × ratio, 0.001, 2.0)
```

**수학적 근거** (Hjalmarsson 2005):
- Fisher info: `I(θ) ∝ Var(u) / σ²_noise`
- Var(u) → constant target² → `I(θ)` 가 controller 무관
- 즉 SE 가 controller 강도와 무관 → "어떤 PID 에서도 같은 정확도로 수렴"

**왜 단계당 한계 (0.5 ~ 2.0)?** — 한 step 에 amp 가 폭발적으로 변하면 그 동안 데이터 stationarity 깨짐. 점진적 변동.

### 4.4 SP-direct vs u-direct 가진 — 왜 SP

이론적으로 u 에 직접 가진 (u-direct = additive perturbation) 더하면 controller 무관 자극이 가능. 하지만 FTD 의 plant (각도 오차 → 추력) 는 *적분기* 라 u 에 직접 inject 하면 비행기가 drift 누적되어 추락. 그래서 SP-direct (r 에 inject) 사용 + adaptive amplitude 로 보완.

---

## 5. IV-ARX/OE (Refined Instrumental Variables)

### 5.1 모델 가정

**OE (Output Error) noise model**:

```
y[k] = G(z)·u[k] + v[k]
```

`G(z) = b·z⁻¹⁻ᵈ / (1 - a₁·z⁻¹ - a₂·z⁻²)` 의 ARX(2,1) 구조. `v[k]` 는 noise (additive output side).

ARX 회귀 형태로 표현:

```
y[k] = a₁·y[k-1] + a₂·y[k-2] + b·u[k-1-δ] + ε[k]
```

`ε[k] = v[k] - a₁·v[k-1] - a₂·v[k-2]` (colored, OE 노이즈를 ARX 회귀로 옮긴 결과).

### 5.2 Stage 1: ARX OLS 초기 추정

3×3 normal equation (Cramer 풀이):

```
M_ij = Σ ϕ_i[k]·ϕ_j[k],   ϕ[k] = [y[k-1], y[k-2], u[k-1-δ]]
t_i = Σ y[k]·ϕ_i[k]

[a₁, a₂, b] = M⁻¹ · t
```

폐루프 데이터에서는 `ε[k]` 와 `ϕ[k]` 가 상관 → OLS bias. **하지만 RIV 의 초기값으로만 사용**.

### 5.3 Stage 2+: RIV iteration

**IV (Instrumental Variables) 의 원리**: regressor `ϕ` 대신 *noise 와 무상관한* instrument `Z` 사용:

```
(Z^T X) θ = Z^T y
```

`E[Z·v] = 0` 이면 N→∞ 에서 unbiased.

**우리 instrument**:
```
Z[k] = [y_sim[k-1], y_sim[k-2], r[k-1-δ]]
```

- `y_sim`: 현재 추정 (a₁, a₂, b) 으로 forward simulation. noise-free → noise 와 무상관 ✓
- `r`: PRBS reference (HPF 통과). 외부 신호 → noise 와 무상관 ✓

**RIV iteration** (Young 1980):
```
초기: (a₁, a₂, b) ← Stage 1 OLS
Repeat (최대 3회):
    안정성 가드: 현재 추정의 |z_max| > 1 이면 break (시뮬레이션 발산 방지)
    y_sim[k] = a₁·y_sim[k-1] + a₂·y_sim[k-2] + b·u[k-1-δ]
    Z matrix 구성 + (Z^T X) θ_new = Z^T y 풀이
    if change < TOL: break
    θ ← θ_new
```

**왜 RIV 가 폐루프에서 consistent?**

Söderström-Stoica §8.6: instrument `r` (외부 reference) 가 closed-loop ID 의 표준 IV. `r` 이 어떤 controller 에서도 noise 와 무관 → bias 제거. 점근적으로 PEM (Prediction Error Method) 과 동일 efficiency.

### 5.4 Plant pole reject / cap

특성다항식 z² - a₁·z - a₂ = 0 의 root:

```
disc = a₁² + 4a₂
disc ≥ 0:  z₁,₂ = (a₁ ± √disc) / 2     (실근)
disc < 0:  |z|² = -a₂                   (복소 conjugate)
```

`zSlow = max(|z₁|, |z₂|)` = dominant pole magnitude.

**판정**:
- `|z_slow| > 2.0` → truly unstable, fail
- `1.0 < |z_slow| ≤ 2.0` → 적분기 plant 로 cap (`HasIntegrator = true`). RIV finite-N noise 가 stable 극을 unit circle 살짝 밖으로 미는 케이스 흡수.
- `|z_slow| ≤ 1.0` → 정상 stable plant.

**적분기 판정 (별도)**: `|1 - a₁ - a₂| < 2·SE_denom` 2σ 통계적 기준. DC gain 의 분모가 noise 보다 작으면 → 적분기.

### 5.5 시정수 변환

이산 극점 → 연속 시정수:

```
τ = -dt / ln|z|     (|z| < 1 가정)
```

`z` 가 1 근처면 τ → ∞ (적분기). 수치 보호로 `min(zSlow, 1 - 1e-6)` cap.

### 5.6 RIV stability guard

매 RIV iteration 시작 시 현재 추정의 |z_max| > 1 이면 break:

```csharp
double zMaxAbs = (disc ≥ 0) ? max(|(a₁+√disc)/2|, |(a₁-√disc)/2|) : √(max(0,-a₂))
if (zMaxAbs > 1.0) break;   // simulation 발산 → IV 오염 방지
```

ARX OLS 초기값이 borderline unstable 이면 simulate 한 y_sim 이 발산 → IV matrix 오염 → 다음 추정이 더 unstable. 이 발산 chain 차단.

### 5.7 SE (Standard Error)

τ_1 의 표준오차 — chain rule:

```
SE(a₁) = √(σ² · M_11 / |det(M)|)         (Fisher info diagonal)
∂z/∂a₁ = 0.5 + a₁/(2·√disc)              (real case)
∂τ/∂z = dt / (z · ln²z)
SE(τ_1) ≈ |∂τ/∂z · ∂z/∂a₁| · SE(a₁)
```

이 SE 가 SE-게이트 (§6.3) 와 결과 신뢰도 표시에 사용.

---

## 6. 수집 종료 — SE-게이트

### 6.1 왜 단순 N ≥ MinSamples 가 부족한가

같은 N 이라도 데이터 품질 (SNR, saturation 비율) 에 따라 SE 다름. weak C + 큰 amp 이면 N 충분해도 SE 클 수 있음. 반대로 strong C + 깨끗한 데이터면 작은 N 으로도 정확.

### 6.2 SE 기반 종료 조건

```
매 240 틱 (~6초):
    if N < MinSamples: skip
    if N ≥ MinSamples × 4: 무조건 종료 (hard cap)
    else:
        QuickIdSeRatio 호출 (intermediate IV-ARX)
        if SE(τ_1)/τ_1 < 0.2: 종료
        else: 수집 계속
```

**왜 0.2?** — 상대 SE 20% = 추정값의 95% CI 가 ±40%. 실용적으로 SIMC 가 합리적 PID 뽑는 한계.

**Hard cap (MinSamples × 4)** — 무한 수집 방지. 어떤 데이터에서도 결국 종료 보장.

### 6.3 QuickIdSeRatio

전체 IV-ARX/OE 파이프라인을 호출해서 `m.TauSE / m.Tau1` 반환. RIV 포함이라 정확하지만 비용 ~10ms (N ~ 4096 기준). 6초마다 1번이라 무시 가능.

---

## 7. SIMC PID 설계 (Skogestad 2003)

### 7.1 1차 plant: PI

`G(s) = K · e⁻ˢθ / (1 + τ_p·s)` 식별 → PI:

```
K_p = τ_p / (K · (τ_c + θ))
T_i = min(τ_p, 4·(τ_c + θ))
T_d = 0
```

`τ_c` (closed-loop 시정수) = 사용자 선택:
- Aggressive: τ_c = τ_p
- Balanced: τ_c = 2·τ_p  (default)
- Conservative: τ_c = 4·τ_p

### 7.2 2차 plant: PID

`G(s) = K · e⁻ˢθ / ((1+τ_1·s)(1+τ_2·s))`, τ_1 > τ_2 식별 → PID:

```
K_p = τ_1 / (K · (τ_c + θ))
T_i = min(τ_1, 4·(τ_c + θ))
T_d = τ_2
```

D 항이 두 번째 시정수 cancel. SIMC 의 자연스러운 2차 처리.

### 7.3 적분기 plant: PI for integrator

`G(s) = K_i · e⁻ˢθ / s` 식별 → PI:

```
K_p = 1 / (K_i · (τ_c + θ))
T_i = 4 · (τ_c + θ)
T_d = 0
```

Skogestad 2003 §6 "integrating processes".

### 7.4 적분기 K_i 계산

ARX 식별에서 적분기 plant 의 DC gain 식 `b/(1-a₁-a₂)` 가 0/0 분모. 대신:

```
K_i = b / dt          (rate gain — unit input 당 출력 변화율)
```

이산 적분기 `1/(1-z⁻¹)` 의 ZOH 등가가 `dt/s` 라서 `b·dt/s` 형태 → `K_i = b/dt`.

---

## 8. 사전 진단 (Pre-Diagnose) — Auto Tune 직후 3초

### 8.1 왜 필요한가

현재 PID 가 "이미 limit cycle" 이거나 "지속 포화" 상태면 어떤 가진을 줘도 식별 못 함 (saturation 데이터 처리해도 비포화 비율 0% 면 한계). 데이터 수집 전에 차단.

### 8.2 측정 (3초간 가진 OFF)

```
매 틱:
    |u| 의 max/min 갱신
    if |u| ≥ SaturationThreshold: satCount++
    if sign(u) ≠ sign(prevU): signChanges++

3초 후:
    satRate = satCount / sampleCount
    uSwing  = uMax − uMin
    crossRate = signChanges / 3초
```

### 8.3 판정 규칙

| 패턴 | 조건 | 동작 |
|---|---|---|
| **Limit cycle** | `satRate > 40%` AND `crossRate > 0.5/s` AND `uSwing > 1.6` | fail + `Kp × 0.4` 권장 |
| **지속 포화** | `satRate > 40%` AND 진동 적음 | fail + `Kp × 0.5` 권장 |
| **살짝 포화** | `satRate 15~40%` | 경고 후 진행 |
| **정상** | `satRate < 15%` | 정상 진행 |

### 8.4 자동 PID 변경은 안 함

진단은 *읽기 전용*. 사용자 통제권 + 안전.

---

## 9. 포화 처리

§3.3 에서 본 것처럼 *saturation 자체는 bias 안 만듦*. 우리 구현:

### 9.1 수집 시 명시적 clamp

```csharp
double u = Math.Max(-1.0, Math.Min(1.0, c.LastControlVariable));
_sess.U.Add(u);
```

FTD 의 `LastControlVariable` 이 post-clip 인지 pre-clip 인지 의존성 제거. 항상 actual plant input.

### 9.2 회귀에서 포화 샘플 포함

IV-ARX/OE 의 모든 합산 루프 (ARX OLS, RIV IV matrix, residual) 에서 saturation 제외 안 함. 모든 N 사용.

**효과**:
- 데이터 손실 0% (이전 구현은 saturation 샘플 통째로 drop → weak C 케이스에서 거의 모든 샘플 제거됨)
- bias 없음 (linear-in-params 회귀)
- 비포화 비율이 작으면 SE 가 자동으로 reflect → SE-게이트가 거부

### 9.3 한계

100% saturated → `Var(u)=0` → `det(M)=0` → 식별 불가능. 코드의 `|det| < 1e-12` 가드가 자동 catch ("regressors collinear" diagnosis).

비포화 비율이 매우 낮으면 SE 가 크게 나옴 → SE-게이트가 거부하거나 hard cap 까지 수집.

---

## 10. FTD 특이사항

### 축별 SP/PV 구조
- `SetPointAdjust` : 외부에서 SP 에 offset 주입 (가진용)
- `FakeSetPoint` + `FakeSetPointInUse` : AI 의 SP 를 외부 값으로 강제 (축 고정용)

### 축 분리 (Axis Fixture)
튜닝 중 다른 축은 `FakeSetPoint = 현재 PV` 로 고정 → 기존 PID 가 자세/고도 유지. 튜닝 끝나면 복원.

### 피치 고도 유지
비행기형 기체는 피치로 고도 제어 → 피치 SP 고정하면 고도 드리프트. Hover 축의 PV 로 고도 오차 측정, 피치 SP 에 실시간 offset 주입.

### 튜닝 순서
권장: Roll → Pitch → Yaw → Hover/Forward.

각 PID UI 를 한 번씩 열어 축 타입 (Yaw/Roll/Pitch/Hover/Forward/Strafe) 지정해야 자동 발견 + 고도 보정 작동.

### 환경
- `dt = Time.fixedDeltaTime ≈ 0.025s` (~40Hz)
- Nyquist ≈ 20Hz
- PRBS broadband ~0 ~ 5Hz

---

## 11. 알려진 한계

1. **초기 컨트롤러 안정성 가정.** 폐루프 데이터 전제. 초기 PID 가 발산이면 데이터 자체가 비선형 transient.

2. **2차 plant 가정.** ARX(2,1) 모델. 3차 이상 plant 는 dominant 2 모드로 근사. higher-order dynamics 는 model error 로 들어감.

3. **OE noise 가정.** Noise 가 multiplicative / colored input 측이면 IV 가 부분적으로만 효과. 대부분 케이스에서 충분.

4. **τ_2 식별의 한계.** 빠른 모드 (τ_2) 는 고주파 정보. PRBS bit duration (4틱=0.1초) 가 너무 길면 τ_2 정보 부족. dt 짧으면 자동 해결.

5. **계산 비용.** SE-게이트로 매 240 틱 IV-ARX 호출. N=4096 기준 ~10ms. 게임 FixedUpdate 무시 가능.

6. **Adaptive amplitude 의 settling.** 처음 80틱 (~2초) 은 amp 가 사용자 슬라이더 값 그대로. 그 뒤 점진적 조정. weak C 케이스에서 처음 amp 너무 작으면 데이터 정보 부족 → 결국 amp 증가하지만 settling time 손해.

---

## 12. 참고문헌

- **PRBS / System ID**: Ljung, L. (1999). *System Identification: Theory for the User* (2nd ed.). Prentice Hall. §13.
- **RIV (Refined Instrumental Variables)**: Young, P.C. (1980). *Parameter estimation for continuous-time models — A survey*. Automatica 17(1).
- **Closed-loop IV with reference instrument**: Söderström, T. & Stoica, P. (1989). *System Identification*. Prentice Hall. §8.6.
- **Input design with power constraint**: Hjalmarsson, H. (2005). *From experiment design to closed-loop control*. Automatica 41(3).
- **SIMC PID tuning**: Skogestad, S. (2003). *Simple analytic rules for model reduction and PID controller tuning*. Journal of Process Control 13(4).
- **Closed-loop identification overview**: Forssell, U. & Ljung, L. (1999). *Closed-loop identification revisited*. Automatica 35(7).
