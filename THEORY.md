# PID 자동 튜너 — 이론 및 구현

---

## 1. 개요

이 모드는 **FRIT (Fictitious Reference Iterative Tuning)** 기반 PID 자동 튜닝 파이프라인을 제공합니다. 구현은 **시간 영역** + **가중 LS** + **Levenberg-Marquardt**.

**식별 체인:**
```
[Auto Tune]
  → 사전 진단 3초 (가진 OFF, |u| 통계만 관찰)
       · Limit cycle 또는 지속 포화 감지 → 즉시 fail + 권장 Kp 표시
       · 정상이면 다음 단계
  → 가진 신호 (멀티사인 + 저주파 square) 를 SetPoint 에 주입
  → 현재 PID C₀ 로 폐루프 데이터 (u, y) + 포화 플래그 연속 수집 (블록 분리 없음)
  → EffectiveValidCount (= transient-tail 밖 깨끗 샘플) ≥ MinSamples 까지 대기
       · 적응형 진폭이 자동 조정 → 결국 비포화 영역으로 수렴
       · 시간 상한 없음 (사용자가 멈추기 전까지 계속 수집)
  → τ_p 추정 (1순위 FOPDT fit on (u,y) prelude, 2순위 prelude 63.2%, 3순위 자기상관 1/e)
  → Ts sweep — k ∈ {1.0, 1.25, 1.5, 1.75, 2.0, 2.25, 2.5, 3.0, 3.5, 4.0} 로
                max(k·τ_p, k·τ_M) 후보 10개 생성, FRIT 10번 돌려서
                Td/Ti < 0.3 + Td/dt < 10 충족하는 가장 공격적인 후보 채택
  → FRIT 1회 (시간 영역 IIR + 가중 LS + LM)
       · 포화 인덱스만 w = ε 로 down-weight (IIR 회복 transient 는 IRLS Huber 가 자동 처리)
       · 1/C(z) 안정성 미충족 시 soft barrier
  → Apply
```

기존에 있던 PEM (PBSID + Gauss-Newton 시간도메인 식별), BLA (Welch-IV 주파수도메인 식별), VRFT (선형 회귀), 그리고 FRIT 의 초기 주파수 영역 구현은 모두 제거됐고, 시간 영역 FRIT 한 가지로 통일했습니다.

---

## 2. PID 제어기

PID 제어기는 **오차** (목표와 현재의 차이)를 보고 보정 출력을 냅니다.

$$e(t) = \mathrm{setpoint} - \mathrm{current\ value}$$

$$u(t) = K_p \left[ e(t) + \frac{1}{T_i} \int_0^t e(\tau)\,d\tau + T_d \frac{de(t)}{dt} \right]$$

**ISA 형식의 핵심 성질:** `K_p`가 세 항 모두에 곱해짐. `K_p`를 줄이면 P, I, D가 동시에 약해짐.

### FTD 값 범위

| 파라미터 | 범위 | 기본값 |
|-----------|------|---------|
| `K_p` | 0 ~ 1 | 0.05 |
| `T_i` | 0 ~ 250 | 250 (=off) |
| `T_d` | 0 ~ 100 | 0.3 |

### 이산 PID — backward Euler

코드에서 사용하는 이산화 (적분/미분 모두 backward Euler, `s ↔ (1 - z⁻¹)/dt`):

$$C(z) = K_p \cdot \frac{a_0 + a_1 z^{-1} + a_2 z^{-2}}{1 - z^{-1}}$$

$$a_0 = 1 + \frac{dt}{T_i} + \frac{T_d}{dt}, \quad a_1 = -\left(1 + \frac{2 T_d}{dt}\right), \quad a_2 = \frac{T_d}{dt}$$

**중요 성질:** `C(z)` 는 분모에 적분기 `(1 - z⁻¹)` 한 개와 분자에 quadratic 한 개. **역필터 `1/C(z)` 는 분자가 `(1 - z⁻¹)`, 분모가 quadratic** — 안정성은 `a₀ z² + a₁ z + a₂ = 0` 의 zero 가 단위원 내부인지에 달림.

---

## 3. FRIT 이론

### 3.1 목표

폐루프가 **참조 모델** `M(s)` 처럼 동작하길 원합니다:

$$\frac{Y(s)}{R(s)} = \frac{C(s)P(s)}{1 + C(s)P(s)} \approx M(s)$$

`P(s)` 는 플랜트 (기체 물리), `C(s)` 는 PID. 플랜트 모델은 모릅니다.

### 3.2 가상 레퍼런스 (Fictitious Reference)

데이터 `(u, y)` 가 초기 컨트롤러 `C₀` 의 폐루프에서 수집되었다고 가정. 새로운 컨트롤러 `C(θ)` 에 대해, "**이 (u, y) 가 C(θ) 의 폐루프 응답이 되려면 레퍼런스는 무엇이어야 했을까?**"

PID 식 `u = C(θ) · (r - y)` 에서 `r` 에 대해 풀면:

$$\tilde{r}(\theta) = y + C(\theta)^{-1} u$$

이것이 **가상 레퍼런스** `r̃(θ)`. 데이터로부터 직접 계산 가능 (플랜트 모델 필요 없음).

**시간 영역 구현 — IIR 역필터.** `e[k] = C(θ)⁻¹ u[k]` 는 다음 재귀로:

$$e[k] = \frac{u[k] - u[k-1] - K_p a_1 e[k-1] - K_p a_2 e[k-2]}{K_p a_0}$$

그러면 `r̃[k] = y[k] + e[k]`.

### 3.3 가중 비용 함수 + IRLS (Huber)

가상 레퍼런스를 참조 모델에 통과시킨 응답이 실제 출력에 가까워야 함:

$$\hat{y}(\theta) = M(z) \cdot \tilde{r}(\theta)$$

$$J(\theta) = \sum_{k=1}^{N} w[k] \cdot \left[ y[k] - \hat{y}(\theta)[k] \right]^2$$

**가중치 `w[k] = w_sat[k] · w_huber[k]`** (두 단계 곱):

- **`w_sat[k]`** — 구조적 (saturation 처리):
  - 포화 또는 IIR transient tail 내: `1e-3`
  - 그 외: `1.0`

- **`w_huber[k]`** — robust (이상치 down-weight, IRLS 외부 루프로 갱신):
  - `|r[k]| ≤ δ` → `1.0` (정상 잔차)
  - `|r[k]| > δ` → `δ / |r[k]|` (이상치, 영향 감소)
  - `δ = 1.5 · 1.4826 · MAD(잔차)` (MAD-based robust scale)

**IRLS 루프 (3 iter)**:
```
초기: w_huber ≡ 1
Repeat 3 회:
  obsY ← √w · y
  θ* ← LM(obsY, model(·) returns √w · ŷ)
  잔차 r[k] = y[k] - ŷ(θ*)[k]
  MAD 로 δ 추정 → Huber 가중치 갱신
```

**왜 robust 가중치 추가하나**: FRIT 비용은 L2 (잔차 제곱합) 이라 큰 잔차에 민감. Plant 가 진짜 LTI 가 아닌 경우 일부 샘플 잔차가 크게 튀어서 L2 해를 오염시킴. Huber 는 큰 잔차의 영향을 선형으로만 키워서 robust.

`θ = (Kp, Ti, Td)` 3개 파라미터 → **Levenberg-Marquardt** (MathNet `LevenbergMarquardtMinimizer`, finite-difference Jacobian).

### 3.4 가중 LS — sqrt-스케일링 트릭

MathNet LM 은 unweighted `||observedY - model(θ)||²` 을 최소화. 가중 LS 를 만들려면:

$$\sum_k w_k \cdot (y_k - \hat{y}_k)^2 = \sum_k (\sqrt{w_k}\, y_k - \sqrt{w_k}\, \hat{y}_k)^2$$

`observedY' = √w · y`, `model 출력' = √w · ŷ` 로 LM 에 던지면 가중 LS 와 등가. 구현 한 줄로 끝.

### 3.5 시간 영역 vs 주파수 영역

이전 (지금은 제거된) 주파수 영역 구현은:
- FFT → `R̃(jω) = Y + U/C(jω)` → IFFT → residual

문제: 포화 샘플의 비선형 클리핑이 FFT 후 **모든 주파수에 broadband leakage** 를 만들어 `U(jω)`, `Y(jω)` 가 오염 → 시간 영역으로 IFFT 하면 *비포화* 인덱스의 `ŷ` 도 오염됨. 잔차 가중치만으로는 부분적 보정밖에 안 됨.

**시간 영역 구현은 spectral contamination 자체가 없음** — `1/C(z)` 역필터는 causal IIR, `M(z)` 도 causal IIR, 가중치는 잔차에 직접 작용. 또한:
- FFT 제거 → LM 반복당 비용 ↓
- circular convolution wrap-around 없음
- 순수 지연을 정수 틱 shift 로 정확히 처리

### 3.6 VRFT 와의 차이

VRFT 는 선형 회귀로 풉니다:

$$J_\text{VRFT}(\theta) = \sum_k \left[ u[k] - C(\theta) \cdot (M^{-1} y - y)[k] \right]^2$$

선형 LS 로 단번에 풀리지만, 노이즈가 `r_v = M⁻¹y` 와 `y` 양쪽에 들어가 **상관 노이즈로 회귀 계수가 편향**됨 (Instrumental Variable 필요).

FRIT 는 출력 매칭 비용 (비선형) 이라 회귀 구조가 없고 상관 노이즈 편향이 없음 (Soma/Kaneko 2004). 측정 노이즈에 더 강건.

### 3.7 왜 올바른 답이 나오나

`y = P · u` (플랜트가 입력을 출력으로 매핑). `θ = θ*` 에서 폐루프가 `M` 과 정확히 일치한다면:

$$y = M \cdot \tilde{r}(\theta^*) \implies J(\theta^*) = 0$$

`N → ∞` 에서 `argmin J(θ) → θ*` (Campi-Savaresi 2006).

### 3.8 파라미터 불확실성 (CRLB)

추정된 `(Kp, Ti, Td)` 의 표준오차 (standard error) 를 같이 보고. **얼마나 신뢰할 수 있는지** 정량 표시.

**유도 (Gauss-Newton approximation of CRLB)**:

비용 함수의 Hessian 근사 `H ≈ Jᵀ W J`, 여기서 `J = ∂(√w·ŷ)/∂θ` (`N × 3` 자코비안).

파라미터 공분산:
$$\mathrm{cov}(\hat{\theta}) \approx \sigma^2 \cdot (J^T J)^{-1}, \quad \sigma^2 \approx \frac{\sum_k w[k]\,r[k]^2}{N_\text{eff} - 3}$$

`N_eff` = effective valid 샘플 수 (`effSat=false` 이고 `√w ≥ 0.5` 인 인덱스).

표준오차 = 공분산 대각선의 sqrt:
$$\mathrm{SE}(K_p) = \sqrt{\mathrm{cov}_{00}}, \quad \mathrm{SE}(T_i) = \sqrt{\mathrm{cov}_{11}}, \quad \mathrm{SE}(T_d) = \sqrt{\mathrm{cov}_{22}}$$

**구현**:
- 자코비안: central finite-difference 로 직접 계산 (`θ ± ε`, 3 파라미터 × 2 = 6 model evaluations)
- `Jᵀ J`: 3×3 행렬, manual sum
- 역행렬: cofactor 전개 (분석적, singular 면 null 반환)
- `σ²`: 가중 SSR / (N_eff - 3)

**해석**:
- `Kp = 0.053 ± 0.001 (2%)` → 매우 신뢰
- `Ti = 5.2 ± 8.1 (155%)` → 부정확, 데이터 부족이거나 식별 안 됨 → 수동 조정 필요
- `Td = 0.04 ± 0.02 (50%)` → 어느 정도 신뢰

UI 에 `Kp = 0.0523 ± 0.0012 (2%)` 형태로 자동 표시.

---

## 4. 참조 모델 M

### 4.1 형태

$$M(s) = \frac{e^{-s \tau_M}}{(1 + s \cdot 0.2 T_s)^{n_M}}$$

- `T_s` : 목표 정착시간 (Ts sweep 10개 후보 `k·max(τ_p, τ_M)` for k ∈ {1.0, 1.25, 1.5, 1.75, 2.0, 2.25, 2.5, 3.0, 3.5, 4.0}, 클램프 [3·dt, 1.0])
- `n_M` : 모델 차수 (FTD 제어 대상 대부분 2차 → `n_M = 2` 고정)
- `τ_M` : 지연 (FTD 순수 지연 ≈ 1틱 → `τ_M = dt` 고정)

### 4.2 이산화 — Tustin (bilinear)

`s = (2/dt)·(1-z⁻¹)/(1+z⁻¹)` 대입. 1차 LP 부분 `1/(1 + s · aM)` (단, `aM = 0.2 Ts`) 은:

$$H_1(z) = \frac{1 + z^{-1}}{\beta_0 + \beta_1 z^{-1}}, \quad \beta_0 = 1 + \frac{2 a_M}{dt}, \quad \beta_1 = 1 - \frac{2 a_M}{dt}$$

**재귀식:**

$$y[k] = \frac{x[k] + x[k-1] - \beta_1 y[k-1]}{\beta_0}$$

`n_M` 차는 이 1차 LP 를 **`n_M` 번 캐스케이드** (n=2 면 두 번 적용). 순수 지연은 `delayN = round(τ_M / dt)` 만큼 인덱스 shift.

### 4.3 0.2 는 어디서?

`n_M = 2` 인 시스템의 4% 정착시간 (4·시정수) 이 `T_s` 가 되려면 시정수 = `T_s / 4 = 0.25 T_s`. 약간 보수적으로 `0.2 T_s` 사용 → 5% 정착시간 기준.

### 4.4 Ts 자동 결정 — Ts sweep + Td 안전 체크

**1단계: τ_p 추정 (또는 캐시 사용)**

반복 AutoTune 시 closed-loop bias drift (현재 PID 가 느릴수록 측정 τ_p 가 커져서 다음 PID 가 더 느려지는 양성 피드백) 방지를 위해 **τ_p 캐싱** 적용. 첫 AutoTune 에서 추정한 값을 `_cachedTauP` 에 저장하고 이후엔 그 값을 재사용. Reset 버튼이 캐시를 클리어해서 강제 재추정.

신선 추정 시 우선순위:

1. **FOPDT fit** (`EstimateTauFromFopdtFit`) — **plant 고유 τ_p, PID 무관 (unbiased)**:
   prelude 의 (u, y) 둘 다 사용해서 ARX(1,1) OLS 로 1차 plant 모델 fit.
   ```
   y_d[k] = a · y_d[k-1] + b · u_d[k-1-delayN]
   τ_p = -dt / ln(a)
   ```
   plant 방정식 `τ_p·dy/dt + y = K·u(t-θ)` 가 PID 무관하게 성립한다는 사실 활용.
   포화 샘플 제외, 응답이 노이즈 수준이면 NaN 반환 → 폴백.

2. **step prelude 63.2%** (`EstimateTauFromStepPrelude`) — closed-loop biased 폴백:
   FOPDT fit 실패 시 y 의 63.2% rise time 으로 대략 추정. 현재 PID 영향 받음.

3. **자기상관 1/e** (`EstimateDominantTau`) — 최후 폴백:
   y(t) 의 자기상관 (PSD → IFFT, Wiener-Khinchin) 에서 1/e 떨어지는 시각.
   Sine/Chirp 가진 또는 위 두 방법 모두 실패 시.

**왜 FOPDT 가 closed-loop 데이터에서도 unbiased 인가** — plant 미분방정식은 PID 와 무관하게 항상 성립. (u, y) 쌍에서 plant 파라미터를 추정하는 건 OLS 의 표준 절차. PID 가 만들어내는 u 는 노이즈와 약하게 상관될 수 있지만 (closed-loop ID bias), prelude step 의 deterministic 변화가 압도해서 편향이 작음.

**캐싱의 이유** — τ_p 는 원리적으로 plant 고유 정보이지만, 우리 추정기는 closed-loop y 데이터에서 얻으니까 현재 PID 의 영향이 섞임. 반복 호출하면:
1. 현재 PID 가 sluggish → closed-loop 응답 느림 → 추정 τ_p 큼
2. 큰 τ_p → Ts 크게 → 더 sluggish 한 PID
3. positive feedback → τ_p 가 상한 1.0 에 박힘 → PID 가 항상 보수적 폴백

캐싱하면 첫 1회 추정값으로 고정되어 이 drift 가 끊김. 단점: vehicle/axis 가 바뀌면 stale 한 캐시 → 사용자가 Reset 으로 재추정 트리거해야 함.

**2단계: Ts sweep** (SIMC 일반화 + τ_M 고려)

```
factors = {1.0, 1.25, 1.5, 1.75, 2.0, 2.25, 2.5, 3.0, 3.5, 4.0}  ← 공격 → 보수, 10개
best = null;  safeCount = 0;  fallback = null
for k in factors:
    ts_k = clamp(max(k·τ_p, k·τ_M), 3·dt, 1.0)
    r = ComputeFritPid(ts = ts_k)
    if r 정상:
        fallback = r                ← 항상 갱신 (마지막 성공 = 가장 보수적)
        if Td/Ti < 0.3 + Td/dt < 10:
            safeCount++
            if best == null:
                best = r            ← 처음 만난 안전 후보 = 가장 공격적
return best ?? fallback  (best 없으면 fallback + ⚠ 경고)
```

**왜 break 안 하고 끝까지 도는가** — `safeCount` 를 사용자에게 보여줘서 결정의 robustness 를 알리기 위함. `safe 8/10` 이면 매우 안정적, `safe 1/10` 이면 fragile 한 채택. 비용은 FRIT 10회 ≈ 0.5초.

**왜 max(k·τ_p, k·τ_M) 인가** — SIMC 의 표준 하한: closed-loop 은 plant delay 보다 빠를 수 없음. FTD 일반 케이스에선 τ_M = dt 라 `k·τ_p` 가 우세하지만, 사용자가 τ_M 슬라이더 올리면 그쪽이 하한이 됨.

**안전 체크 기준** (D 진동 방지):
- `Td/Ti < 0.3` — 산업 표준 PID 비율. 넘으면 D 항이 너무 우세
- `Td/dt < 10` — 이산 미분의 per-sample 노이즈 게인. 넘으면 측정 잡음 폭증

**왜 sweep 인가** — 단일 SIMC factor (예: 2·τ_p balanced) 로는 D 진동 회피와 빠른 응답을 동시 달성 못함. plant 별로 안전한 τ_c 범위가 달라서, 사용자가 1클릭으로 "안전한 가장 빠른 응답" 받게 하려면 자동 sweep 필요.

**왜 sweep 비용이 안 부담인가** — FRIT 1회 ≈ 30~80ms, 10회 ≈ 0.3~0.8s. 사용자가 체감 못함.

**한계**: 단일 τ_p 추정이라 multi-mode plant 에선 부정확할 수 있음. FTD 게임 환경은 대부분 효과적 1st-order 근사가 OK.

---

## 5. 계산 파이프라인 (FRIT 시간 영역)

### 단계 0: Ts sweep (자세히는 §4.4)
1. **τ_p 추정** — 캐시 ↘ FOPDT fit ↘ step prelude 63.2% ↘ 자기상관 1/e (자세히는 §4.4)
2. **10개 후보 sweep** — `k ∈ {1, 1.25, 1.5, 1.75, 2, 2.25, 2.5, 3, 3.5, 4}` 로 `Ts_k = clamp(max(k·τ_p, k·τ_M), 3·dt, 1.0)`
3. **각 Ts_k 마다 ComputeFritPid 호출** → 안전 체크 (`Td/Ti < 0.3 ∧ Td/dt < 10`) 통과한 가장 공격적인 결과 채택. 끝까지 돌면서 `safeCount` 카운트
4. **모두 실패 시** 마지막 성공 후보 (가장 보수적) + 경고 메시지 폴백

### 단계 1: 디트렌드
DC + 선형 추세 제거 (전체 데이터 기준). 짧은 포화 샘플 영향은 미미.

### 단계 2: 참조 모델 계수 사전 계산
`β₀, β₁, delayN, nM` 은 `θ` 와 무관 → LM 외부에서 한 번만 계산.

### 단계 3: per-sample 가중치
```
w[k] = ε (≈ 1e-3)  if sat[k]
       1.0         otherwise
sqrtW[k] = √w[k]
```

### 단계 4: LM 모델 함수 (`θ → √w · ŷ`)

매 LM 평가마다:

```
1. PID 계수 (a₀, a₁, a₂) ← (Kp, Ti, Td)
2. 1/C(z) 안정성 체크 (단위원 내부 zero?)
     불안정: soft barrier (큰 residual 반환)
3. e[k] = (u[k] - u[k-1] - Kp·a₁·e[k-1] - Kp·a₂·e[k-2]) / (Kp·a₀)   (역필터)
4. r̃[k] = y[k] + e[k]                                                (가상 ref)
5. r̃_d[k] = r̃[k - delayN]                                           (순수 지연)
6. ŷ ← H₁ 캐스케이드 nM 번 적용 to r̃_d                              (참조 모델)
7. NaN/Inf 검사 → 발견 시 soft barrier
8. return √w · ŷ
```

### 단계 5: IRLS 외부 루프 (3 iter) + LM 최적화
- 초기값: 현재 PID `(kP, kI, kD)` (sanity check 후 사용)
- 각 IRLS iter: MathNet `LevenbergMarquardtMinimizer` (`maxIter = 30`, FD Jacobian)
- IRLS 사이: 잔차 MAD → δ → Huber 가중치 갱신 → 다음 LM

### 단계 6: CRLB (표준오차)
- LM 수렴 후 `θ*` 에서 자코비안 FD 계산 (6 model evals)
- `cov ≈ σ² · (JᵀJ)⁻¹` → `SE(Kp), SE(Ti), SE(Td)`

### 단계 7: RMSE 계산
포화 인덱스 제외, robust-down-weighted 도 제외, unweighted 잔차:

$$\mathrm{RMSE} = \sqrt{\frac{1}{N_\mathrm{valid}} \sum_{k:\,\neg\mathrm{sat}[k]} (y_k - \hat{y}_k)^2}$$

### 단계 8: 경계 클램프
- `Kp ∈ [0, 1]`
- `Ti ∈ [0.1, 250]`
- `Td ∈ [0, 10]`
- NaN/Inf 처리

### 단계 9: 게임 PID 에 적용
`RoundToStep` 으로 game UI 호환 단위 (Kp 0.001, Ti/Td 0.1) 로 양자화 후 `_focus.Pid.kP/kI/kD` 에 기록.

---

## 6. 1/C(z) 안정성 — soft barrier

PID 의 분자 quadratic `a₀ z² + a₁ z + a₂` 의 zero 가 **단위원 외부** 면 역필터 `1/C(z)` 가 발산. LM 이 탐색 도중 이런 `θ` 를 던질 수 있음.

**판별:**
```
disc = a₁² - 4 a₀ a₂
disc ≥ 0: 실근
  z₁,₂ = (-a₁ ± √disc) / (2 a₀)
  stable ⇔ |z₁| < 1 AND |z₂| < 1
disc < 0:  복소 conjugate 한 쌍
  |z|² = a₂ / a₀
  stable ⇔ a₂/a₀ < 1
```

**처리:** 모든 LM 모델 평가 시작에서 체크. 불안정이면 `result = 1e6 · √w` 반환 → 큰 cost 로 LM 이 후퇴.

실전 PID 값 (`dt=0.02`, `Kp~0.05`, `Ti~5`, `Td~0.05`) 에서 zero 가 0.71, 0.99 정도 → 모두 내부 안전. Td 가 매우 크거나 Ti 가 매우 작을 때만 경계 근접.

---

## 7. 가진 (Excitation)

### 7.1 왜 필요한가
PID 가 안정적으로 잘 작동하면 `u/y` 가 거의 일정 → 플랜트 정보 없음. 외부 가진으로 SP 를 흔들어 정보를 만듭니다.

### 7.2 멀티사인 + 저주파 square wave (기본)

진폭 예산을 둘로 분할: 멀티사인 `(1-r)·A` + square wave `r·A`, 기본 `r = 0.5`.

**멀티사인 (P/D 모드 식별):**
12 성분, 로그 간격 (`fBase` ~ `fMax`), 슈뢰더 위상.

$$x_\text{ms}(t) = (1-r) A \sum_{i=0}^{11} \frac{1}{\sqrt{12}} \sin(2\pi f_i t + \phi_i), \quad \phi_i = -\frac{\pi i (i+1)}{12}$$

슈뢰더 위상 → 피크 팩터 ≈ √2 (균일 사인). 같은 RMS 진폭으로 가장 큰 SNR 확보.

**저주파 square wave (I 모드 식별 — DC 정보 보강):**

$$x_\text{sq}(t) = r A \cdot \mathrm{sign}\!\left[\sin(2\pi f_\text{sq} t)\right]$$

기본 `f_sq = 0.1Hz` (주기 10초). 각 half-period 동안 SP 가 일정 → **적분기에 sustained 오차** → Ti 식별 강화. 평균 0 이라 자세 bias 없음. Square 전환 시점마다 transient → FRIT 가 좋아하는 데이터.

전체:

$$x(t) = x_\text{ms}(t) + x_\text{sq}(t)$$

설정: `SquareAmpRatio` (`r`, 기본 0.5), `SquareFreqHz` (`f_sq`, 기본 0.1Hz). `r = 0` 으로 두면 순수 멀티사인.

**왜 DC 정보가 따로 필요한가**: 멀티사인은 DC 성분이 0 (모든 사인의 평균은 0). PID 적분 모드 `(K_p/T_i) ∫e dt` 는 저주파 / DC 오차에 반응 → 저주파 가진 없으면 `T_i` 가 데이터에 거의 안 묻혀나옴. Square wave 의 half-period 가 길수록 (`f_sq` 작을수록) 적분 모드가 더 강하게 식별됨, 단 녹화 시간 내 최소 2회 전환 필요.

### 7.3 Step Prelude (초기 DC kick)
첫 0.5초는 가진 대신 `x = A` 일정 (full magnitude DC pulse). 이후 multisine + square 정상 사이클. FRIT 는 정상성 가정이 없으므로 **이 transient 도 식별 데이터에 그대로 포함** (구 PEM 시절 step skip 제거됨).

### 7.4 적응형 진폭 (saturation 기반 binary)

식별 이론적으로 amp 가 클수록 SNR ↑ → 식별 정확도 ↑. 진짜 cap 은 saturation 뿐 (y info 는 부산물). 그래서 단순 binary 규칙:

```
윈도우 (60샘플 ≈ 1.2초) 통계:
  satRate = (포화 카운트) / 윈도우
  uPeak   = max |u| in 윈도우

규칙 (3초 쿨다운):
  satRate > 2%  OR  uPeak > 0.85   → amp × 0.667   (포화 위험 → 후퇴)
  그 외                              → amp × 1.5     (margin 있음 → 밀어붙임)
```

자연스럽게 amp 가 saturation 경계 주위 (uPeak ≈ 0.85) 로 수렴 → 항상 max 가능 amp 에서 식별.

**비대칭 (↑ ×1.5, ↓ ×0.667)** 이유: 사이클당 `1.5 × 0.667 = 1.0` 균형 — 정확히 boundary 주위에서 oscillation. ↓ 가 살짝 더 강하면 안전 마진 ↑.

**y 기준 없음**: 이전 코드는 `yStd/amp < 0.15` 또는 `uStd < 0.1` 로 "정보 충분" 판단했으나, FRIT 입장에서 이건 PEM 잔재. FRIT 비용은 출력 매칭 `||y - ŷ||²` 이라 y info 가 본질적이고 amp 늘리면 y 도 자동으로 늘어남. 그래서 별도 y 기준 불필요.

---

## 8. 사전 진단 (Pre-Diagnose) — Auto Tune 직후 3초

### 8.1 왜 필요한가

현재 PID 가 "이미 limit cycle" 이거나 "지속 포화" 상태면 어떤 가진을 줘도 식별 못 함:
- u 가 자체 진동 (±포화 사이) → 모든 샘플이 포화 → effSat 가 거의 전체 → 유효 샘플 0
- 60 초 타임아웃까지 발버둥 → "왜 안 됐지?" fail 메시지

진단 단계는 **가진을 켜기 전 3 초간 관찰만** 해서 위 상황을 사전 차단 + 사용자에게 정확한 권장값 제시.

### 8.2 측정 (3초간 가진 OFF)

```
매 틱:
  |u| 의 max/min 갱신
  if |u| ≥ SaturationThreshold: satCount++
  if sign(u) ≠ sign(prevU): signChanges++  (부호 변환)

3초 후:
  satRate    = satCount / sampleCount
  uSwing     = uMax − uMin
  uPeak      = max(|uMax|, |uMin|)
  crossRate  = signChanges / 3초
```

### 8.3 판정 규칙

| 패턴 | 조건 | 동작 |
|---|---|---|
| **Limit cycle** | `satRate > 40%` AND `crossRate > 0.5/s` AND `uSwing > 1.6` | fail + `Kp × 0.4` 권장 |
| **지속 포화** | `satRate > 40%` AND 진동 적음 | fail + `Kp × 0.5` 권장 |
| **살짝 포화** | `satRate 15~40%` | 경고 후 진행 |
| **정상** | `satRate < 15%` | 정상 진행 |

### 8.4 사용자 경험

**Before**: `[Auto Tune]` → 60초 후 `"failed: only 38 valid samples"` (왜 실패했는지 불명)

**After**: `[Auto Tune]` → 3초 후 `"Limit cycle (u=-0.94~+0.97, 1.7회/s, sat=78%). Kp 0.150→0.060 으로 줄이고 재시도."` (정확한 처방)

### 8.5 자동 PID 변경은 안 함

진단은 **읽기 전용** — 현재 PID 를 자동으로 수정하지 않음. 권장값만 표시. 이유:
- 사용자 통제권 유지 (PID 가 모르는 새 바뀌는 거 거부감)
- 진단 권장값이 항상 최선은 아님 (Ti/Td 도 손봐야 할 수도)
- 안전 (자동 수정으로 더 나빠질 위험 차단)

---

## 9. 포화 처리 — 가중치 + IRLS 안전망

`|u| ≥ 0.98` 이면 비선형 영역 → FRIT 의 선형 LTI 가정 위반. 모든 샘플을 연속 저장하되, 식별 단계에서 처리:

### 9.1 실시간 가진 회피 (`ApplyExcitation`)
가진 적용 시 `|u|` 가 0.98 근처면 가진 진폭을 자동 축소 (스케일 0.1 ~ 1.0). 포화를 사전 차단.

### 9.2 양방향 적응형 진폭
위 §7.4 — `uPeak > 0.85` 이면 진폭 ÷1.5. 포화 발생 시 점진적으로 감쇠.

### 9.3 모든 샘플 저장 + 포화 플래그
포화 여부와 무관하게 `(U, Y, Saturated)` 에 모두 저장. 블록 분리, 샘플 드롭, satRatio fail-fast 모두 없음.

### 9.4 포화 인덱스만 가중치 ε
`Saturated[k] = true` 인 샘플만 `w_sat = 1e-3` 로 down-weight.

```csharp
sqrtW[k] = √(sat[k] ? 1e-3 : 1.0)
```

**`TransientTailSamples = 0` (기본)** — IIR 회복 transient 보호 없음.

### 9.5 IRLS Huber 가 회복 transient 처리
`1/C(z)` 역필터는 IIR 라 포화 직후 ~2초 회복 transient 가 있음. 명시적으로 tail 을 안 막는 대신 IRLS 가 자동 처리:

```
LM 1차 → 잔차 계산
MAD 로 robust scale 추정
δ = 1.5 × 1.4826 × MAD
잔차 |r| > δ 인 샘플 → w_huber = δ/|r| (down-weight)
LM 2차 → 반복
```

**효과**:
- 짧은 spike (1~2틱) → 회복 transient 가 작음 → 잔차 분포에 안 묻혀 검출 안 됨 → 그대로 사용. 영향 미미.
- 좀 긴 포화 (5~20틱) → 회복 transient 잔차가 outlier 처럼 보임 → IRLS 가 자동 down-weight.
- 매우 긴 포화 (>50틱) → 진단 단계가 차단했어야 함. 진단 통과 후 발생하면 IRLS 보호 약하지만 매우 드문 케이스.

### 9.6 EffectiveValidCount 기반 종료
수집은 `EffectiveValidCount ≥ MinSamples` 까지 계속. 적응형 진폭이 자동으로 진폭을 줄여서 결국 비포화 영역으로 수렴 → 깨끗한 샘플이 쌓임. 시간 상한은 없음 — 사용자가 직접 멈추지 않는 한 비포화 샘플이 모일 때까지 계속 녹화.

---

**왜 IRLS 에 의존하나** (Tail = 0 결정)

이전에는 `TransientTailSamples = 100` (≈2초) 로 포화 직후 100 샘플을 통째로 down-weight 했음. 단점:

- 짧은 spike 5개만 있어도 데이터 50% 손실 → CRLB SE % ↑
- 정통적이지만 과보호

새 접근 (`Tail = 0` + IRLS):
- §8 진단이 큰 포화 사전 차단 → recording 중 포화는 대부분 짧은 spike
- 짧은 spike 의 회복 transient 는 영향 자체가 작음 → tail 보호 불필요
- IRLS 가 outlier 잔차 자동 처리 → 좀 더 긴 포화도 부분 보호
- 데이터 손실 50% → 0.5% → CRLB 정밀도 ↑

→ 진단 + 가중치 + IRLS 3중 보호로 동등 robustness, 데이터 활용 극대화.

---

## 10. FTD 특이사항

### 축별 SP/PV 구조
- `SetPointAdjust` : 외부에서 SP 에 offset 주입 (가진용)
- `FakeSetPoint` + `FakeSetPointInUse` : AI 의 SP 를 외부 값으로 강제 (축 고정용)

### 축 분리 (Axis Fixture)
튜닝 중 다른 축은 `FakeSetPoint = 현재 PV` 로 고정 → 기존 PID 가 자세/고도 유지. 튜닝이 끝나면 복원.

### 피치 고도 유지
비행기형 기체는 피치로 고도 제어 → 피치 SP 고정하면 고도 드리프트. Hover 축의 PV 로 고도 오차를 측정, 피치 SP 에 실시간 offset 주입.

### 튜닝 순서
권장: Roll → Pitch → Yaw (속도 → 자세 → 항법) → Hover/Forward (있다면 마지막).

각 PID UI 를 한 번씩 열어 축 타입 (Yaw/Roll/Pitch/Hover/Forward/Strafe) 을 지정해야 자동발견 + 고도 보정이 작동.

### 환경
- `dt = Time.fixedDeltaTime ≈ 0.02s` (50Hz)
- Nyquist = 25Hz
- 멀티사인 대역 ~0.05Hz (저주파) ~ `min(fs/4, ChirpEndHz)`

---

## 11. 알려진 한계

1. **초기 컨트롤러 안정성 가정.** FRIT 는 폐루프 데이터를 전제. 초기 PID 가 발산 직전이면 데이터 자체가 비선형 transient 라 결과 신뢰도 ↓.

2. **Ti 식별 한계.** 저주파 square wave 와 step prelude 로 DC/저주파 가진을 주입하지만, 녹화가 짧으면 (square half-period 보다 짧으면) 적분 모드가 충분히 활성화 안 됨. `SquareFreqHz` 를 낮추거나 녹화 시간을 늘려야 함. Ti 가 250 (=off) 에 가까이 수렴하면 신호 부족 의심.

3. **비선형성.** 큰 진폭에서는 LTI 근사가 깨짐. 적응형 진폭이 자동 축소하지만, 본질적으로 비선형이 강한 경우 (예: 고기동 영역) 는 한계.

4. **LM local minima.** 비선형 최적화의 본질적 한계. 초기 시드 (현재 PID) 가 멀리 떨어져 있으면 다른 minimum 에 수렴 가능. Ts 스캔이 부분적인 안전망.

5. **1/C(z) 안정성 제약.** 매우 큰 Td 나 매우 작은 Ti 에서 역필터 zero 가 단위원 밖으로 → soft barrier 가 작동해서 LM 이 그 영역을 못 들어감. 가끔 합리적이지만 "이상한" PID 값 (예: 매우 빠른 inner loop) 이 그 경계 근처면 도달 못 함.

6. **계산 비용.** Ts 10단계 × LM 30 iter × (역필터 + M 캐스케이드) ≈ N · 30 · 10 · (~4 cost evals) flop. 주파수 영역보다 빠르지만 (FFT 없음) 여전히 블록 길이에 선형 비례.

---

## 12. 참고문헌

- **FRIT 원본**: Soma, S., Kaneko, O., & Fujii, T. (2004). *A new method of controller parameter tuning based on input-output data — Fictitious Reference Iterative Tuning (FRIT)*. IFAC Workshop on Adaptation and Learning in Control and Signal Processing.
- **FRIT 강건성 (vs VRFT)**: Kaneko, O. (2013). *Data-driven controller tuning: FRIT approach*. IFAC Proceedings Volumes 46(11).
- **VRFT (참고용)**: Campi, M.C., Lecchini, A., & Savaresi, S.M. (2002). *Virtual reference feedback tuning: a direct method for the design of feedback controllers*. Automatica 38(8).
- **Levenberg-Marquardt**: Moré, J.J. (1978). *The Levenberg-Marquardt algorithm: implementation and theory*. Numerical Analysis.
- **Tustin bilinear transform**: Oppenheim, A.V. & Schafer, R.W. *Discrete-Time Signal Processing*, ch. 7.
- **MathNet LM 구현**: `MathNet.Numerics.Optimization.LevenbergMarquardtMinimizer`
