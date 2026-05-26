# PID 자동 튜너 — 이론 및 구현

---

## 1. 개요

이 모드는 **FRIT (Fictitious Reference Iterative Tuning)** 기반 PID 자동 튜닝 파이프라인을 제공합니다. 구현은 **시간 영역** + **가중 LS** + **Levenberg-Marquardt**.

**식별 체인:**
```
[Auto Tune]
  → 가진 신호 (멀티사인) 를 SetPoint 에 주입
  → 현재 PID C₀ 로 폐루프 데이터 (u, y) + 포화 플래그 수집
  → 최장 비-드롭 블록 추출 (긴 연속 포화는 데이터 자체 드롭, 블록 분리)
  → Ts 자동 스캔 (10단계)
  → 각 Ts 마다 FRIT (시간 영역 IIR + 가중 LS + LM)
       · 짧은 포화 인덱스는 w = ε 로 down-weight
       · 1/C(z) 안정성 미충족 시 soft barrier
  → 인접 Ts 간 파라미터 안정성 기반 best Ts 선택
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

### 3.3 가중 비용 함수

가상 레퍼런스를 참조 모델에 통과시킨 응답이 실제 출력에 가까워야 함:

$$\hat{y}(\theta) = M(z) \cdot \tilde{r}(\theta)$$

$$J(\theta) = \sum_{k=1}^{N} w[k] \cdot \left[ y[k] - \hat{y}(\theta)[k] \right]^2$$

**가중치 `w[k]`** :
- 비-포화 샘플: `w[k] = 1`
- 짧은 포화 샘플: `w[k] = ε ≈ 10⁻³` (down-weight)
- 긴 연속 포화 샘플: 데이터 자체에서 드롭 + 블록 분리 (앞 단계에서 처리)

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

---

## 4. 참조 모델 M

### 4.1 형태

$$M(s) = \frac{e^{-s \tau_M}}{(1 + s \cdot 0.2 T_s)^{n_M}}$$

- `T_s` : 목표 정착시간 (Ts 자동 스캔 0.1~1.0초)
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

### 4.4 Ts 자동 스캔

작은 Ts 는 공격적인 제어기를 요구 → Kp 가 큼. 큰 Ts 는 보수적. "물리적으로 달성 가능한" Ts 영역에서는 파라미터가 안정.

전략: 0.1 ~ 1.0초를 로그 간격 10단계로 스캔, 각 Ts 에서 LM 한 번 돌림, 인접 Ts 간 `max(|ΔKp/Kp|, |ΔTi/Ti|, |ΔTd/Td|) < 0.3` 인 가장 작은 Ts 선택.

---

## 5. 계산 파이프라인 (FRIT 시간 영역)

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

### 단계 5: LM 최적화
- 초기값: 현재 PID `(kP, kI, kD)` (sanity check 후 사용)
- MathNet `LevenbergMarquardtMinimizer`, `maxIter = 30`
- FD Jacobian (3 파라미터 × 1 = 3 추가 평가 / iter)

### 단계 6: RMSE 계산
포화 인덱스 제외, unweighted 잔차:

$$\mathrm{RMSE} = \sqrt{\frac{1}{N_\mathrm{valid}} \sum_{k:\,\neg\mathrm{sat}[k]} (y_k - \hat{y}_k)^2}$$

### 단계 7: 경계 클램프
- `Kp ∈ [0, 1]`
- `Ti ∈ [0.1, 250]`
- `Td ∈ [0, 10]`
- NaN/Inf 처리

### 단계 8: 게임 PID 에 적용
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

### 7.2 멀티사인 (기본)
12 성분, 로그 간격 (`fBase` ~ `fMax`), 슈뢰더 위상.

$$x(t) = \sum_{i=0}^{11} \frac{A}{\sqrt{12}} \sin(2\pi f_i t + \phi_i), \quad \phi_i = -\frac{\pi i (i+1)}{12}$$

슈뢰더 위상 → 피크 팩터 ≈ √2 (균일 사인). 같은 RMS 진폭으로 가장 큰 SNR 확보.

### 7.3 Step Prelude (DC 정보 주입)
멀티사인은 DC 성분 없음 → FRIT 의 DC 동작 매칭이 외삽 영역. 첫 0.5초 일정 SP offset (= amp) → DC 보강.

### 7.4 적응형 진폭
- `yStd/amp < 0.15` AND `uStd < 0.1` → 진폭 2배 증가 (정보 부족)
- `uStd > 0.7` → 진폭 절반 (포화 근접)
- `yStd/amp > 1.5` → 진폭 절반 (과응답)
- 변경 후 3초 쿨다운 (chatter 방지)

---

## 8. 포화 처리 — 다층 방어

`|u| ≥ 0.98` 이면 비선형 영역 → FRIT 의 선형 LTI 가정 위반. 5층으로 대응:

### 8.1 실시간 가진 회피 (`ApplyExcitation`)
가진 적용 시 `|u|` 가 0.98 근처면 가진 진폭을 자동 축소 (스케일 0.1 ~ 1.0). 포화를 사전 차단.

### 8.2 양방향 적응형 진폭
위 §7.4 — `uStd > 0.7` 이면 진폭 절반.

### 8.3 짧은 포화: 가중치로 처리 (NEW)
`ConsecutiveSaturated < 10` (≈0.2초) 인 짧은 blip 은 데이터에 저장 + `Saturated[k] = true` 플래그. FRIT 비용에서 `w[k] = 1e-3` 으로 down-weight → 식별에 거의 영향 없음.

이 방식의 장점:
- 데이터 시간적 연속성 보존 (블록 분리 불필요)
- 비포화 인접 샘플은 정상 가중치로 활용
- 시간 영역 FRIT 라 spectral contamination 없음

### 8.4 긴 포화: 블록 분리 + 드롭
`ConsecutiveSaturated ≥ 10` 이면 그 틱 데이터를 저장 자체에서 드롭, `BlockStarts` 에 분기점 추가. **LTI 가정이 깊게 깨진 구간은 가중치만으로 부족** 하므로 보수적으로 데이터에서 완전 배제.

### 8.5 최장 블록 선택 + fail-fast
`PickLongestBlock` 으로 y 변동 충분한 가장 긴 블록만 식별에 사용. 전체 `satRatio > 50%` 면 자동 튜닝 실패 (가진 진폭이 과함).

---

## 9. FTD 특이사항

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

## 10. 알려진 한계

1. **초기 컨트롤러 안정성 가정.** FRIT 는 폐루프 데이터를 전제. 초기 PID 가 발산 직전이면 데이터 자체가 비선형 transient 라 결과 신뢰도 ↓.

2. **DC 정보 부족 시 Ti 부정확.** 멀티사인에 DC 가 없으므로 step prelude 가 있어도 적분 모드 가진은 약함. Ti 가 250 (=off) 에 가까이 수렴하면 수동 조정 필요.

3. **비선형성.** 큰 진폭에서는 LTI 근사가 깨짐. 적응형 진폭이 자동 축소하지만, 본질적으로 비선형이 강한 경우 (예: 고기동 영역) 는 한계.

4. **LM local minima.** 비선형 최적화의 본질적 한계. 초기 시드 (현재 PID) 가 멀리 떨어져 있으면 다른 minimum 에 수렴 가능. Ts 스캔이 부분적인 안전망.

5. **1/C(z) 안정성 제약.** 매우 큰 Td 나 매우 작은 Ti 에서 역필터 zero 가 단위원 밖으로 → soft barrier 가 작동해서 LM 이 그 영역을 못 들어감. 가끔 합리적이지만 "이상한" PID 값 (예: 매우 빠른 inner loop) 이 그 경계 근처면 도달 못 함.

6. **계산 비용.** Ts 10단계 × LM 30 iter × (역필터 + M 캐스케이드) ≈ N · 30 · 10 · (~4 cost evals) flop. 주파수 영역보다 빠르지만 (FFT 없음) 여전히 블록 길이에 선형 비례.

---

## 11. 참고문헌

- **FRIT 원본**: Soma, S., Kaneko, O., & Fujii, T. (2004). *A new method of controller parameter tuning based on input-output data — Fictitious Reference Iterative Tuning (FRIT)*. IFAC Workshop on Adaptation and Learning in Control and Signal Processing.
- **FRIT 강건성 (vs VRFT)**: Kaneko, O. (2013). *Data-driven controller tuning: FRIT approach*. IFAC Proceedings Volumes 46(11).
- **VRFT (참고용)**: Campi, M.C., Lecchini, A., & Savaresi, S.M. (2002). *Virtual reference feedback tuning: a direct method for the design of feedback controllers*. Automatica 38(8).
- **Levenberg-Marquardt**: Moré, J.J. (1978). *The Levenberg-Marquardt algorithm: implementation and theory*. Numerical Analysis.
- **Tustin bilinear transform**: Oppenheim, A.V. & Schafer, R.W. *Discrete-Time Signal Processing*, ch. 7.
- **MathNet LM 구현**: `MathNet.Numerics.Optimization.LevenbergMarquardtMinimizer`
