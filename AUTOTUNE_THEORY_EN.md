# PIDSupporter: current tuning method and its limits

Reviewed against production source on 27 October 2026. Release: **1.1.5**. Recording-time validation is corrected; IV fitting, model selection and PID search policies remain as in 1.1.3.

## Recording-time validation (1.1.5)

The model sample interval is estimated as `(T[last] - T[first]) / (N - 1)`. Multiplying the median interval by sample count could falsely reject a valid 30-second record because the game supplies float timestamps. The span estimate avoids accumulating that median quantization bias. Start/end coverage is checked separately, allowing one sample of collection-boundary rounding. Every interval must still be within 10% of the estimated step; missing, duplicate, reversed or non-finite timestamps are rejected. Severely degraded clock precision may still require a game restart. A perfectly regular lower-rate record cannot be distinguished from systematic decimation from timestamps alone. This change does not bypass PID-setting change cancellation or relax the model/robustness criteria.

[한국어 설명](AUTOTUNE_THEORY_KO.md)

## Purpose and workflow

PIDSupporter is an in-game **tune helper**. It suggests settings for the user to review, apply manually and restore if necessary. It does not guarantee improved flight, stability or a globally optimal PID.

The current path is:

1. Observe the existing loop and adjust its working proportional gain before collection.
2. Freeze the PID and a square-wave setpoint excitation for a 30-second recording.
3. Fit a low-order linear model using instrumental-variable projection.
4. Simulate candidate PIDs with the game's output limits and integral handling.
5. Screen candidates using empirical frequency-response discrepancies; evaluate nearby PID combinations.
6. Rank surviving candidates using a tracking-first hierarchy and offer a manual recommendation.

The earlier FRIT/relay workflow, REN training, residual-based uncertainty radii, set-membership model ellipsoids, and continuous-frequency research certificates are **not the current production path**.

## Recorded signals

- $r$: the setpoint actually received by the PID, reconstructed as measured output plus controller error.
- $u$: the returned PID command **after** the controller's output limit.
- $y$: the measured controlled variable in the game's coordinate convention.
- Controller parameters, integral/error state, timestamps, actual injected SP and AI/base SP are also recorded.
- $q$: the independently generated, unit-amplitude excitation carrier before amplitude scaling and SP rounding. Its delayed values are the instruments.

Knowing the post-limit command avoids treating clipping as an unknown plant input. It does not remove actuator rate limits, changing control authority, coordinate coupling or nonlinear plant dynamics. These remain part of the approximate input/output behavior.

## Preparation and excitation

Preparation injects no tuning waveform. It observes four-second windows and requires two consecutive passing windows. It can raise or lower the working P gain. For example, rapid command alternation can reduce P by a factor of 0.7; low effort with persistent error can increase it. These are operating heuristics, not stability proofs. Preparation can repeat; the 30-second recording starts only after it passes.

After preparation, the final PID is used to recompute the amplitude cap:

$$A_{\max}=\min\left(5,\frac{0.1(u_{\max}-u_{\min})}{K_p}\right).$$

The starting amplitude is half this cap. An existing observed-command-peak factor between 0.5 and 1.5 is applied, without exceeding the cap. This policy does not guarantee that saturation will be avoided.

Collection uses a **0.2 Hz square wave**, a newly drawn phase and sign, and one-second cosine fades at both ends. Its tapered mean is approximately removed using 600 midpoint samples, then it is normalized. Both amplitude and working PID remain fixed during collection. The unit carrier, rather than the total AI setpoint, supplies the IV instruments. E-optimal multisine adaptation is not active in this release.

Fake Set Point does not disable injection: the game still applies SP adjustment and scaling. Changing the active Fake SP setting, PID settings or controller mode during a recording cancels collection rather than silently mixing configurations.

## Linear identification

The first 15 seconds provide training means and standard deviations. In normalized coordinates, models have no direct feedthrough:

$$\tilde y_t=\sum_{j=1}^{p}a_j\tilde y_{t-j}+\sum_{j=1}^{p}b_j\tilde u_{t-j}+c+e_t,\qquad p\in\{1,2,3\}.$$

The implementation uses one-sample-delayed input regressors. It does not separately identify a free command-delay parameter. Orders are selected from the available structures; a research 1-SE rule is not active.

Build the instrument matrix $Z$ from the carrier's current/past values up to two seconds, capped at 80 delayed samples, plus a constant. Normalize it using training rows only. Direct one-sided Jacobi SVD supplies an orthonormal basis $Q$ for its numerically resolved column space, with threshold

$$\max(N,m)\,\epsilon_{\mathrm{mach}}\,\sigma_{\max}.$$

Only numerical-zero instrument directions are removed. No coefficient ridge penalty is applied. The projected fitting objective is

$$\min_\theta\frac1N\|Q^T(y-X\theta)\|_2^2,\qquad |\lambda_i|\le\exp(-\Delta t/300).$$

The pole restriction excludes exact integrators and unstable open-loop models. Real and conjugate-complex pole structures are explored with finite multistart search. Coefficient solves use column-pivoted Householder QR and reject unresolved relative pivots. This is not a global optimization or uniqueness guarantee. The previous normalized response-amplitude cap is removed; finite-response checks remain. Coefficients are not forced positive, but a selected non-positive or non-finite DC gain cannot support a positive-PID recommendation.

IV consistency requires suitable instrument/error orthogonality, excitation and model assumptions. An externally generated carrier alone does not make nonlinear mismatch or coupled-axis effects independent of that carrier. Normalization and SVD improve numerical handling; they cannot create missing information.

## Model selection and accumulated records

The 15–20 second segment ranks models by the largest of three normalized RMS errors: free output reproduction under recorded input, existing-PID closed-loop output reproduction, and its input reproduction. The model with the smallest finite score is selected. Free reproduction uses observed initial histories, then no later output correction. Closed-loop reproduction uses recorded input initially and switches to the recorded PID/setpoint after the initial four seconds with the recorded controller state.

The last ten seconds are final diagnostics, not coefficient/order reselection. A large but finite validation error does not itself block the search.

Up to five recent recordings from the same PID window are jointly identified. Dynamic coefficients are shared; each record retains its own centering/offset, initial history and PID. Histories are not concatenated across boundaries. Closing the PID window stops collection and clears that window's accumulated history. Repeated use is not a proof of convergence to the true plant or best controller.

## Candidate performance simulation

Each candidate starts from the recent measured input/output history and the controller's endpoint integral/error state. The selected single model is simulated for 15 seconds in each sign of a smooth SP change:

$$r_k=y_{\mathrm{end}}\pm\delta(1-e^{-k\Delta t/\tau}),\qquad\delta=\max(0.5A,0.1\sigma_{y,\mathrm{train}}).$$

The response-style control selects $\tau=2$, $1.25$ or $0.5$ seconds. This is an evaluation SP timescale, not a promised vehicle response time, and it does not alter the collection waveform.

The simulated PID uses the game's float arithmetic, derivative on error, output clipping and integral-contribution clamping. $T_i\ge250$ disables I; $T_d=0$ disables D. Thruster ramping is not a separately modeled limiter. Candidate PIDs are quantized to the supported slider increments: 0.001 for P and 0.1 for time parameters, with explicit I/D-off modes.

## Empirical candidate screening

For each recording, the first 20 seconds are split into five-second windows. At frequencies $f_j=0.2+0.4j$ Hz, $j=0,\ldots,9$, below Nyquist, a constant-detrended Hann-window spectrum is computed. The radius is the largest observed discrepancy from the selected model over eligible windows/records:

$$w_j=\max_b\left|\frac{\overline{Q_{bj}}Y_{bj}}{\overline{Q_{bj}}U_{bj}}-\hat G(e^{i\omega_j})\right|.$$

Within one window the instrument cancels algebraically. This radius is an **empirical discrepancy envelope, not an IV confidence interval** or a guaranteed bound on future plant error.

For each evaluated PID, its nominal discrete-time closed-loop characteristic polynomial must pass a Schur-stability test, and every measured-frequency check must satisfy

$$w_j\left|\frac{C(e^{i\omega_j})}{1+\hat G(e^{i\omega_j})C(e^{i\omega_j})}\right|<1.$$

The expression is motivated by the small-gain condition for additive plant uncertainty. A theorem-level guarantee would require a valid uncertainty bound and the required conditions over the entire frequency range. The implementation checks ten observed frequencies: it does **not** certify between them, outside the measured range, future operating changes, axis coupling, or nonlinear saturation. It can reject useful candidates and can still accept candidates that do not work well in the game.

## Neighborhood evaluation and selection

The default is up to **27 combinations**, including simultaneous changes in all active PID parameters. Each coordinate changes by plus/minus 10%, with at least one slider increment. Off modes stay off; clamping/rounding can create duplicate points at boundaries. This is not a continuum guarantee.

Among evaluated feasible candidates:

1. Keep those whose worst-neighborhood tracking MSE is within 1% of its minimum.
2. Among them, keep those whose center tracking MSE is within 1% of its minimum.
3. Minimize center input-change-rate MSE.
4. Break exact ties using center input-effort MSE.

The tolerance has an absolute floor of $10^{-12}$. Tracking is normalized by $\delta$; input effort and slew use the output range. This hierarchy does not use a weighted sum to trade arbitrary tracking degradation against smaller input motion.

Differential evolution and local refinement explore a finite budget (approximately 10 seconds / 18,000 evaluations, with some loop-boundary overrun). They do not exhaust the full PID space. A recommendation requires more than 5% predicted improvement in worst-neighborhood tracking relative to the raw current-PID baseline. That raw baseline is evaluated separately even when the current PID fails the empirical screen. If no searched candidate passes, no new PID is recommended. Retaining the current PID is not a stability certificate for it.

The UI's predicted comparison is not measured improvement in actual flight. Manual Apply and Restore remain essential.

## Runtime and scope of claims

Identification and search use a dedicated managed C# worker. Cooperative yielding limits individual work bursts, but OS scheduling and game load mean there is no hard frame-time guarantee. No Python runtime is required in the game.

Finite-window IV fitting, bounded search, empirical screening and rolling record history do not imply parameter convergence, true optimality or actual closed-loop stability. Sensor-noise-free data can still contain external disturbances, nonlinearities, poor excitation and changing operating conditions. Successful examples do not establish universal performance.

## References and source map

- Gilson and Van den Hof, *Instrumental variable methods for closed-loop system identification*, Automatica (2005): https://www.sciencedirect.com/science/article/pii/S0005109804002845
- Forssell, *Closed-loop Identification: Methods, Theory, and Applications*: https://rt.isy.liu.se/research/pub-db-supplemental/phd/566/main.pdf
- These references motivate identification concepts; the specific preparation heuristics, finite search and empirical radius are not claimed to reproduce an entire published guaranteed controller-design method.

Production source: `AutoTuneSession.cs` (preparation/collection), `NativeLinear*.cs` and `NumericalSvd.cs` (identification), `NativeAutoTune.cs` (simulation), `PidSearch.cs` (hierarchy/search/neighborhood), `EmpiricalFrequencyGate.cs` (candidate screen), and `AutoTuneHistory.cs` (window-local accumulation).
