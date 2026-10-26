using System;
using System.Collections.Generic;
using System.Numerics;
using MathNet.Numerics.LinearAlgebra;
partial class Program {
static readonly MatrixBuilder<double> MB = Matrix<double>.Build;
static readonly VectorBuilder<double> VB = MathNet.Numerics.LinearAlgebra.Vector<double>.Build;
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