using System;
using System.Collections.Generic;
using System.Threading;

namespace PIDSupporter
{
    // SISO Lipschitz REN, nx=nv=8, tanh, D22=0. The direct H parameterization
    // follows R2DN/Revay et al.; see REN-NOTICE.txt. This is not an unconstrained RNN.
    // A reusable scalar reverse-AD graph differentiates the constrained matrix
    // construction. Sequence gradients use allocation-free explicit BPTT.
    internal sealed class NativeRen
    {
        internal const int N = 8, HN = 24;
        internal const int P = 576, Y = 577, IB2 = 641, ID12 = 649, IC2 = 657, ID21 = 665,
            IBx = 673, IBv = 681, IBy = 689, IX0 = 690, ParameterCount = 698;
        internal const int A = 0, B1 = 64, B2 = 128, C1 = 136, C2 = 200,
            D11 = 208, D12 = 272, D21 = 280, Bx = 288, Bv = 296, By = 304, ExplicitCount = 305;
        internal readonly double[] Parameters = new double[ParameterCount];
        internal readonly double[] Explicit = new double[ExplicitCount];
        internal readonly double[] Gradient = new double[ParameterCount];
        private readonly Tape tape = new Tape(ParameterCount);
        private readonly int[] output = new int[ExplicitCount];
        private readonly double[] eg = new double[ExplicitCount];
        private readonly double[] state, features, predictions;
        private readonly double[] gx = new double[8], gn = new double[8], gw = new double[8];

        internal NativeRen(int samples)
        {
            state = new double[(samples + 1) * N]; features = new double[samples * N]; predictions = new double[samples];
            BuildGraph();
        }

        internal void Initialize(int seed)
        {
            Array.Clear(Parameters, 0, Parameters.Length);
            var random = new Random(seed);
            double Normal() => Math.Sqrt(-2 * Math.Log(Math.Max(1e-12, random.NextDouble()))) * Math.Cos(2 * Math.PI * random.NextDouble());
            var h = new double[HN, HN];
            for (int i = 0; i < HN; i++) h[i, i] = 1;
            for (int i = 0; i < N; i++) { double f = .95 + .04 * random.NextDouble(); h[i, 16 + i] = h[16 + i, i] = f; }
            // Upper Cholesky X so X^T X starts close to a long-memory model.
            for (int i = 0; i < HN; i++)
                for (int j = i; j < HN; j++)
                {
                    double v = h[i, j];
                    for (int k = 0; k < i; k++) v -= Parameters[k * HN + i] * Parameters[k * HN + j];
                    Parameters[i * HN + j] = i == j ? Math.Sqrt(v) : v / Parameters[i * HN + i];
                }
            double norm = 1e-6;
            for (int i = 0; i < P; i++) norm += Parameters[i] * Parameters[i];
            Parameters[P] = Math.Sqrt(norm);
            for (int i = 0; i < N; i++)
            {
                Parameters[Y + i * N + i] = 1;
                Parameters[IB2 + i] = Normal(); Parameters[ID12 + i] = Normal();
                Parameters[IC2 + i] = Normal() / Math.Sqrt(N); Parameters[ID21 + i] = Normal() / Math.Sqrt(N);
            }
            Refresh();
        }

        private void BuildGraph()
        {
            int zero = tape.Constant(0), two = tape.Constant(2), gamma = tape.Constant(5), eps = tape.Constant(1e-6);
            int norm = tape.Constant(1.1920928955078125e-7); // R2DN l2_norm default epsilon.
            for (int i = 0; i < P; i++) norm = tape.Add(norm, tape.Mul(i, i));
            int polar = tape.Div(tape.Mul(P, P), norm);
            var h = new int[HN, HN];
            var q = new int[HN]; var r = new int[HN];
            for (int i = 0; i < HN; i++)
            {
                q[i] = i < 8 ? IC2 + i : i < 16 ? ID21 + i - 8 : zero;
                r[i] = i < 8 ? zero : i < 16 ? tape.Sub(zero, ID12 + i - 8) : IB2 + i - 16;
            }
            for (int i = 0; i < HN; i++)
                for (int j = i; j < HN; j++)
                {
                    int v = zero;
                    for (int k = 0; k < HN; k++) v = tape.Add(v, tape.Mul(k * HN + i, k * HN + j));
                    v = tape.Mul(v, polar);
                    if (i == j) v = tape.Add(v, eps);
                    v = tape.Add(v, tape.Div(tape.Add(tape.Mul(q[i], q[j]), tape.Mul(r[i], r[j])), gamma));
                    h[i, j] = h[j, i] = v;
                }
            // E has positive-definite symmetric part, hence nonzero leading pivots.
            // Elimination solves E*[A B1 B2]=[H31 H32 direct.B2].
            var aug = new int[N, 25];
            for (int i = 0; i < N; i++)
                for (int j = 0; j < N; j++)
                {
                    int v = tape.Add(h[i, j], tape.Div(h[16 + i, 16 + j], tape.Constant(.999 * .999)));
                    aug[i, j] = tape.Div(tape.Add(v, tape.Sub(Y + i * N + j, Y + j * N + i)), two);
                    aug[i, 8 + j] = h[16 + i, j]; aug[i, 16 + j] = h[16 + i, 8 + j];
                }
            for (int i = 0; i < N; i++) aug[i, 24] = IB2 + i;
            for (int k = 0; k < N; k++)
                for (int i = k + 1; i < N; i++)
                {
                    int factor = tape.Div(aug[i, k], aug[k, k]);
                    for (int j = k + 1; j < 25; j++) aug[i, j] = tape.Sub(aug[i, j], tape.Mul(factor, aug[k, j]));
                }
            for (int col = 8; col < 25; col++)
                for (int i = N - 1; i >= 0; i--)
                {
                    int v = aug[i, col];
                    for (int j = i + 1; j < N; j++) v = tape.Sub(v, tape.Mul(aug[i, j], aug[j, col]));
                    aug[i, col] = tape.Div(v, aug[i, i]);
                }
            for (int i = 0; i < N; i++)
            {
                int inv = tape.Div(two, h[8 + i, 8 + i]);
                for (int j = 0; j < N; j++)
                {
                    output[A + i * N + j] = aug[i, 8 + j]; output[B1 + i * N + j] = aug[i, 16 + j];
                    output[C1 + i * N + j] = tape.Mul(inv, tape.Sub(zero, h[8 + i, j]));
                    output[D11 + i * N + j] = j < i ? tape.Mul(inv, tape.Sub(zero, h[8 + i, 8 + j])) : zero;
                }
                output[B2 + i] = aug[i, 24]; output[D12 + i] = tape.Mul(inv, ID12 + i);
                output[C2 + i] = IC2 + i; output[D21 + i] = ID21 + i;
                output[Bx + i] = IBx + i; output[Bv + i] = IBv + i;
            }
            output[By] = IBy; tape.Seal();
        }

        internal void Refresh()
        {
            tape.Forward(Parameters);
            for (int i = 0; i < ExplicitCount; i++)
            {
                Explicit[i] = tape.Values[output[i]];
                if (!Finite(Explicit[i])) throw new ArithmeticException("REN parameter conversion failed");
            }
        }

        // x and xn may be different offsets in the same array; never the same slice.
        internal double Step(double[] x, int xi, double u, double[] xn, int ni, double[] w, int wi)
        {
            var e = Explicit;
            for (int i = 0; i < N; i++)
            {
                double v = e[Bv + i] + e[D12 + i] * u;
                for (int j = 0; j < N; j++) v += e[C1 + i * N + j] * x[xi + j];
                for (int j = 0; j < i; j++) v += e[D11 + i * N + j] * w[wi + j];
                w[wi + i] = Math.Tanh(v);
            }
            double y = e[By];
            for (int i = 0; i < N; i++)
            {
                y += e[C2 + i] * x[xi + i] + e[D21 + i] * w[wi + i];
                double v = e[Bx + i] + e[B2 + i] * u;
                for (int j = 0; j < N; j++) v += e[A + i * N + j] * x[xi + j] + e[B1 + i * N + j] * w[wi + j];
                xn[ni + i] = v;
            }
            return y;
        }

        internal double Loss(double[] u, double[] y, int count, bool gradient, CancellationToken token)
        {
            Refresh(); Array.Copy(Parameters, IX0, state, 0, N);
            double loss = 0;
            for (int k = 0; k < count; k++)
            {
                if ((k & 31) == 0) token.ThrowIfCancellationRequested();
                double yp = Step(state, k * N, u[k], state, (k + 1) * N, features, k * N);
                predictions[k] = yp; double error = yp - y[k]; loss += error * error / count;
            }
            if (!gradient) return loss;
            Array.Clear(eg, 0, eg.Length); Array.Clear(gn, 0, N);
            var e = Explicit;
            for (int k = count - 1; k >= 0; k--)
            {
                if ((k & 31) == 0) token.ThrowIfCancellationRequested();
                int offset = k * N; double gy = 2 * (predictions[k] - y[k]) / count;
                Array.Clear(gx, 0, N); Array.Clear(gw, 0, N); eg[By] += gy;
                for (int i = 0; i < N; i++)
                {
                    eg[C2 + i] += gy * state[offset + i]; gx[i] += gy * e[C2 + i];
                    eg[D21 + i] += gy * features[offset + i]; gw[i] += gy * e[D21 + i];
                    eg[B2 + i] += gn[i] * u[k]; eg[Bx + i] += gn[i];
                    for (int j = 0; j < N; j++)
                    {
                        int ij = i * N + j;
                        eg[A + ij] += gn[i] * state[offset + j]; gx[j] += gn[i] * e[A + ij];
                        eg[B1 + ij] += gn[i] * features[offset + j]; gw[j] += gn[i] * e[B1 + ij];
                    }
                }
                for (int i = N - 1; i >= 0; i--)
                {
                    double v = gw[i] * (1 - features[offset + i] * features[offset + i]);
                    eg[Bv + i] += v; eg[D12 + i] += v * u[k];
                    for (int j = 0; j < N; j++)
                    { eg[C1 + i * N + j] += v * state[offset + j]; gx[j] += v * e[C1 + i * N + j]; }
                    for (int j = 0; j < i; j++)
                    { eg[D11 + i * N + j] += v * features[offset + j]; gw[j] += v * e[D11 + i * N + j]; }
                }
                Array.Copy(gx, gn, N);
            }
            tape.ClearGradient();
            for (int i = 0; i < ExplicitCount; i++) tape.Gradients[output[i]] += eg[i];
            for (int i = 0; i < N; i++) tape.Gradients[IX0 + i] += gn[i];
            tape.Backward(); Array.Copy(tape.Gradients, Gradient, ParameterCount);
            return loss;
        }

        internal double Validation(double[] u, double[] y, int start, CancellationToken token)
        {
            Loss(u, y, u.Length, false, token); double value = 0;
            for (int i = start; i < u.Length; i++) value += Math.Pow(predictions[i] - y[i], 2);
            return Math.Sqrt(value / (u.Length - start));
        }
        internal void LastState(double[] destination, int samples) => Array.Copy(state, samples * N, destination, 0, N);
        internal double[] Predictions(int count) { var copy = new double[count]; Array.Copy(predictions, copy, count); return copy; }
        internal static bool Finite(double x) => !double.IsNaN(x) && !double.IsInfinity(x);

        private sealed class Tape
        {
            private struct Node { internal byte Op; internal int L, R; internal double Value; }
            private readonly List<Node> build = new List<Node>();
            private Node[] nodes = null!;
            internal double[] Values = null!, Gradients = null!;
            private readonly int inputs;
            internal Tape(int n) { inputs = n; for (int i = 0; i < n; i++) build.Add(new Node()); }
            internal int Constant(double value) { build.Add(new Node { Value = value }); return build.Count - 1; }
            private int Op(byte op, int l, int r) { build.Add(new Node { Op = op, L = l, R = r }); return build.Count - 1; }
            internal int Add(int l, int r) => Op(1, l, r);
            internal int Sub(int l, int r) => Op(2, l, r);
            internal int Mul(int l, int r) => Op(3, l, r);
            internal int Div(int l, int r) => Op(4, l, r);
            internal void Seal()
            {
                nodes = build.ToArray(); Values = new double[nodes.Length]; Gradients = new double[nodes.Length];
                for (int i = inputs; i < nodes.Length; i++) Values[i] = nodes[i].Value;
                build.Clear();
            }
            internal void Forward(double[] parameters)
            {
                Array.Copy(parameters, Values, inputs);
                for (int i = inputs; i < nodes.Length; i++)
                {
                    var n = nodes[i];
                    switch (n.Op)
                    {
                        case 1: Values[i] = Values[n.L] + Values[n.R]; break;
                        case 2: Values[i] = Values[n.L] - Values[n.R]; break;
                        case 3: Values[i] = Values[n.L] * Values[n.R]; break;
                        case 4: Values[i] = Values[n.L] / Values[n.R]; break;
                    }
                }
            }
            internal void ClearGradient() => Array.Clear(Gradients, 0, Gradients.Length);
            internal void Backward()
            {
                for (int i = nodes.Length - 1; i >= inputs; i--)
                {
                    var n = nodes[i]; double g = Gradients[i];
                    switch (n.Op)
                    {
                        case 1: Gradients[n.L] += g; Gradients[n.R] += g; break;
                        case 2: Gradients[n.L] += g; Gradients[n.R] -= g; break;
                        case 3: Gradients[n.L] += g * Values[n.R]; Gradients[n.R] += g * Values[n.L]; break;
                        case 4: Gradients[n.L] += g / Values[n.R]; Gradients[n.R] -= g * Values[i] / Values[n.R]; break;
                    }
                }
            }
        }
    }
}
