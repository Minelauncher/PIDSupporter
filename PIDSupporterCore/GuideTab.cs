// ============================================================================
// GuideTab.cs — FRIT 사용 안내 탭
// ============================================================================

using BrilliantSkies.Ai.Control.Pids;
using BrilliantSkies.Ui.Consoles;
using BrilliantSkies.Ui.Consoles.Getters;
using BrilliantSkies.Ui.Consoles.Interpretters.Subjective;
using BrilliantSkies.Ui.Consoles.Segments;
using BrilliantSkies.Ui.Consoles.Styles;
using BrilliantSkies.Ui.Tips;

namespace PIDSupporter
{
    public class GuideTab : SuperScreen<VariableControllerMaster>
    {
        public GuideTab(ConsoleWindow window, VariableControllerMaster focus) : base(window, focus)
        {
            this.Name = new Content("Guide", new ToolTip("How to use the FRIT auto-tuner.\n---\nFRIT 자동 튜너 사용법.", 220f), "guide");
        }

        public override void Build()
        {
            // ── Workflow / 작업 흐름 ──
            ScreenSegmentStandard seg1 = base.CreateStandardSegment(InsertPosition.OnCursor);
            seg1.BackgroundStyleWhereApplicable = ConsoleStyles.Instance.Styles.Segments.OptionalSegmentDarkBackgroundWithHeader.Style;
            seg1.NameWhereApplicable = "Workflow / 작업 흐름";
            seg1.SpaceAbove = 10f;
            seg1.SpaceBelow = 5f;

            seg1.AddInterpretter(new SubjectiveDisplay<VariableControllerMaster>(
                this._focus,
                M.m<VariableControllerMaster>(_ =>
                    "Two tuning paths — pick one:\n" +
                    "튜닝 경로 두 개 — 골라 쓰기:\n\n" +
                    "── [Auto Tune] (FRIT, ~60s) ──\n" +
                    "  · Broadband PRBS + γ²-weighted tracking + Skogestad cap\n" +
                    "  · Ts auto-derived from closed-loop bandwidth (no sweep)\n" +
                    "  · Best for: complex plants, fine-grained ID\n" +
                    "  · 복잡 plant, 정밀 식별. Ts 는 데이터에서 자동 산출\n\n" +
                    "── [Quick Tune (Relay)] (~30s) ──\n" +
                    "  · Åström-Hägglund relay feedback + Ziegler-Nichols\n" +
                    "  · Industry standard #1, vessel oscillates around SP\n" +
                    "  · 산업 표준 #1, 함체가 SP 주변에서 진동\n\n" +
                    "Recommended workflow:\n" +
                    "권장 흐름:\n" +
                    "  1. Quick Tune first (fast baseline PID)\n" +
                    "     Quick Tune 먼저 (빠른 baseline)\n" +
                    "  2. [Apply] result\n" +
                    "  3. Auto Tune for refinement (better identification)\n" +
                    "     Auto Tune 으로 정밀화\n" +
                    "Both work independently too.\n" +
                    "각각 단독 사용도 가능."
                ),
                M.m<VariableControllerMaster>(new ToolTip(
                    "FRIT: data-driven broadband ID. Quick Tune: industrial-standard relay feedback.\n---\n" +
                    "FRIT: 데이터 기반 broadband 식별. Quick Tune: 산업 표준 relay 피드백.", 300f))
            ));

            // ── Auto Tune (FRIT) details ──
            ScreenSegmentStandard segFrit = base.CreateStandardSegment(InsertPosition.OnCursor);
            segFrit.BackgroundStyleWhereApplicable = ConsoleStyles.Instance.Styles.Segments.OptionalSegmentDarkBackgroundWithHeader.Style;
            segFrit.NameWhereApplicable = "Auto Tune (FRIT) Details";
            segFrit.SpaceAbove = 5f;
            segFrit.SpaceBelow = 5f;

            segFrit.AddInterpretter(new SubjectiveDisplay<VariableControllerMaster>(
                this._focus,
                M.m<VariableControllerMaster>(_ =>
                    "Pipeline (~60s recording + ~5s compute):\n" +
                    "파이프라인 (60초 수집 + 5초 계산):\n\n" +
                    "  1. 3s diagnose (excitation OFF)\n" +
                    "     3초 진단\n" +
                    "  2. Recording (up to 60s): PRBS excitation\n" +
                    "     60초 수집: PRBS 가진\n" +
                    "     - Adaptive bit_ticks (low/mid/high band targeting)\n" +
                    "     - Saturation-aware amplitude (target 10-25%)\n" +
                    "  3. Compute:\n" +
                    "     - Closed-loop bandwidth ω_B from |T̄(jω)|² = 0.5\n" +
                    "       (Bartlett 3-bin avg, Skogestad-Postlethwaite §2.4.5)\n" +
                    "     - Ts = √(2^(1/nM)-1)/(0.2·ω_B) — auto-derived per nM\n" +
                    "     - nM ∈ {2,3,4} sweep × 9-seed multistart LM\n" +
                    "     - Skogestad Td cap: Td ≤ min(Ti/4, 1/ω_B)\n\n" +
                    "Result message format:\n" +
                    "결과 메시지 형식:\n" +
                    "  FRIT (CL-BW Ts=..., nM=...; ω_B=...rad/s) → Kp=... Ti=... Td=...\n" +
                    "  (cost=..., Td cap (X→Y by 1/ω_B), converged)\n\n" +
                    "What 'Td cap' means:\n" +
                    "'Td cap' 의미:\n" +
                    "  LM gave Td=X, capped to Y by realizability rule.\n" +
                    "  LM 이 X 줬는데 realizability 규칙으로 Y 로 제한됨."
                ),
                M.m<VariableControllerMaster>(new ToolTip(
                    "Ts is derived from data (not swept). nM sweep keeps model order selection. " +
                    "Skogestad cap prevents Td drift.\n---\n" +
                    "Ts 는 데이터에서 (sweep X). nM 만 sweep. Skogestad cap 으로 Td drift 차단.", 320f))
            ));

            // ── Quick Tune (Relay feedback) ──
            ScreenSegmentStandard segQt = base.CreateStandardSegment(InsertPosition.OnCursor);
            segQt.BackgroundStyleWhereApplicable = ConsoleStyles.Instance.Styles.Segments.OptionalSegmentDarkBackgroundWithHeader.Style;
            segQt.NameWhereApplicable = "Quick Tune (Relay + ZN)";
            segQt.SpaceAbove = 5f;
            segQt.SpaceBelow = 5f;

            segQt.AddInterpretter(new SubjectiveDisplay<VariableControllerMaster>(
                this._focus,
                M.m<VariableControllerMaster>(_ =>
                    "Phases (~30s total):\n" +
                    "단계 (약 30초):\n" +
                    "  3s diag → relay warm-up (~2 cycles) → measure (~3 cycles)\n\n" +
                    "What happens:\n" +
                    "동작:\n" +
                    "  PID is temporarily replaced by ±h relay.\n" +
                    "  Vessel naturally oscillates around current SP.\n" +
                    "  PID 가 잠시 ±h relay 로 교체. 함체가 SP 주변 진동.\n\n" +
                    "Sliders:\n" +
                    "슬라이더:\n" +
                    "  · h (Relay amplitude): bigger = bigger oscillation\n" +
                    "    h 큼 = 진동 큼 (SNR↑ but 함체 더 흔들림)\n" +
                    "  · ε (Hysteresis): noise rejection (0 = pure relay)\n" +
                    "    노이즈 거부 (0 = 순수 relay)\n\n" +
                    "Method: A, T measured → K_c=4h/(πA), T_c=T\n" +
                    "        Ziegler-Nichols: Kp=0.6·K_c, Ti=0.5·T_c, Td=0.125·T_c\n\n" +
                    "Safer than open-loop ZN: relay output bounded ±h.\n" +
                    "Open-loop ZN 보다 안전: relay 출력 ±h 한정."
                ),
                M.m<VariableControllerMaster>(new ToolTip(
                    "Åström-Hägglund 1984 + Ziegler-Nichols 1942.\n" +
                    "The most cited industrial PID auto-tuning method.\n---\n" +
                    "산업 PID auto-tuning 의 가장 인용 많은 표준.", 320f))
            ));

            // ── Iterative Tuning / 반복 튜닝 ──
            ScreenSegmentStandard segIter = base.CreateStandardSegment(InsertPosition.OnCursor);
            segIter.BackgroundStyleWhereApplicable = ConsoleStyles.Instance.Styles.Segments.OptionalSegmentDarkBackgroundWithHeader.Style;
            segIter.NameWhereApplicable = "Iterative Tuning / 반복 튜닝";
            segIter.SpaceAbove = 5f;
            segIter.SpaceBelow = 5f;

            segIter.AddInterpretter(new SubjectiveDisplay<VariableControllerMaster>(
                this._focus,
                M.m<VariableControllerMaster>(_ =>
                    "Why repeat? CL-BW Ts targets CURRENT closed-loop bandwidth.\n" +
                    "왜 반복? CL-BW Ts 가 현재 닫힌루프 대역폭에 맞춰짐.\n\n" +
                    "Each round pushes ω_B up → next Ts tighter → crisper response.\n" +
                    "매 round 마다 ω_B 증가 → 다음 Ts 더 작아짐 → 응답 더 빨라짐.\n\n" +
                    "Typical convergence: 2-3 rounds to plant capability limit.\n" +
                    "보통 2-3 round 에 plant 한계 도달.\n\n" +
                    "Warning signs (stop and revert):\n" +
                    "위험 신호 (중단하고 직전 결과 사용):\n" +
                    "  · Kp halves or doubles between rounds\n" +
                    "    round 간 Kp 가 반/배로 swing\n" +
                    "  · Response oscillates more each round\n" +
                    "    응답이 round 마다 더 진동\n" +
                    "  · Cap activates inconsistently\n" +
                    "    cap 활성 여부가 round 별로 변동\n\n" +
                    "Note: Convergence is NOT guaranteed in theory (Ts target moves with ω_B).\n" +
                    "      Practically usually safe due to Skogestad cap + conservative cost.\n" +
                    "참고: 이론적 수렴 보장 X (Ts target 이 ω_B 따라 움직임). 그러나 실제로는\n" +
                    "      Skogestad cap + 보수적 cost function 때문에 대부분 안전."
                ),
                M.m<VariableControllerMaster>(new ToolTip(
                    "Standard FRIT convergence proof assumes fixed M(s). We adapt M(s) bandwidth " +
                    "to data each round, which is an extension not covered by the theorem.\n---\n" +
                    "표준 FRIT 수렴 정리는 M(s) 고정 가정. 우리는 매 round M(s) bandwidth 를 " +
                    "데이터에 맞추는 확장이라 정리 적용 안 됨.", 320f))
            ));

            // ── Reading the Numbers / 숫자 읽기 ──
            ScreenSegmentStandard segRead = base.CreateStandardSegment(InsertPosition.OnCursor);
            segRead.BackgroundStyleWhereApplicable = ConsoleStyles.Instance.Styles.Segments.OptionalSegmentDarkBackgroundWithHeader.Style;
            segRead.NameWhereApplicable = "Reading the Numbers / 숫자 읽기";
            segRead.SpaceAbove = 5f;
            segRead.SpaceBelow = 5f;

            segRead.AddInterpretter(new SubjectiveDisplay<VariableControllerMaster>(
                this._focus,
                M.m<VariableControllerMaster>(_ =>
                    "SE (Standard Error): ± value next to each PID gain\n" +
                    "SE (표준오차): 각 게인 옆 ± 값\n" +
                    "  · < 10%        → reliable\n" +
                    "  · 10-30%       → reasonable\n" +
                    "  · 30-50%       → [low conf] — verify\n" +
                    "  · > 50%        → [uncertain] — do not trust\n\n" +
                    "ω_B (closed-loop bandwidth): from |T̄(jω)|² = 0.5\n" +
                    "ω_B (닫힌루프 대역폭):\n" +
                    "  · Airplane roll: ~5 rad/s (fast)\n" +
                    "  · Ship roll: ~1 rad/s (slow)\n" +
                    "  · Used to derive Ts and 1/ω_B Td cap\n" +
                    "  · Ts 와 1/ω_B Td cap 도출에 사용\n\n" +
                    "γ² (Coherence): data quality per band\n" +
                    "γ² (코히어런스): 대역별 데이터 품질\n" +
                    "  · > 0.7 reliable, < 0.3 noise-dominated\n\n" +
                    "|S| (Sensitivity): controller strength per band\n" +
                    "|S| (감도): 대역별 제어 약함 정도\n" +
                    "  · low S = strong control there (GOOD)\n" +
                    "  · S_high ≈ 1 is NORMAL (Bode integral — can't beat physics)"
                ),
                M.m<VariableControllerMaster>(new ToolTip(
                    "SE from Cramér-Rao bound. γ² from Welch coherence. |S| from cross-spectrum. " +
                    "ω_B from |T(jω)| bandwidth definition (Skogestad-Postlethwaite §2.4.5).\n---\n" +
                    "SE 는 Cramér-Rao bound. γ² 는 Welch 코히어런스. |S| 는 cross-spectrum. " +
                    "ω_B 는 |T(jω)| 대역폭 정의.", 320f))
            ));

            // ── Compute vs Auto-tune / 수동 vs 자동 ──
            ScreenSegmentStandard segMode = base.CreateStandardSegment(InsertPosition.OnCursor);
            segMode.BackgroundStyleWhereApplicable = ConsoleStyles.Instance.Styles.Segments.OptionalSegmentDarkBackgroundWithHeader.Style;
            segMode.NameWhereApplicable = "Compute vs Auto-tune / 수동 vs 자동";
            segMode.SpaceAbove = 5f;
            segMode.SpaceBelow = 5f;

            segMode.AddInterpretter(new SubjectiveDisplay<VariableControllerMaster>(
                this._focus,
                M.m<VariableControllerMaster>(_ =>
                    "[Auto Tune]\n" +
                    "  · Collects data + computes (CL-BW Ts + nM sweep)\n" +
                    "  · Auto-derived Ts written back to slider\n" +
                    "  · Best nM written back to slider\n" +
                    "  · 데이터 수집 + 자동 계산. 결과 Ts/nM 이 슬라이더 반영\n\n" +
                    "[Record start/stop]\n" +
                    "  · Manual recording (same excitation as Auto Tune)\n" +
                    "  · Useful when you want non-default recording duration\n" +
                    "  · 수동 녹화 (가진은 Auto Tune 과 동일)\n\n" +
                    "[Compute (FRIT)]\n" +
                    "  · Uses CURRENT slider Ts and nM (no auto-derivation)\n" +
                    "  · Useful for: re-running FRIT with custom Ts/nM\n" +
                    "  · 현재 슬라이더의 Ts/nM 직접 사용\n" +
                    "  · 용도: 같은 데이터로 Ts/nM 수동 미세 조정"
                ),
                M.m<VariableControllerMaster>(new ToolTip(
                    "Auto Tune does the heavy lift (CL-BW Ts + nM sweep). Compute uses slider Ts directly.\n---\n" +
                    "Auto Tune 이 자동 (CL-BW Ts + nM sweep). Compute 는 슬라이더 Ts 직접 사용.", 320f))
            ));

            // ── Troubleshooting / 문제 해결 ──
            ScreenSegmentStandard seg4 = base.CreateStandardSegment(InsertPosition.OnCursor);
            seg4.BackgroundStyleWhereApplicable = ConsoleStyles.Instance.Styles.Segments.OptionalSegmentDarkBackgroundWithHeader.Style;
            seg4.NameWhereApplicable = "Troubleshooting / 문제 해결";
            seg4.SpaceAbove = 5f;
            seg4.SpaceBelow = 10f;

            seg4.AddInterpretter(new SubjectiveDisplay<VariableControllerMaster>(
                this._focus,
                M.m<VariableControllerMaster>(_ =>
                    "Result message says 'no |T̄|²=0.5 crossing in band':\n" +
                    "결과 메시지 '|T̄|² 교차 없음':\n" +
                    "  → Starting PID too weak. Run Quick Tune first, Apply, retry.\n" +
                    "  → 시작 PID 약함. Quick Tune 먼저 → Apply → 재시도.\n\n" +
                    "'Td cap (X→Y by 1/ω_B)' message:\n" +
                    "'Td cap' 메시지:\n" +
                    "  → LM picked aggressive Td=X, capped to plant timescale Y.\n" +
                    "  → This is the safety working as intended.\n" +
                    "  → LM 이 큰 Td=X 줬는데 plant timescale Y 로 제한. 정상 동작.\n\n" +
                    "Response feels too soft / 응답이 너무 부드러움:\n" +
                    "  → CL-BW Ts matches current bandwidth. Iterate for crisper.\n" +
                    "  → Or manually reduce Ts slider, press [Compute].\n" +
                    "  → CL-BW Ts 가 현재 대역폭 매칭. 반복하면 더 빨라짐.\n" +
                    "  → 또는 Ts 슬라이더 직접 줄이고 [Compute].\n\n" +
                    "All SE [uncertain] / 모두 [uncertain]:\n" +
                    "  → Data too short or too noisy. Fly steadier, retry.\n" +
                    "  → 데이터 짧거나 노이즈 많음. 더 안정 비행 후 재시도.\n\n" +
                    "γ² < 0.3 in some band / 일부 대역 γ² < 0.3:\n" +
                    "  → That band is noise-dominated, ID there unreliable.\n" +
                    "  → 그 대역 노이즈 우세, ID 신뢰 X.\n\n" +
                    "Limit cycle warning / Limit cycle 경고:\n" +
                    "  → Current PID is oscillating. Reduce Kp first, re-tune.\n" +
                    "  → 현재 PID 진동 중. Kp 낮추고 재튜닝.\n\n" +
                    "Saturation warning / 포화 경고:\n" +
                    "  → Excitation too strong or SP near actuator limit.\n" +
                    "  → 가진 너무 세거나 SP 가 액추에이터 한계 근처."
                ),
                M.m<VariableControllerMaster>(new ToolTip(
                    "Common issues and solutions.\n---\n" +
                    "자주 발생하는 문제와 해결법.", 300f))
            ));
        }
    }
}
