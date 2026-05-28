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
                    "1. Fly steady (level, no maneuvers, no combat)\n" +
                    "   안정 비행 (수평, 기동 X, 전투 X)\n\n" +
                    "2. Press [Auto Tune] — collects ~60s, sweeps nM × Ts\n" +
                    "   [Auto Tune] 누르기 — 약 60초 수집, nM × Ts 스윕\n\n" +
                    "3. Check Result panel — see Kp/Ti/Td ± SE\n" +
                    "   결과 패널 확인 — Kp/Ti/Td ± SE 표시\n\n" +
                    "4. Press [Apply] to write to PID\n" +
                    "   [Apply] 눌러 PID 에 반영\n\n" +
                    "5. (Optional) Repeat steps 2-4 — see 'Iterative Tuning' below\n" +
                    "   (선택) 2-4 반복 — 아래 'Iterative Tuning' 참조"
                ),
                M.m<VariableControllerMaster>(new ToolTip(
                    "Standard FRIT auto-tune workflow. Steps 2-4 may be repeated for refinement.\n---\n" +
                    "표준 FRIT 자동 튜닝 워크플로우. 정제를 위해 2-4 반복 가능.", 300f))
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
                    "Why repeat? Closed-loop ID limitation.\n" +
                    "왜 반복? 폐루프 식별의 본질적 한계.\n\n" +
                    "Round 1: weak PID → low-band excitation strong → Kp/Ti accurate, Td unreliable\n" +
                    "1차: 약한 PID → 저주파 자극 강함 → Kp/Ti 정확, Td 신뢰 X\n\n" +
                    "Round 2: better PID → high-band excitation strong → Td accurate too\n" +
                    "2차: 나은 PID → 고주파 자극 강함 → Td 도 정확\n\n" +
                    "Tip: if Td shows [uncertain], reduce Td manually (try 0.1) then re-tune\n" +
                    "팁: Td 에 [uncertain] 뜨면 수동으로 줄이고 (예: 0.1) 재튜닝"
                ),
                M.m<VariableControllerMaster>(new ToolTip(
                    "Each round excites where the controller is currently weakest. Td needs high-band " +
                    "excitation which only appears after Kp/Ti are reasonable.\n---\n" +
                    "각 라운드는 현재 약한 대역을 자극. Td 는 고주파 자극이 필요한데, 이는 " +
                    "Kp/Ti 가 어느정도 잡힌 후에야 생김.", 320f))
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
                    "γ² (Coherence): data quality per band\n" +
                    "γ² (코히어런스): 대역별 데이터 품질\n" +
                    "  · > 0.7 reliable, < 0.3 noise-dominated\n\n" +
                    "|S| (Sensitivity): controller strength per band\n" +
                    "|S| (감도): 대역별 제어 약함 정도\n" +
                    "  · low S = strong control there (GOOD)\n" +
                    "  · S_high ≈ 1 is NORMAL (Bode integral — can't beat physics)"
                ),
                M.m<VariableControllerMaster>(new ToolTip(
                    "SE from Cramér-Rao bound. γ² from Welch coherence. |S| from cross-spectrum.\n" +
                    "Bode's integral theorem forbids low |S| at all frequencies.\n---\n" +
                    "SE 는 Cramér-Rao bound. γ² 는 Welch 코히어런스. |S| 는 cross-spectrum.\n" +
                    "Bode 의 적분 정리: 모든 주파수에서 |S| 작은 건 물리적으로 불가능.", 320f))
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
                    "  · Collects data + sweeps nM ∈ {2,3,4} × Ts ∈ {0.1,0.3,1,3,10}\n" +
                    "  · ~30s sweep, writes best nM/Ts back to sliders\n" +
                    "  · 데이터 수집 + nM×Ts 자동 sweep, best 를 슬라이더에 기록\n\n" +
                    "[Compute (FRIT)]\n" +
                    "  · Uses CURRENT slider Ts and nM (no sweep)\n" +
                    "  · Useful for: tweaking Ts/nM manually after Auto Tune\n" +
                    "  · 현재 슬라이더의 Ts/nM 직접 사용 (sweep 없음)\n" +
                    "  · 용도: Auto 후 Ts/nM 수동 미세 조정"
                ),
                M.m<VariableControllerMaster>(new ToolTip(
                    "Auto Tune does the heavy lift. Compute is for follow-up manual experimentation.\n---\n" +
                    "Auto Tune 이 무거운 작업. Compute 는 후속 수동 실험용.", 320f))
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
                    "Td huge with [uncertain] / Td 큰데 [uncertain]:\n" +
                    "  → Reduce Td manually (e.g., 0.1), Apply, re-tune\n" +
                    "  → Td 수동으로 줄이고 (예: 0.1) Apply 후 재튜닝\n\n" +
                    "All SE [uncertain] / 모두 [uncertain]:\n" +
                    "  → Data too short or too noisy. Extend recording, fly steadier\n" +
                    "  → 데이터 짧거나 노이즈 많음. 녹화 연장, 더 안정 비행\n\n" +
                    "γ² < 0.3 in some band / 일부 대역 γ² < 0.3:\n" +
                    "  → That band is noise-dominated, ID there unreliable\n" +
                    "  → 그 대역 노이즈 우세, ID 신뢰 X\n\n" +
                    "Limit cycle warning / Limit cycle 경고:\n" +
                    "  → Current PID is oscillating. Reduce Kp first, re-tune\n" +
                    "  → 현재 PID 가 진동 중. Kp 낮추고 재튜닝\n\n" +
                    "Saturation warning / 포화 경고:\n" +
                    "  → Excitation too strong or setpoint near actuator limit\n" +
                    "  → 가진 너무 세거나 SP 가 액추에이터 한계 근처"
                ),
                M.m<VariableControllerMaster>(new ToolTip(
                    "Common issues and solutions.\n---\n" +
                    "자주 발생하는 문제와 해결법.", 300f))
            ));
        }
    }
}
