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
            this.Name = new Content("Guide", new ToolTip("Quick start guide.\n---\n빠른 시작 가이드.", 220f), "guide");
        }

        public override void Build()
        {
            // ── 빠른 시작 / Quick Start ──
            ScreenSegmentStandard seg1 = base.CreateStandardSegment(InsertPosition.OnCursor);
            seg1.BackgroundStyleWhereApplicable = ConsoleStyles.Instance.Styles.Segments.OptionalSegmentDarkBackgroundWithHeader.Style;
            seg1.NameWhereApplicable = "Quick Start / 빠른 시작";
            seg1.SpaceAbove = 10f;
            seg1.SpaceBelow = 5f;

            seg1.AddInterpretter(new SubjectiveDisplay<VariableControllerMaster>(
                this._focus,
                M.m<VariableControllerMaster>(_ =>
                    "1. Fly steady (no maneuvers)\n" +
                    "   안정 비행 (기동 X)\n\n" +
                    "2. Press [Quick Tune] (~30s)\n" +
                    "   [Quick Tune] 누르기 (30초)\n\n" +
                    "3. Press [Apply]\n" +
                    "   [Apply] 누르기\n\n" +
                    "4. (Optional) Press [Auto Tune] for refinement\n" +
                    "   (선택) [Auto Tune] 으로 정밀 튜닝\n\n" +
                    "Done. / 끝."
                ),
                M.m<VariableControllerMaster>(new ToolTip(
                    "Quick Tune first for a fast baseline, then optionally Auto Tune to refine.\n---\n" +
                    "Quick Tune 먼저 (빠른 baseline), 필요시 Auto Tune 정밀화.", 280f))
            ));

            // ── 버튼 설명 / Buttons ──
            ScreenSegmentStandard segBtn = base.CreateStandardSegment(InsertPosition.OnCursor);
            segBtn.BackgroundStyleWhereApplicable = ConsoleStyles.Instance.Styles.Segments.OptionalSegmentDarkBackgroundWithHeader.Style;
            segBtn.NameWhereApplicable = "Buttons / 버튼";
            segBtn.SpaceAbove = 5f;
            segBtn.SpaceBelow = 5f;

            segBtn.AddInterpretter(new SubjectiveDisplay<VariableControllerMaster>(
                this._focus,
                M.m<VariableControllerMaster>(_ =>
                    "[Quick Tune]\n" +
                    "  Vessel oscillates ±h around target.\n" +
                    "  Fast, ~30s. Industry standard.\n" +
                    "  함체가 목표값 주변 ±h 진동, 30초.\n\n" +
                    "[Auto Tune]\n" +
                    "  Small random noise injected.\n" +
                    "  Slower, ~60s. More accurate.\n" +
                    "  작은 무작위 신호, 60초. 더 정밀.\n\n" +
                    "[Apply]\n" +
                    "  Write result to PID gains.\n" +
                    "  결과를 PID 게인에 적용.\n\n" +
                    "[Reset]\n" +
                    "  Cancel and clear current run.\n" +
                    "  취소 + 초기화."
                ),
                M.m<VariableControllerMaster>(new ToolTip(
                    "Quick Tune: Relay feedback (Åström-Hägglund) + Ziegler-Nichols.\n" +
                    "Auto Tune: FRIT (Soma-Kaneko) with γ²-weighted cost + Skogestad cap.\n---\n" +
                    "내부: Quick=relay+ZN, Auto=FRIT.", 280f))
            ));

            // ── 진단 메시지 / Diagnose ──
            ScreenSegmentStandard segDiag = base.CreateStandardSegment(InsertPosition.OnCursor);
            segDiag.BackgroundStyleWhereApplicable = ConsoleStyles.Instance.Styles.Segments.OptionalSegmentDarkBackgroundWithHeader.Style;
            segDiag.NameWhereApplicable = "If diagnose fails / 진단 실패 시";
            segDiag.SpaceAbove = 5f;
            segDiag.SpaceBelow = 5f;

            segDiag.AddInterpretter(new SubjectiveDisplay<VariableControllerMaster>(
                this._focus,
                M.m<VariableControllerMaster>(_ =>
                    "First 3s checks if current PID is OK.\n" +
                    "첫 3초가 현재 PID 상태 진단.\n\n" +
                    "⚠ Limit cycle:\n" +
                    "  Current PID oscillates.\n" +
                    "  현재 PID 진동 중.\n" +
                    "  → Lower Kp, then retry / Kp 낮추고 재시도\n\n" +
                    "⚠ Persistent saturation:\n" +
                    "  Actuator stuck at limit.\n" +
                    "  액추에이터 한계 도달.\n" +
                    "  → Lower Kp/Ki or check SP / Kp/Ki 낮추거나 SP 확인"
                ),
                M.m<VariableControllerMaster>(new ToolTip(
                    "Diagnose runs with excitation OFF for 3s, measuring saturation rate and sign-change rate.\n---\n" +
                    "진단은 가진 OFF 로 3초 동안 sat rate / cross rate 측정.", 280f))
            ));

            // ── 결과 신뢰도 / Result confidence ──
            ScreenSegmentStandard segRead = base.CreateStandardSegment(InsertPosition.OnCursor);
            segRead.BackgroundStyleWhereApplicable = ConsoleStyles.Instance.Styles.Segments.OptionalSegmentDarkBackgroundWithHeader.Style;
            segRead.NameWhereApplicable = "Result confidence / 결과 신뢰도";
            segRead.SpaceAbove = 5f;
            segRead.SpaceBelow = 5f;

            segRead.AddInterpretter(new SubjectiveDisplay<VariableControllerMaster>(
                this._focus,
                M.m<VariableControllerMaster>(_ =>
                    "SE = ± value next to each gain.\n" +
                    "SE = 게인 옆 ± 값.\n\n" +
                    "  < 30%  → trust / 신뢰\n" +
                    "  30-50% → low / 낮음 — verify\n" +
                    "  > 50%  → uncertain / 불확실 — re-run\n\n" +
                    "[uncertain] flag means data wasn't informative\n" +
                    "enough — fly steadier or retry.\n" +
                    "[uncertain] 표시 = 데이터 부족, 안정 비행 후 재시도."
                ),
                M.m<VariableControllerMaster>(new ToolTip(
                    "SE from Cramér-Rao bound. Coherence γ² and sensitivity |S| panels show data quality per band.\n---\n" +
                    "SE 는 Cramér-Rao bound. γ² / |S| 패널이 대역별 데이터 품질 표시.", 280f))
            ));

            // ── 반복 사용 / Iteration ──
            ScreenSegmentStandard segIter = base.CreateStandardSegment(InsertPosition.OnCursor);
            segIter.BackgroundStyleWhereApplicable = ConsoleStyles.Instance.Styles.Segments.OptionalSegmentDarkBackgroundWithHeader.Style;
            segIter.NameWhereApplicable = "Iteration / 반복";
            segIter.SpaceAbove = 5f;
            segIter.SpaceBelow = 5f;

            segIter.AddInterpretter(new SubjectiveDisplay<VariableControllerMaster>(
                this._focus,
                M.m<VariableControllerMaster>(_ =>
                    "Repeat [Auto Tune] → [Apply] for crisper response.\n" +
                    "[Auto Tune] → [Apply] 반복 = 더 빠른 응답.\n\n" +
                    "Typical: 2-3 rounds is enough.\n" +
                    "보통 2-3 round 면 충분.\n\n" +
                    "Stop if:\n" +
                    "다음 시 중단:\n" +
                    "  · Gains swing wildly between rounds\n" +
                    "    게인이 round 마다 크게 변함\n" +
                    "  · Response gets more oscillatory\n" +
                    "    응답이 더 진동 적"
                ),
                M.m<VariableControllerMaster>(new ToolTip(
                    "Each round pushes closed-loop bandwidth higher. Convergence not theoretically " +
                    "guaranteed but practically safe due to Skogestad cap.\n---\n" +
                    "매 round 닫힌루프 대역폭 상승. 수렴 보장 X 이지만 cap 으로 안전.", 280f))
            ));

            // ── 자주 묻는 / FAQ ──
            ScreenSegmentStandard segFaq = base.CreateStandardSegment(InsertPosition.OnCursor);
            segFaq.BackgroundStyleWhereApplicable = ConsoleStyles.Instance.Styles.Segments.OptionalSegmentDarkBackgroundWithHeader.Style;
            segFaq.NameWhereApplicable = "FAQ";
            segFaq.SpaceAbove = 5f;
            segFaq.SpaceBelow = 10f;

            segFaq.AddInterpretter(new SubjectiveDisplay<VariableControllerMaster>(
                this._focus,
                M.m<VariableControllerMaster>(_ =>
                    "Q: Response a bit soft?\n" +
                    "   응답이 약간 부드러움?\n" +
                    "A: Normal. Iterate or reduce Ts slider.\n" +
                    "   정상. 반복 or Ts 슬라이더 줄임.\n\n" +
                    "Q: Td came out 0 or weird?\n" +
                    "   Td 가 0 이거나 이상함?\n" +
                    "A: Plant may not need D. Try [Compute] with smaller Ts.\n" +
                    "   D 불필요할 수도. Ts 줄여서 [Compute].\n\n" +
                    "Q: \"increase starting PID gain\" message?\n" +
                    "   \"PID 강화\" 메시지?\n" +
                    "A: Run [Quick Tune] first, [Apply], then [Auto Tune].\n" +
                    "   [Quick Tune] 먼저 → [Apply] → [Auto Tune].\n\n" +
                    "Q: Vessel oscillates during Quick Tune?\n" +
                    "   Quick Tune 중 함체 진동?\n" +
                    "A: Normal (relay limit cycle). Wait ~30s.\n" +
                    "   정상 (relay 진동). 30초 대기.\n\n" +
                    "Q: Want to learn the math?\n" +
                    "   수학 공부하고 싶음?\n" +
                    "A: See THEORY.md in mod folder.\n" +
                    "   모드 폴더의 THEORY.md 참조."
                ),
                M.m<VariableControllerMaster>(new ToolTip(
                    "Common questions and quick answers.\n---\n" +
                    "자주 묻는 질문.", 280f))
            ));
        }
    }
}
