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
            this.Name = new Content("Guide / 안내", new ToolTip("How to use the FRIT auto-tuner.\n---\nFRIT 자동 튜너 사용법.", 220f), "guide");
        }

        public override void Build()
        {
            // ── Before Tuning / 튜닝 전 ──
            ScreenSegmentStandard seg1 = base.CreateStandardSegment(InsertPosition.OnCursor);
            seg1.BackgroundStyleWhereApplicable = ConsoleStyles.Instance.Styles.Segments.OptionalSegmentDarkBackgroundWithHeader.Style;
            seg1.NameWhereApplicable = "Before Tuning / 튜닝 전";
            seg1.SpaceAbove = 10f;
            seg1.SpaceBelow = 5f;

            seg1.AddInterpretter(new SubjectiveDisplay<VariableControllerMaster>(
                this._focus,
                M.m<VariableControllerMaster>(_ =>
                    "1. Fly steady (level flight, no maneuvers)\n" +
                    "   안정 비행 (수평 비행, 기동 없음)\n\n" +
                    "2. Avoid combat or collisions\n" +
                    "   전투나 충돌을 피하세요\n\n" +
                    "3. Ensure the PID axis is active\n" +
                    "   PID 축이 활성화되어 있는지 확인"
                ),
                M.m<VariableControllerMaster>(new ToolTip(
                    "FRIT reference-model matching works best on linear, time-invariant plants.\n" +
                    "Stable flight provides the best data quality.\n---\n" +
                    "FRIT 의 참조 모델 매칭은 선형·시불변 플랜트에서 가장 정확합니다.\n" +
                    "안정 비행이 최고의 데이터 품질을 제공합니다.", 300f))
            ));

            // ── During Tuning / 튜닝 중 ──
            ScreenSegmentStandard seg2 = base.CreateStandardSegment(InsertPosition.OnCursor);
            seg2.BackgroundStyleWhereApplicable = ConsoleStyles.Instance.Styles.Segments.OptionalSegmentDarkBackgroundWithHeader.Style;
            seg2.NameWhereApplicable = "During Tuning / 튜닝 중";
            seg2.SpaceAbove = 5f;
            seg2.SpaceBelow = 5f;

            seg2.AddInterpretter(new SubjectiveDisplay<VariableControllerMaster>(
                this._focus,
                M.m<VariableControllerMaster>(_ =>
                    "1. The vehicle will wobble — this is normal\n" +
                    "   기체가 흔들립니다 — 정상입니다\n\n" +
                    "2. Do NOT maneuver during data collection\n" +
                    "   데이터 수집 중 기동하지 마세요\n\n" +
                    "3. Wait ~20 seconds for collection to finish\n" +
                    "   수집 완료까지 약 20초 대기\n\n" +
                    "4. If too much saturation: reduce excitation amplitude\n" +
                    "   포화가 많으면: 가진 진폭을 줄이세요"
                ),
                M.m<VariableControllerMaster>(new ToolTip(
                    "Excitation signal is added to the setpoint to collect data.\n" +
                    "Saturation (|u| near 1) corrupts data quality.\n---\n" +
                    "가진 신호가 목표값에 더해져 데이터를 수집합니다.\n" +
                    "포화(|u|≈1)는 데이터 품질을 훼손합니다.", 300f))
            ));

            // ── After Tuning / 튜닝 후 ──
            ScreenSegmentStandard seg3 = base.CreateStandardSegment(InsertPosition.OnCursor);
            seg3.BackgroundStyleWhereApplicable = ConsoleStyles.Instance.Styles.Segments.OptionalSegmentDarkBackgroundWithHeader.Style;
            seg3.NameWhereApplicable = "After Tuning / 튜닝 후";
            seg3.SpaceAbove = 5f;
            seg3.SpaceBelow = 5f;

            seg3.AddInterpretter(new SubjectiveDisplay<VariableControllerMaster>(
                this._focus,
                M.m<VariableControllerMaster>(_ =>
                    "1. Press 'Apply' to write PID values\n" +
                    "   '적용'을 눌러 PID 값 반영\n\n" +
                    "2. Test the response — observe if stable\n" +
                    "   응답을 테스트 — 안정적인지 확인\n\n" +
                    "3. Ti may need manual adjustment\n" +
                    "   Ti는 수동 미조정이 필요할 수 있음\n\n" +
                    "4. Re-tune other axes if needed (Roll→Pitch→Yaw)\n" +
                    "   필요 시 다른 축도 재튜닝 (롤→피치→요)"
                ),
                M.m<VariableControllerMaster>(new ToolTip(
                    "FRIT may converge to large Ti when the integral mode is poorly excited.\n" +
                    "If oscillation occurs, increase Ti manually.\n" +
                    "Tune axes sequentially for best results.\n---\n" +
                    "적분 모드 가진이 약하면 FRIT 가 큰 Ti 로 수렴할 수 있습니다.\n" +
                    "진동이 발생하면 Ti를 수동으로 올리세요.\n" +
                    "최상의 결과를 위해 축을 순차적으로 튜닝하세요.", 300f))
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
                    "Kp too low / Kp가 너무 낮음:\n" +
                    "  → Increase excitation amplitude or cutoff Hz\n" +
                    "  → 가진 진폭 또는 컷오프 주파수를 올리세요\n\n" +
                    "Kp negative / Kp가 음수:\n" +
                    "  → Poor data quality. Fly steady and retry\n" +
                    "  → 데이터 품질 불량. 안정 비행 후 재시도\n\n" +
                    "Oscillation after apply / 적용 후 진동:\n" +
                    "  → Increase Ti (weaker integral)\n" +
                    "  → Ti를 올리세요 (적분을 약하게)\n\n" +
                    "Td = 0 / D항 없음:\n" +
                    "  → Reduce model delay (τ) or increase cutoff\n" +
                    "  → 모델 지연(τ)을 줄이거나 컷오프를 올리세요\n\n" +
                    "Ti = 250 (no integral) / Ti=250 (적분 없음):\n" +
                    "  → Integral mode poorly excited. Set Ti manually or extend recording\n" +
                    "  → 적분 가진 부족. Ti 수동 설정 또는 녹화 시간 연장"
                ),
                M.m<VariableControllerMaster>(new ToolTip(
                    "Common issues and solutions.\n---\n" +
                    "자주 발생하는 문제와 해결법.", 300f))
            ));
        }
    }
}
