Research-only response-cap ablation. Not linked into PIDSupporterCore or the installed game.

Prepare original records (requires local Duster/Spirit logs and existing synthetic-one-se fixtures):
python tests/prepare_cap_crosscheck.py
Run the standalone native IV model comparison:
dotnet run --project tests/ResponseCapAudit -c Release -- tests/output/response-cap-crosscheck-20261025
Evaluate frozen models on the synthetic plants and corrected conditional BP replay:
python tests/grade_cap_crosscheck.py
python tests/grade_cap_real.py
python tests/grade_cap_new_bp.py

Run from the parent workspace directory, using PIDSupporter/ prefixes for the commands above.
The scripts currently resolve paths from that workspace. See RESULTS_KO.txt for scope and limitations.
Game records, generated fits, and account-specific publishing state are not included in Git.
