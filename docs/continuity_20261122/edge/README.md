# Reference photograph and dual registration, 22 November 2026

The selected offline POINTS candidate combines one corrected reference knot with two complete three-patch sets. Its raw counterpart is retained for comparison. The original checkout was `1e30933`. The results below use the archived source snapshots, with no camera or actuator access. The geometry owner evaluates the final source/configuration binding, the independent placements and the physical tilt measurements. [Geometry report](../geometry/REPORT.md)

## Reference photograph

The only input to the knot measurement was `moving_reference.png`, SHA-256 `3c6da668646e530e87fc80368bd68fdef1be0c5f59344afd466ba7cc1505eff3`. The reference depicts the bright leading lip immediately above the printed graduations. We marked the rising grayscale edge on 19 vertical transects. We then refined the trace every 16 px using the median Sobel response in a 17-column curved strip. At each original knot we compared the measured lip with its existing ±5 px search range. Only x=657.760 fell outside. The corrected y is 649.047 px, previously 642.231 px. The search window, supported-column fraction of 0.55, positive response above 35, coordinate axis and ROI anchors keep their original values. No clip labels or output coordinates entered the knot calculation. [Photo extraction](photo_points.py) · [Single-knot rule](correct_knot.py)

| Second −5 frame | Original gray columns | Corrected gray columns | Required |
|---:|---:|---:|---:|
| 0 | 451 | 521 | 473/859 |
| 1 | 453 | 517 | 473/859 |
| 2 | 463 | 527 | 473/859 |
| 22 | 192 | 207 | 474/861 |
| 23 | 168 | 182 | 474/861 |
| 24 | 237 | 268 | 474/861 |

Frames 22–24 retain insufficient grayscale evidence. The blue channel has 777–795 supported columns there, recorded for diagnosis only. The first three frames now have a measured rail. Four normal acquisition frames and the three geometry rejects account for all seven missing positions. On the same glint-enabled code snapshot the existing profile yielded 140/150 and the single correction yielded 143/150. Historical pre-glint coverage was 137/150. Substituting the entire dense photo trace changed downstream circle fits, including center-y jumps of about 10 px, and yielded 119/150. Its configs live under `rejected_dense/`. They retain the original Jetson-relative asset paths and can be replayed from the archived Jetson directory. [Measured six-frame profiles](six_frame_profiles.csv) · [Single-profile placement comparisons](heldout_comparison.csv)

## Three-patch identity sets

Every configuration uses the same reference photograph and the same reference coordinate system. The two original box sets share their first patch. Both mixed combinations were tested with exactly the same timing and processed frame IDs. The dual configuration starts with the digit patches and tries the complete left-print set after an identity failure. The alternate starts with cleared pose history. A confident competing peak in the failed set vetoes a proposed switch. A successful set becomes the first set for the next frame. Registration records carry `reference_set`; the production CSV records `primary/` or `fallback/` in `registration_mode`. The current frame must still pass the grayscale edge test. Failure of that edge test ends processing.

| Patches | +6 fit /150 | Moving free /366 | Second −5 /150 |
|---|---:|---:|---:|
| Left print | 146 | 8 | 143 |
| Digits middle, left-print right | 146 | 40 | 143 |
| Left-print middle, digits right | 0 | 67 | 143 |
| Digits | 0 | 159 | 143 |
| Dual, digits first | 146 | 175 | 143 |

All 150 +6 frames used the fallback set in the dual replay. On moving free, 331 rails used the primary and 11 the fallback. Of 175 measured centers, independent image labels classified 167 inside and eight at the border. All 16 additional measurements over digits-only were inside. Fast motion gave 432/833 and parked free gave 72/528, matching digits-only on both. Across these three processed schedules, there were zero valid outputs labelled outside or uncertain. These are measured pixel outputs. Moving-centimeter accuracy requires independent physical ground truth. [Same-frame totals and rejection counts](patch_comparison.csv) · [Replay driver](mixed_patches.py)

The dual POINTS configuration passed eight historical held-out placements and the independent zero check. Second −5 reached 143/150 with MAE 0.08680 cm and maximum error 0.14726 cm. The largest historical POINTS error was 0.20442 cm. Raw failed second −3 at 0.33807 cm maximum. On the full fixed-O tilt clip 886/895 rails passed and 820/895 positions were measured. Static edgeknot had 860/895 and 804/895 respectively. These counts measure coverage. Pose-dependent bias and the final independent calibration decision belong to the geometry report.

## Negative controls and reproduction

With the current reference and warmed matcher the intact photograph passed. Five controls failed closed: original print with erased lip, an additional off-band bright curve, a short in-band fragment, doubled print, and a blank frame. Separate tests confirmed that a duplicated central digit patch vetoes an otherwise valid alternate, an alternate starts cold and remains preferred after success, and an edge failure keeps the alternate closed. The three dual tests, 28 ruler tests and six rail-validity tests passed. [Control results](negative_controls.csv) · [Dual-specific tests](../../../Jetson/tests/test_dual_upper_edge.py)

Run from the repository root. The first two commands recompute and compare the photo-derived configurations. The third replays the unit controls.

```sh
python3 -B docs/continuity_20261122/edge/photo_points.py
python3 -B docs/continuity_20261122/edge/correct_knot.py
PYTHONPATH=Jetson python3 -B -m unittest discover -s Jetson/tests -p test_dual_upper_edge.py
```

Full per-frame CSVs, source snapshots and fit/holdout summaries are archived at `/home/user/桌面/jetson_nano/7/motion_20261121/edge_evaluation_20261122`. The `dual_trial` snapshot predates the lead's guarded blur integration. Its `heldouts/heldouts.csv` contains raw and POINTS placement errors. The archived `photo_replay` was an initial incomplete run in a changing checkout; all quoted dense results use `current_photo`, its frozen replacement. To repeat a paired dynamic trial use `mixed_patches.py ARCHIVE/dual_trial RUNS dual moving_free` with a fresh output directory containing a copy of the archived `Jetson` directory. The replay driver verifies frame IDs and capture times. [Archive SHA-256 inventory](archive_sha256.json)
