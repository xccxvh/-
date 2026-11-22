# Static geometry and tilt audit

## NCC release decision

The final dual, guarded points model passed all 14 labeled static verification and development placements, including both independent zero checks. The two previously observed −5 clips now count as development checks. The separate blind −5 placement yielded 145/150 valid centimeter measurements (96.67%) in a production replay using the archived native model, with MAE 0.04238 cm and maximum error 0.12513 cm. Across the 14 placements, minimum coverage was 95.33% and maximum absolute error was 0.28245 cm. The validated model is loaded by Jetson/config_motion_cm_20261122.json. Camera tests and run manifests are in the parent report. The pre-NCC results below document earlier freezes.

[Validated points model](release/points_validated.json) · [Release decision and binding](release/decision.json) · [Checks including both modes and original conflicting labels](release/checks.csv)

## Pre-NCC guarded dual configuration

The production points binding is `0d746f2e36d25609e9c687cc26e6fdd11bce32f0496ec82e780d29d5b2987676`. The final independent −5 repeat yielded **121/150 (80.67%)**, with MAE 0.101 cm and maximum error 0.154 cm. This is below the 143/150 coverage threshold. Of the 29 missing outputs, 18 were identity rejections, 10 were acquisition frames and one had competing candidates. The direct checkout entry point reproduced all 150 frame statuses and axial coordinates. The static edgeknot control below remains a separate result.

Both original-exporter candidates load against the final checkout with ready=true, validated=false and centimeter output disabled. Each of −6/O/+6 contributed 146 measured frames from its own complete static recording. Consecutive frames are correlated; the units of physical replication are placements. Historical points coverage was 1164/1200, with maximum error 0.204 cm. On the original fixed-O tilt video there were 820/895 measured coordinates, mean -0.429 cm and maximum absolute error 0.523 cm. Fresh plateau interpretation and release are maintained by the operator.

[Bound points model](/home/user/桌面/jetson_nano/8/continuity_20261122/geometry_final_dual/replay/points_candidate.json) · [All held-out placements](dual_final/heldouts.csv) · [Verified binding and final checks](dual_final/final_check.json) · [Input and replay manifest](/home/user/桌面/jetson_nano/8/continuity_20261122/geometry_final_dual/replay/manifest.csv)

## Independent −5 repeat on the static edgeknot snapshot

The operator-confirmed new placement has 150 complete captured and decoded frames, raw video SHA-256 `d189f878898f8ba794d846ff81f77b2f85f15dd6d65c687e56ee1fbe48d46494`. Both modes produced **118/150** (78.67%), 25 below the per-placement minimum. Points MAE / max error were 0.09981 / 0.15224 cm, raw 0.15371 / 0.20023 cm. The longest missing run was 12 frames. All 150 frames passed rail verification. The 32 missing results comprise 17 single-candidate identity rejections, 14 tracker-acquisition frames and one two-candidate ambiguity. On every one of those 17 identity rejections the diameter and height checks passed and the measured appearance change exceeded the configured tolerance. An uninstrumented-coordinate comparison matched the original repeat replay's axial values on all 150 frames. The evidence supports keeping the static geometry change and declining validation on this recording. [Per-frame chronology](final_edgeknot/independent_m5/production.csv) · [Identity check inputs](final_edgeknot/repeat_identity.csv)

Using the fresh repeat as independent verification and excluding the development −5 clip and historical −5 repair target from the original exporter's verification sessions, both modes fail the external per-placement gate. Both final files were rebuilt by the original exporter in fit-only mode, verified by the loader against the checkout's live binding, and return no centimeters. [Points candidate](final_edgeknot/points_candidate.json) · [Raw candidate](final_edgeknot/raw_candidate.json) · [All independent and regression checks](final_edgeknot/heldouts.csv)

## Frozen edgeknot result

The final production refit uses the photograph-derived single corrected knot at x=657.760, y=649.047 px, the checkout's glint disambiguation and the actual current-frame tracking bounds. It ran on a file-identical snapshot after the shared checkout changed during an earlier run. The SHA-256 of the points configuration is `f63712d04dcdb38becfb75f0be17d813e471a5eb0eb1d20dbc652d77065eb7f2`. The points binding is `4cbfeb0f99dfecc1b19ccdd0f12615fc5a9e65774093705b0e33ce3ef660196c`. Independent replay through the uninstrumented checkout's points entry point agreed on all 150 frame statuses and axial coordinates of the key −5 clip. [Production parity](frozen_edgeknot/direct_points_parity.json) · [Input hashes](frozen_edgeknot/provenance.json) · [Per-clip hashes and roles](frozen_edgeknot/manifest.csv)

Each of the three fitting clips provided 146/150 measured samples. The original build_model exporter estimated points cm = 0.015918020520255753 × s_px − 0.1539610740024757, over [−363.54336, +390.11149] px. Two historical static rounds provided 1164/1200, the largest points error was 0.20454 cm, and the separate O placement gave 146/150 with maximum absolute error 0.13808 cm. The frozen O-tilt clip still gives 804/895 measured coordinates, signed mean −0.42624 cm, median −0.43042 cm, and extrema −0.51540 and −0.27728 cm. The static calibration has no pose correction. [Fitted points document with its three evidence sessions](frozen_edgeknot/points_candidate.json) · [Frozen tilt timeline](frozen_edgeknot/old_fixed_O_tilt/points.csv)

| Placement | Points valid / 150 | Points max error (cm) | Raw max error (cm) |
|---|---:|---:|---:|
| Historical −5, round 1 | 145 | 0.098 | 0.197 |
| Historical −3, round 1 | 146 | 0.112 | 0.242 |
| Historical +3, round 1 | 146 | 0.022 | 0.192 |
| Historical +5, round 1 | 146 | 0.113 | 0.146 |
| Historical −5, round 2 | 143 | 0.147 | 0.078 |
| Historical −3, round 2 | 146 | 0.205 | 0.338 |
| Historical +3, round 2 | 146 | 0.050 | 0.235 |
| Historical +5, round 2 | 146 | 0.175 | 0.122 |
| Historical reseated O | 146 | 0.138 | 0.101 |
| New −5, development | 143 | 0.174 | 0.155 |
| New +5 | 146 | 0.282 | 0.387 |
| New O | 146 | 0.072 | 0.081 |

In the corrected −5 second placement, the new profile moves the first three frames above the existing gray edge threshold, the glint check removes the stop competitors, and the tracker completes its normal four acquisition frames. Three edge rejects remain at frames 22–24. Points reaches the 143/150 gate. Raw fails the 0.3 cm bound at historical −3 and new +5. The points fit-only file has `ready=true`, `validated=false` at the intermediate stage before the separate new −5 repeat. Its final outcome is recorded above. The runtime loader returns no coordinate for this file. The glint-development clip stays outside the independent gate; the original historical −5 second placement remains a geometry regression. [All errors, coverage and missing spans](frozen_edgeknot/heldouts.csv) · [Raw decision](frozen_edgeknot/raw_decision.json) · [Points decision](frozen_edgeknot/points_decision.json)

## What the fixed-ball recording measures

On the original 1e30933 left-print configuration, the signed points result averaged −0.43323 cm relative to nominal O. Of 895 frames, 786 produced an axial coordinate and 109 did not. The maximum absolute deviation was 0.51421 cm. The full observed range was −0.51421 to −0.28467 cm (span 0.22954 cm). The central 90% spanned 0.03977 cm, with a standard deviation of 0.01625 cm. Medians of consecutive 60-frame windows spanned 0.00486 cm. The last window contains 55 frames. These windows describe the recorded chronology. The recording has no synchronized mechanical-angle labels for assigning dwell angles to frames. [Excursion](baseline_pose/excursion.json) · [Time windows](baseline_pose/windows.csv)

The −0.43511 cm tilt median differs from the separately reseated O median (−0.11103 cm) by −0.32408 cm. The original O fitting placement also maps to −0.11467 cm because the equal-weight three-position line has a nonzero residual at its middle anchor. At frame 75, a six-row image measurement of the printed 12 cm tick gives ball-center minus tick displacements of +1.62 px at fit O, +2.64 px at reseated O, and −17.79 px in the tilt clip. The tilt clip shows a different projected placement before applying calibration. The two extreme tilt frames, 153 and 383, place the estimated center at visibly different positions inside the ball outline. A center at a different height from the ruler and a different viewing pose can change its projection. These images do not resolve the contributions of physical placement, parallax and individual circle fits. [Matched crops with center crosses and tick marks](baseline_pose/zero_tick.jpg) · [Image measurements](baseline_pose/image_checks.csv)

Among valid tilt coordinates, the median corrected in-plane ruler rotation is −0.5462° (range −0.6766° to −0.4093°). The median projected raw rail axis angle is +1.2542° (range +1.1373° to +1.3674°). The reseated O medians are −0.1968° and +1.4676° respectively. These quantities describe orientation in the camera image. The operator reported a fixed ball and stop during the manual +0.5°, 0°, −0.5°, 0° command sequence. There is no same-clip measurement of physical pitch at the camera timestamps. No tilt-only coefficient was fitted. [Tilt coordinates](left_print/fixed_O_tilt.csv) · [Reseated O](left_print/verify_zero.csv)

In the earlier mechanical survey, command 0° gave a right-minus-left height of −8 mm over a horizontal 260 mm, or −1.762°. The positive command gave −9 mm (−1.983°), with a ball reseat during that experiment. Negative command and both zero returns all read −8 mm. One 1 mm scale division corresponds to about 0.22°. In the contiguous fixed-ball zero-return → negative-command transition the points median changed +0.017 cm, and the difference between return medians was −0.029 cm. These readings come from separate motion captures and camera calibration. [Measured mechanical plateaus](../../board_stage1_20261121/servo_angle.csv) · [Placement caveats](../../board_stage1_20261121/README.md)

## The 137/150 placement

At −5 cm (paper mark 7 cm) the complete second clip originally lost 13 outputs. The paper matcher confirmed its transform in all six frames where the upper-edge geometry failed. The existing gate requires a positive grayscale Sobel response above 35 in at least 55% of registered columns within ±5 px. The measured column counts are shown below. Detection depends on a valid rail. [All 150 frames](minus5_frames.csv) · [First-round control](minus5_round1.csv)

| Frame | Supported / domain | Blue supported in same band |
|---:|---:|---:|
| 0 | 451/859 | 709 |
| 1 | 453/859 | 716 |
| 2 | 463/859 | 718 |
| 22 | 192/861 | 713 |
| 23 | 168/861 | 715 |
| 24 | 237/861 | 737 |

Frames 3–6 each contain one real candidate during the required five consecutive acquisition frames. Frame 7 produces the first coordinate. The tracker resumes on frame 25. Frames 30, 67 and 146 each have two separated proposals, at x≈363–368 px on the ball and x≈317–321 px on the adjacent stop. Single-candidate coordinate validation correctly withholds the result in that case. The first-round control had 841–855 supported gray columns out of 857 across the entire recording. Independent footage shows severe local degradation of gray edge evidence in the second placement. The experiment does not identify a unique optical cause of the contrast change.

The isolated blue-channel helper repeated the paper identity check on the same gray frame, ran the normal positive edge test on the blue plane, and required near-profile support in each third. All six would pass aggregate blue support, but only 1.4–3.8% of the central third supported a positive edge within ±2 px of the old profile. All six remained invalid and the tracker produced 137/150. No recovery was accepted, so the helper's conditional negative-control branches did not run. The evidence favors correcting the reference profile from the photo before considering an alternative edge channel. [Isolated helper](blue_edge_trial.py) · [Full trial](minus5_trial.csv)

## Requested board placements

1. At measured horizontal and measured positive/negative offsets bracketing the intended ±0.5° operating range, obtain timestamps and settled physical angles from an inclinometer or endpoint height readings with resolution finer than the intended displacement. If using the previous 260 mm baseline, 0.5° is only 2.27 mm of height difference. Record commands, approach direction and returns to zero independently.
2. Clamp the ball and stop at an independently surveyed center projection for O. Keep both fixed through 0, positive, 0, negative, 0 and repeat the sequence from the opposite direction. Repeat after removing and reseating the fixture. Register a side or orthogonal photo/fiducial locating the center and record its height relative to the scale.
3. Repeat the same fixed sweeps at −5 and +5 cm to distinguish a position-dependent error from a shared offset. Collect separate stationary verification placements at the operating pose, including an independent −5 repeat withheld from glint development. Retain full raw video, read clocks and original rejection counts.

## Reproduction and acceptance

`refit.py` uses the historical replay for comparisons with the original failure. `production_fit.py` invokes the current motion observer and instruments its normal entry point. It checks the actual search bounds on every call to the tracker and logs the original coordinate object's binding. Both coordinate modes use the same detection and tracker result. Configuration files retain a null calibration path during measurement. All reported centimeters are derived in the audit.

The three fitting videos are `fit_m6`, `fit_zero`, and `fit_p6` at paper marks 6, 12 and 18 cm. Each is a distinct full recording. The per-placement criteria are ≥143 of 150 and max absolute error ≤0.300 cm. The two mislabeled historical +3 clips are excluded; their independently reseated retry clips are evaluated. Today's first −5 was used to develop the glint rule. It is retained as a development check. The old second −5 is a regression target for geometry. New angle tests and release acceptance are owned by the operator.


Intermediate source and per-placement replay directories from the rejected single-reference version are retained under the local experiment directory `8/continuity_20261122/archived_intermediate_geometry/`.
