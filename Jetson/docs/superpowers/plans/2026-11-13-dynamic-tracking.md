# Dynamic tracking and drawing implementation plan

Goal: reject clipped bright background candidates, associate by elapsed time and proximity, reduce overlay overhead. User authorized software adjustments; illumination deferred. Existing Python3.6/OpenCV stack, no firmware, UART writes or calibration model changes.

Evidence: rolling frame160 geometric envelope baseline offset21px; false peaks plateau at the upper search-band boundary. Geometry lacks the truncation rejection already used by color. Tracker velocity uses pixels/frame, ignores supplied timestamp and chooses highest confidence within154px. Panel blends a full1280x720 copy for a small box.

- [x] Add failing tests for clipped bright regions vs visible ball, time-dependent association, stale/nonmonotonic timestamps, acquisition ambiguity and nearby distractors.
- [x] Reject geometry when its envelope is clipped by the local upper search boundary; preserve accepted geometry coordinates and calibration version.
- [x] Track in pixels/second, gate using actual dt, expire on long gap, prioritize spatial continuity, reject ambiguous acquisition. Feed monotonic read times live, video times offline; keep wall time in exported timestamp.
- [x] Restrict panel blending to panel rectangle, verify pixel equivalence and input immutability.
- [ ] Run all unit tests locally and Nano; compare original and modified same rolling/static/no-ball videos. DETECTED counts are not correctness labels. Preserve static calibration hash.
- [ ] Stage in new Nano directory with backup and review; retain original directories. Record limitations and update context.

Review focus: stale frame cannot reacquire instantly; dropped frames must not change inferred velocity; abrupt reversal must permit safe reacquisition; multiple bright peaks must not count as a ball; raw video time is average-fps approximation and does not enable UART validity. Dynamic cm remains blocked until reference points are validated.

Local verification:47 tests passed, static replay preserved, panel pixels identical. Reviewer findings on acquisition velocity and dt-dependent filtering fixed with regression tests. Nano remains unreachable; do not mark hardware checks complete.
