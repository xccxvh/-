# Jetson measurement implementation plan

Goal: improve existing measurement code and provide opt-in UART telemetry without changing firmware.
Spec: user task in this conversation; G3519 excluded by user.
Architecture: reuse detector, add fail-closed calibration and pose checks, bounded serial transport.
Tech stack: Python 3.6+, NumPy, OpenCV; stdlib termios for Linux UART.

Constraints: no invented calibration; no automatic motor commands; no claim of live accuracy from synthetic tests.
Ruling: directory is not a Git repository; verified full backup replaces Git isolation/commits.
Ruling: user requested execution; implement inline without another approval round.

Review focus: reverse coordinate direction, corrupt models, stale rail/frames, partial serial writes, replay prohibition.

- [x] Regression tests then fixes: calibration requires three monotonic finite points including -5/0/+5, rejects out-of-range values; weak candidates expire; failed rail fit invalidates immediately.
- [x] Calibration CLI: explicit known-position video frame interval; preserve frame/pose evidence; independent validation reports signed bias, MAE, max error, standard deviation and peak-to-peak.
- [x] UART: versioned ASCII with CRC16, session/sequence/capture time/age, invalid records contain no coordinate; monotonic freshness; bounded writes; default disabled; offline replay forbidden.
- [x] Integration: CSV validity reasons and pose checks; CLI/docs; full tests and replay existing recordings if available.
- [x] Review final diff and report field-only gaps.


## Execution record

Local suite: 29/29 passed. Nano Python 3.6.9 suite: 29/29 passed, 7.794 s; log test_results/tests_final.txt.
Final candidate files verified by SHA-256 before board tests. Original board 15 top-level files verified unchanged against initial backup.
Fresh independent reviewer: Epicurus. Findings reproduced before fixes: numeric-string models, unknown coordinate, smoothed pose gate, future/extreme timestamps, masked reasons, init resource cleanup. Fixed with regression tests.
Ruling: sensor capture age is not verifiable through current OpenCV backend. Keep live feedback invalid instead of treating read-completion time as exposure time; cost: automated feedback cannot yet be armed.
Ruling: use static pose gate until a track-bound reference origin/scale is measured; cost: intentional loss of centimeter output during tilt.
Ruling: do not fabricate calibration or suppress geometry-only detections based on the occluded rolling video; cost: false background lock remains a field-validation blocker.
No firmware changes, real UART writes, camera capture, or actuator operations performed.
The checked tasks concern software tools and offline checks, not completion of the user's physical calibration/closed-loop experiments.
