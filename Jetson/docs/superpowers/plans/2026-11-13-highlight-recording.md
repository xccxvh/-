# Split highlights and recording repair

User authorized repair of two demonstrated failures. Preserve old projects and calibration. Backup: /home/user/ball_vision_before_highlight_20261113_191744.

- [x] Reproduce real manual48 frame split at x516..530 and542..548. Add positive and negative synthetic cases before modifying detector.
- [x] Hysteresis recovery: low-threshold contiguous cap must contain multiple high-threshold fragments with bounded gaps, adequate coverage, rounded envelope and untruncated top. Retain primary coordinate when not recovered; label recovered method separately and prohibit silent old-calibration reuse for recovered methods. Never lower minimum width globally.
- [x] Recorder worker owns video_writer.py and tests/test_video_writer.py. Limit encoding/filter threads, avoid blocking measurement on queue congestion, expose recording failures and timestamp evidence. Main integration owned by primary agent.
- [x] Full local and Nano unit tests, old four static recordings and faulty rolling footage regression. Test live no-record vs record under same manual10ms gain48, preserve original camera controls.
- [x] Verify real frame duration metadata, saved outputs, source hashes; independent review before completing. Current pose remains incompatible with static calibration and UART remains disabled.

Boundaries: pixel recovery is not cm accuracy. Synthetic shape tests and existing bad video are regressions, not new independent whole-track validation. No G3519 or car firmware changes.

Completed:73/73 local and Nano tests; real held-ball live test26.7fps with262/267 detected. Review low-fps finding reproduced and fixed. Default live recording downsampled to640x360/15fps; source detection unchanged. Timing from timestamp sidecar, no CFR real-time claim. Dynamic cm and full-track validation remain outside this repair.
