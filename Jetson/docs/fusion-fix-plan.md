# Fusion coordinate fix

User authorized fixing color-centroid jumps. Use systematic debugging and test-first verification.
Scope: geometric coordinate for hybrid measurements; nearest one-to-one color association within half geometric width and original global limit. Colors alone remain weak evidence. No temporal clipping, no changes to firmware/UART safety.
Tasks: verified snapshot; failing fusion tests; patch; full local and Nano suites; regenerate three-point calibration from original recorded windows using new measurement definition; replay old +2 as regression (not independent holdout); fresh camera capture only as follow-up without inventing ground truth; save provenance and report.
Validation focus: offset color, close matched color, moving geometric target, competing candidates, no-ball baseline, existing point intervals, Nano Python3.6.
Coordinate revision must invalidate old calibrations rather than silently reuse incompatible coordinates.

Complete: snapshot verified; regression RED→GREEN; 36 tests local and Nano passed; independent review no important issues; original three-point windows recalibrated on Nano; old +2 regression max0.03059cm; historical center/no-ball unchanged. Fresh capture blocked by changed bright background, reported explicitly. No firmware/UART changes.
