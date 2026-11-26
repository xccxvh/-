# Measured velocity-return baseline

In run **174903**, the controller sent eight below-neutral pulses. The ball stayed 1.935–2.060 cm below the target throughout the fresh measurements. The measured hold within ±0.3 cm was **0 frames, 0 s**. [Experiment CSV](../adaptive_live_20261126/lit10/experiment_host_adaptive_20260324_174903.csv)

| Quantity | Recomputed result |
|---|---|
| TX cadence | 106 writes, 105 intervals, 16.966 Hz. Median interval 53.1 ms, maximum 89.8 ms. |
| Commanded neutral | 77 writes at 1324 ticks. First TX to abort spans 6.211 s, including **4.172 s neutral (67.2%)** and 2.039 s below neutral. Restricting the window to first through last TX gives 4.149 s neutral. |
| Pulse widths, in order | 553, 183, 361, 205, 198, 87, 90 and 363 ms. The minimum command and minimum RX output were both 1266 ticks. |
| Actual software output | 62 RX samples, 40 neutral. The first-to-last RX spans of the nine neutral periods sum to 3.101 s. Transitions between successive readbacks have ~100 ms sampling uncertainty. |
| Exposure coverage | 125/138 fresh positions. Seven pose failures, five stale drops and one aged decision. Median exposure-to-log age 88.3 ms, maximum 99.3 ms. One fresh-to-fresh gap exceeded 150 ms (200.0 ms). |
| Stop | The controller aborted for lack of reliable fresh motion for 220 ms. The final readback confirmed neutral. |

**Directional dynamics.** At a 0.5 cm/s threshold, 174903 has no observed onset or stop crossing. Four neutral exposure segments support quadratic fits. No constant non-neutral RX plateau supports one. In 174234, a below-neutral period includes a positive-speed onset (+0.586 cm/s, timing bracket −59 to +108 ms relative to the sampled RX transition). A neutral period includes a negative-speed stop crossing (−0.517 cm/s into the slow band, +90 to +225 ms). The first bracket spans zero. Filter delay, host RX logging latency and changing ticks within a signed period limit causal interpretation. [174234 CSV](../adaptive_live_20261126/experiment_host_adaptive_20260324_174234.csv)

**Engineering defaults.** The history supports negative PWM-to-position polarity for this mounting. In 174123 a positive excursion reached 1372 ticks and the probe ended at −1.420 cm/s. The 16 archived adaptive runs provide no qualifying below-neutral plateau fit. In 162918 the 1368-tick plateau fit gives +0.119 cm/s² (nominal SE 0.067) over 14 fresh exposures in 0.504 s, with fitted endpoint speeds −0.032 and +0.028 cm/s. These data do not identify either rolling gain, breakaway torque, braking acceleration or mechanical delay. The controller defaults of 0.55 cm/s²/tick in both directions, 6 cm/s² braking, 0.12 s delay and 3.2/s velocity feedback are engineering priors. There is no empirical basis here for changing them or claiming convergence. [174123 CSV](../adaptive_live_20261126/experiment_host_adaptive_20260324_174123.csv) · [162918 CSV](../adaptive_return_20261126/experiment_host_adaptive_20260324_162918.csv) · [Controller](../../velocity_return.py)

## Reproduce or compare runs

From the repository root with Python 3 and NumPy, pass one experiment CSV per run. The first command prints the full baseline. The second pools all 16 historical runs. Omit `--summary-only` for per-run details.

```sh
a=Jetson/docs/velocity_return_20261126/analyze.py
d=Jetson/docs/adaptive_live_20261126
python3 -B "$a" "$d"/lit10/experiment*174903.csv
python3 -B "$a" --summary-only --neutral 1324 "$d"/experiment*.csv \
    "$d"/lit10/experiment*.csv Jetson/docs/adaptive_return_20261126/experiment*.csv
```

The analyzer uses Nano monotonic seconds. It infers neutral from each armed echo unless overridden. Dwell ends at the logged stop request, abort or identification endpoint when present, otherwise at the final TX. Band time counts only consecutive valid raw exposures within 100 ms of logging and breaks at invalid rows, skipped frames or gaps over 150 ms. A single in-band frame has zero duration. The pooled rate divides within-run TX intervals by their summed duration.

`rx_signed_periods` groups actual output by side of neutral, including ramps. `settled_plateaus` requires identical echoed target and output. Logged onset/stop crossings require two confirming velocities after the threshold crossing. Independent fits use raw exposure positions strictly within the sampled plateau, at least six consecutive points spanning 250 ms. Each reports signed endpoint velocities, acceleration, residual RMS and nominal acceleration SE. Fitted threshold crossings stay inside the observed fit interval and require curvature above two nominal SEs. The RX timing brackets use host log times. Neither camera/telemetry clock lag nor physical rod movement was independently measured. Empty fits or crossings indicate insufficient evidence. [Analyzer](analyze.py)
