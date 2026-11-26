"""Estimate rolling response from joined REMOTE and calibrated vision logs."""
import argparse
import csv
from itertools import groupby
import math
import sys

import numpy as np


def analyze(rows, threshold, out):
    if not math.isfinite(threshold) or threshold <= 0:
        raise ValueError('supply measured stationary acceleration noise floor (cm/s^2)')
    writer = csv.writer(out)
    writer.writerow(('plateau', 'target_ticks', 'direction', 'frames', 'span_s',
                     'accel_cm_s2', 'fit_rms_cm', 'movement_sign', 'status'))
    staged = []
    stage = None
    output = None
    for row in rows:
        if row['event'] == 'tx':
            stage = (int(row['plateau_index']), int(row['target_pwm_ticks']))
        elif row['event'] == 'remote':
            output = int(row['output_pwm_ticks']) if row['remote_enabled'] == '1' else None
        elif row['event'] in ('abort', 'stop_requested', 'stop_confirmed', 'stop_unconfirmed'):
            stage = None
        if stage and stage[0] >= 0:
            staged.append((stage, output, row))

    previous = None
    passed = []
    for (index, ticks), group in groupby(staged, lambda item: item[0]):
        direction = 0 if previous is None else int(np.sign(ticks-previous))
        previous = ticks
        samples = {}
        first = None
        for _, output, row in group:
            now = float(row['nano_timestamp'])
            if first is None:
                first = now
            if (output == ticks and row['cm_valid'] == '1'
                    and float(row['vision_read_ms']) >= first*1000):
                samples[float(row['vision_read_ms'])/1000.] = float(row['ball_x_cm'])
        times = np.array(sorted(samples))
        n = len(times)
        span = times[-1]-times[0] if n else 0.
        prefix = (index, ticks, direction, n, span)
        if n < 6 or span < .25 or np.max(np.diff(times)) > .15:
            writer.writerow(prefix + ('', '', '', 'insufficient_trajectory'))
            continue
        x = [samples[t] for t in times]
        t = times-times[0]
        fit = np.polyfit(t, x, 2)
        accel = 2*fit[0]
        rms = float(np.mean((np.polyval(fit, t)-x)**2)**.5)
        sign = int(np.sign(accel)) if abs(accel) > threshold else 0
        writer.writerow(prefix + (accel, rms, sign, 'fit'))
        passed.append((index, ticks, direction, sign))
    return passed


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('joined_csv')
    parser.add_argument('--accel-noise', required=True, type=float,
                        help='stationary apparent acceleration threshold in cm/s^2')
    args = parser.parse_args()
    with open(args.joined_csv, newline='') as stream:
        passed = analyze(csv.DictReader(stream), args.accel_noise, sys.stdout)
    for a,b in zip(passed, passed[1:]):
        if b[0] == a[0]+1 and a[2] == b[2] and a[3]*b[3] < 0:
            print('rolling balance bracket: %s..%s ticks direction=%s' %
                  (min(a[1],b[1]), max(a[1],b[1]), b[2]), file=sys.stderr)
