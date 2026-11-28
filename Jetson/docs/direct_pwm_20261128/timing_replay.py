"""Re-evaluate exposure admission at recorded delivery times. No actuation or learning."""
from bisect import bisect_right
import csv
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]))
from adaptive_return import exposure_output
from direct_return import DirectReturn
from velocity_return import AlphaBeta, FRESH_S, LOST_S


def replay(path):
    state = AlphaBeta()
    law = DirectReturn((1236, 1413), 1324, -1)
    experiment = path.with_name(path.name.replace('track_', 'experiment_', 1))
    with experiment.open() as stream:
        events = list(csv.DictReader(stream))
    echoes = [e for e in events if e['event'] == 'rx']
    times = [float(e['nano_s']) for e in echoes]
    samples = [(float(e['nano_s']), int(e['target_ticks']), int(e['output_ticks'])) for e in echoes]
    changes = []
    for event in events:
        if event['event'] in ('armed', 'tx'):
            pwm = int(event['target_ticks'])
            if not changes or changes[-1][1] != pwm:
                changes.append((float(event['nano_s']), pwm))
    stop = next(float(e['nano_s']) for e in events if e['event'] == 'stop_requested')
    rows = list(csv.DictReader(path.open()))
    worst = 0.
    admitted = lost = 0
    for row in rows:
        # uart_frame_time_ms is the full precision monotonic SOE in these logs.
        stamp = float(row['uart_frame_time_ms']) / 1000.
        age = float(row['uart_age_ms']) / 1000.
        if state.stamp is not None:
            gap = stamp + age - state.stamp
            worst = max(worst, gap)
            lost += int(gap > LOST_S)
        if (row['feedback_reason'] in ('ok', 'stale_only_drop')
                and row['cm_valid'] == row['timestamp_verified'] == '1' and 0 <= age <= FRESH_S):
            observed = float(row['rail_x_cm'])
            if state.stamp is not None and stamp - state.stamp > FRESH_S:
                law.reset_observation()
            if state.update(stamp, observed):
                admitted += 1
                now = stamp + age
                at_delivery = bisect_right(times, now)-1
                if at_delivery >= 0 and now < stop:
                    echo = exposure_output(samples, changes, stamp)
                    x, v = state.predict(now)
                    law.step(x, v, stamp, observed, int(echoes[at_delivery]['output_ticks']), echo)
        else:
            law.interrupt_hold()
    return (len(rows), sum(r['feedback_reason'] == 'ok' for r in rows), admitted, 1000*worst, lost, sum(map(len, law.experience['regions'].values())))


if __name__ == '__main__':
    logs = HERE.parent / 'roll_platform_20261126/field/from11_10ms/logs'
    print('run,frames,legacy_valid,new_admitted,max_age_before_delivery_ms,over_300ms,learned_records')
    for path in sorted(logs.glob('track_host_adaptive_*.csv')):
        print('%s,%d,%d,%d,%.2f,%d,%d' % ((path.stem.split('_')[-1],) + replay(path)))
