"""Analyze timestamped experiment CSVs offline. One file is one independent run."""
import argparse
import csv
import json
import math
import re
from collections import Counter
from itertools import groupby
from pathlib import Path
from statistics import median

import numpy as np


GAP = .150
NUMERIC = ('frame', 'capture_s', 'x_cm', 'target_ticks', 'output_ticks',
           'velocity_cm_s', 'v_ref', 'a_cmd', 'last_capture_s', 'last_received_s')
REQUIRED = {'event', 'nano_s', 'frame', 'capture_s', 'x_cm', 'valid',
            'target_ticks', 'output_ticks'}


def number(value):
    if value in (None, ''):
        return None
    val = float(value)
    if not math.isfinite(val):
        raise ValueError('nonfinite numeric field')
    return val


def read(path):
    with path.open(newline='', encoding='utf-8-sig') as stream:
        reader = csv.DictReader(stream)
        missing = REQUIRED - set(reader.fieldnames or ())
        if missing:
            raise ValueError('missing CSV columns ' + ', '.join(sorted(missing)))
        rows = []
        for row in reader:
            try:
                for key in ('nano_s',) + NUMERIC:
                    row[key] = number(row.get(key))
                if row['nano_s'] is None or (rows and row['nano_s'] < rows[-1]['nano_s']):
                    raise ValueError('missing or decreasing nano_s')
                if row['event'] == 'tx' and row['target_ticks'] is None:
                    raise ValueError('missing transmitted target')
            except ValueError as exc:
                raise ValueError('line %d: %s' % (reader.line_num, exc)) from exc
            rows.append(row)
    if not rows or sum(r['event'] == 'armed' for r in rows) > 1:
        raise ValueError('expected one experiment per file')
    return rows


def stats(values):
    values = [v for v in values if v is not None]
    if not values:
        return None
    return dict(n=len(values), min=min(values), median=median(values), max=max(values))


def direction(value):
    return int(value > 0) - int(value < 0)


def vision(rows, max_age):
    # 每个新曝光最多用一次。原始位置的时标是曝光时间；接收或决策时间仅用于检查帧龄。
    runs, broken, ages = [], Counter(), []
    active = None
    last_t = last_frame = None
    for row in rows:
        t, frame = row['capture_s'], row['frame']
        age = None if t is None else row['nano_s'] - t
        reason = None
        if (row['valid'] or '').lower() not in ('1', 'true'):
            reason = row.get('reason') or 'invalid'
        elif t is None or frame is None or row['x_cm'] is None:
            reason = 'missing measurement'
        elif not 0 <= age <= max_age:
            reason = 'outside age window'
        elif (last_t is not None and t <= last_t) or (last_frame is not None and frame <= last_frame):
            reason = 'repeated/out-of-order measurement'
        if reason:
            broken[reason] += 1
            active = None
        else:
            if active and (t - last_t > GAP or frame != last_frame + 1):
                broken['gap or skipped frame'] += 1
                active = None
            if active is None:
                active = []
                runs.append(active)
            active.append(row)
            ages.append(age)
        if t is not None:
            last_t = t if last_t is None else max(t, last_t)
        if frame is not None:
            last_frame = frame if last_frame is None else max(frame, last_frame)
    return runs, broken, ages


def span(rows, clock):
    return rows[-1][clock] - rows[0][clock] if rows else 0.


def commands(tx, neutral, end):
    # 占空时间用指令零阶保持积分。仅在明确的 abort/stop_requested 时延长最后一次发送。
    pulses, dwell = [], Counter()
    for i, r in enumerate(tx):
        d = direction(r['target_ticks'] - neutral)
        start = r['nano_s']
        stop = tx[i + 1]['nano_s'] if i + 1 < len(tx) else end
        dwell[d] += stop - start
        if d and (i == 0 or direction(tx[i-1]['target_ticks'] - neutral) != d):
            pulses.append(dict(start_s=start, direction=d, targets=[]))
        if d:
            pulses[-1]['targets'].append(r['target_ticks'])
            pulses[-1]['end_s'] = stop
    intervals = [b['nano_s'] - a['nano_s'] for a, b in zip(tx, tx[1:])]
    return dict(count=len(tx), hz=(len(intervals)/sum(intervals) if sum(intervals) else None),
                interval_s=stats(intervals), neutral_count=sum(r['target_ticks'] == neutral for r in tx),
                neutral_s=dwell[0], nonneutral_s=dwell[-1]+dwell[1], pulses=pulses)


def groups(rows, neutral, settled):
    # 平台要求重复回读为相同的实际输出；期间若出现相反命令、失能或遥测空档，必须切断拟合。
    groups_ = []
    active = None
    previous_rx = previous_output = last_target = target_since = last_phase = None
    for r in rows:
        if r['event'] == 'tx':
            last_phase = r.get('phase') or None
            if r['target_ticks'] != last_target:
                last_target, target_since = r['target_ticks'], r['nano_s']
            if settled and active and last_target != active['key']:
                active = None
        if r['event'] != 'rx':
            continue
        output = r['output_ticks']
        reason = r.get('reason') or ''
        if output is None or re.search(r'\benabled=0\b|\bresult=(?!0\b)-?\d+', reason):
            active = previous_rx = previous_output = None
            continue
        if settled and (output != r['target_ticks'] or (last_target is not None and last_target != output)):
            previous_rx, previous_output = r['nano_s'], output
            active = None
            continue
        key = output if settled else direction(output-neutral)
        t = r['nano_s']
        if active is None or t - active['last_s'] > GAP or key != active['key']:
            # 前一回读和第一个变化回读仅界定采样转变窗口，无法精确给出机械动作时刻。
            prev = None
            if previous_rx is not None and t - previous_rx <= GAP:
                prior_key = previous_output if settled else direction(previous_output-neutral)
                if prior_key != key:
                    prev = previous_rx
            tx_start = target_since if settled and last_target == output else None
            active = dict(key=key, first_s=t, prior_rx_s=prev, count=0,
                          target_since_s=tx_start, phase_at_first_rx=last_phase,
                          first_rx_minus_tx_s=t-tx_start if tx_start is not None else None)
            groups_.append(active)
        active.update(last_s=t, count=active['count'] + 1)
        previous_rx, previous_output = t, output
    return groups_


def quad(points):
    # 设计矩阵中心化，输出端点速度、加速度及残差。标准误只描述独立同方差噪声下的拟合精度。
    t = np.array([p['capture_s'] for p in points])
    u = t - t.mean()
    x = np.array([p['x_cm'] for p in points])
    design = np.column_stack([np.ones(len(t)), u, .5*u*u])
    coeff = np.linalg.lstsq(design, x, rcond=None)[0]
    residual = x - design @ coeff
    rms = float(np.sqrt(np.mean(residual**2)))
    se = np.linalg.pinv(design)[2]
    return dict(n=len(t), first_s=t[0], span_s=t[-1]-t[0],
                v_start=coeff[1]+coeff[2]*u[0], v_end=coeff[1]+coeff[2]*u[-1],
                accel_cm_s2=coeff[2], accel_se=float(np.linalg.norm(se)*np.linalg.norm(residual)
                                                     / math.sqrt(len(t)-3)), rms_cm=rms)


def fit_crossings(fit, plateau, threshold):
    # 二次拟合的导数是线性函数；只报告样本覆盖范围内的阈值交叉，禁止外推启动/刹停时刻。
    # 误差来自轨迹拟合、相机时间与遥测采样。两倍标准误只作为曲率筛选，不是物理置信区间。
    accel = fit['accel_cm_s2']
    if abs(accel) <= 2 * fit['accel_se']:
        return []
    result = []
    for speed in (-threshold, threshold):
        elapsed = (speed - fit['v_start']) / accel
        if not 0 < elapsed < fit['span_s']:
            continue
        stamp = fit['first_s'] + elapsed
        prior = plateau['prior_rx_s']
        result.append(dict(kind='onset' if speed * accel > 0 else 'stop',
                           speed_cm_s=speed, capture_s=stamp,
                           from_first_rx_s=stamp-plateau['first_s'],
                           from_prior_rx_s=stamp-prior if prior is not None else None))
    return sorted(result, key=lambda item: item['capture_s'])


def plateau_fits(plateaus, runs, threshold):
    for plateau in plateaus:
        fits = []
        n = 0
        for run in runs:
            if run[-1]['capture_s'] < plateau['first_s'] or run[0]['capture_s'] > plateau['last_s']:
                continue
            points = [p for p in run if plateau['first_s'] <= p['capture_s'] <= plateau['last_s']]
            n += len(points)
            if len(points) >= 6 and span(points, 'capture_s') >= .25:
                fit = quad(points)
                fit['first_capture_after_rx_s'] = fit['first_s'] - plateau['first_s']
                fit['fitted_crossings'] = fit_crossings(fit, plateau, threshold)
                fits.append(fit)
        plateau.update(fresh_points=n, fits=fits)


def crossings(group, runs, threshold):
    # 用连续三帧识别速度阈值交叉：前一帧在旧状态，随后两帧在新状态且运动同向。
    # 暂停后的滑行保留原符号。曝光和回读各有采样区间，区间差保留负下界的时间不确定性。
    prior = group['prior_rx_s']
    if prior is None:
        return []
    events = {}
    for run in runs:
        if run[0]['capture_s'] > group['first_s'] or run[-1]['capture_s'] < group['first_s']:
            continue
        for a, b, c in zip(run, run[1:], run[2:]):
            if not (prior <= a['capture_s'] and group['first_s'] <= c['capture_s']
                    and c['capture_s'] <= group['last_s']):
                continue
            v = [p['velocity_cm_s'] for p in (a, b, c)]
            if any(x is None for x in v):
                continue
            old, new, confirm = [abs(x) >= threshold for x in v]
            if old == new or confirm != new:
                continue
            if new and direction(v[1]) != direction(v[2]):
                continue
            kind = 'onset' if new else 'stop'
            if kind in events:
                continue
            # 同一段实测数据的位移和时延均可追溯；禁止跨失效帧回填停车距离。
            anchor = next((p for p in run if prior <= p['capture_s'] <= a['capture_s']), a)
            events[kind] = dict(kind=kind, signed_speed_cm_s=v[1] if new else v[0],
                                speed_pair_cm_s=v[:2], frame_pair=[a['frame'], b['frame']],
                                x_cross_cm=[a['x_cm'], b['x_cm']],
                                signed_travel_from_anchor_cm=b['x_cm']-anchor['x_cm'],
                                anchor_capture_s=anchor['capture_s'],
                                latency_bounds_s=[a['capture_s']-group['first_s'], b['capture_s']-prior])
    return list(events.values())


def analyze(path, args):
    rows = read(path)
    armed = next((r for r in rows if r['event'] == 'armed'), None)
    neutral = args.neutral
    if neutral is None and armed and armed['target_ticks'] == armed['output_ticks']:
        neutral = armed['output_ticks']
    if neutral is None:
        raise ValueError('neutral unavailable; specify --neutral for this firmware')
    # 停机确认的回读单独报告，不给写命令虚增一个遥测周期的驻留时间。
    terminal = next((r for r in rows if r['event'] in ('abort', 'brake_end', 'no_motion', 'stop_requested', 'stop_confirmed')), None)
    start = armed['nano_s'] if armed else rows[0]['nano_s']
    active = [r for r in rows if start <= r['nano_s'] and
              (terminal is None or r['nano_s'] < terminal['nano_s'])]
    tx = [r for r in active if r['event'] == 'tx']
    end = tx[-1]['nano_s'] if tx else start
    if terminal and terminal['event'] != 'stop_confirmed':
        end = terminal['nano_s']
    cmd = commands(tx, neutral, end)
    vr = [r for r in active if r['event'] == 'vision']
    runs, breaks, ages = vision(vr, args.max_age)
    points = [r for run in runs for r in run]
    band = []
    for run in runs:
        # 启动预检的曝光可能在使能后才写日志，不能计入闭环到达带的保持时间。
        in_window = (r for r in run if r['capture_s'] >= start and (armed or tx))
        for inside, g in groupby(in_window, lambda r: abs(r['x_cm']-args.goal) <= .3):
            if inside:
                band.append(list(g))
    longest = max(band, key=lambda g: span(g, 'capture_s'), default=[])
    gaps = [b['capture_s'] - a['capture_s'] for a, b in zip(points, points[1:])]
    phases = groups(active, neutral, False)
    for g in phases:
        samples = [p for p in points if g['first_s'] <= p['capture_s'] <= g['last_s']]
        g['logged_v_cm_s'] = stats(p['velocity_cm_s'] for p in samples)
        g['crossings'] = crossings(g, runs, args.speed)
    plateaus = groups(active, neutral, True)
    plateau_fits(plateaus, runs, args.speed)
    for plateau in plateaus:
        plateau['crossings'] = crossings(plateau, runs, args.speed)

    def ages_on_tx(field):
        return stats(r['nano_s']-r[field] for r in tx if r[field] is not None)
    rx = [r for r in active if r['event'] == 'rx']
    return dict(file=str(path), neutral_ticks=neutral, tx=cmd, rx_count=len(rx),
                rx_output_ticks=stats(r['output_ticks'] for r in rx),
                rx_intervals_s=stats(b['nano_s']-a['nano_s'] for a,b in zip(rx,rx[1:])),
                rx_signed_periods=phases, settled_plateaus=plateaus,
                vision_rows=len(vr), fresh_count=len(points), reasons=dict(breaks),
                phase_counts=dict(Counter(r.get('phase') or '(blank)' for r in vr)),
                capture_to_decision_s=stats(ages), x_cm=stats(r['x_cm'] for r in points),
                logged_velocity_cm_s=stats(r['velocity_cm_s'] for r in points),
                tx_phase_counts=dict(Counter(r.get('phase') or '(blank)' for r in tx)),
                v_ref=stats(r['v_ref'] for r in tx), a_cmd=stats(r['a_cmd'] for r in tx),
                tx_last_capture_age_s=ages_on_tx('last_capture_s'),
                tx_last_received_age_s=ages_on_tx('last_received_s'),
                fresh_gap_max_s=max(gaps, default=None), gaps_over_150ms=sum(v > GAP for v in gaps),
                band_frames=sum(map(len,band)), longest_band_s=span(longest, 'capture_s'),
                longest_band_frames=[longest[0]['frame'],longest[-1]['frame']] if longest else [],
                terminal=terminal['event'] if terminal else None,
                terminal_reason=terminal.get('reason') if terminal else None,
                stop_confirmed_neutral=any(r['event']=='stop_confirmed' and r['output_ticks']==neutral for r in rows))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('files', nargs='+', type=Path, help='one experiment CSV per run')
    parser.add_argument('--neutral', type=int, help='override armed echo, required for legacy unarmed logs')
    parser.add_argument('--summary-only', action='store_true', help='print only pooled run metrics')
    parser.add_argument('--goal', type=float, default=0., help='center of measured ±0.3 cm band')
    parser.add_argument('--max-age', type=float, default=.1, help='maximum exposure-to-log age (s)')
    parser.add_argument('--speed', type=float, default=.5, help='logged rolling-speed threshold (cm/s)')
    args = parser.parse_args()
    if not all(math.isfinite(x) for x in (args.goal,args.max_age,args.speed)) or min(args.max_age,args.speed) <= 0:
        parser.error('supply finite parameters and positive age and speed')
    results = []
    for path in args.files:
        try:
            results.append(analyze(path, args))
        except (ValueError, OSError, csv.Error) as exc:
            parser.exit(2, '%s: %s\n' % (path, exc))
    # 合并独立试验时只累加同一轮内的间隔和驻留；最长到达带绝不跨文件拼接。
    tx = [r['tx'] for r in results]
    intervals = sum(max(0,t['count']-1) for t in tx)
    duration = sum((t['count']-1)/t['hz'] for t in tx if t['hz'])
    summary = dict(runs=len(results), tx_count=sum(t['count'] for t in tx),
                   tx_hz=intervals/duration if duration else None,
                   neutral_count=sum(t['neutral_count'] for t in tx),
                   neutral_s=sum(t['neutral_s'] for t in tx),
                   pulses=sum(len(t['pulses']) for t in tx),
                   longest_band_s=max(r['longest_band_s'] for r in results))
    report = dict(goal_cm=args.goal, max_age_s=args.max_age,
                  speed_threshold_cm_s=args.speed, summary=summary)
    if not args.summary_only:
        report['runs'] = results
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
