"""Read the two unloaded scans. Print tables, or one JSON object per JPG with --detail."""
import csv
import json
import sys
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
JETSON = HERE.parents[1]
sys.dont_write_bytecode = True
sys.path.insert(0, str(JETSON))
from ruler_reference import similarity_from_points
from static_fast_reference import FastUpperEdgeReference

FIELD = HERE / 'field'
RUNS = [('185851', '1774349931986'), ('190039', '1774350039814')]
BINDING = '8fcf1384625f6dfc57f90b883c161791d6b99079c56361774c2f74d9459d9318'


def read_log(kind, run):
    path = FIELD / 'logs' / f'{kind}_host_scan_20260324_{run}.csv'
    with path.open() as stream:
        return list(csv.DictReader(stream))


def soe(row):
    return float(row['uart_frame_time_ms']) / 1000


def plateaus(events):
    tx = {}
    first_tx = {}
    periods = {}
    for row in events:
        t = float(row['nano_timestamp'])
        if row['event'] == 'tx':
            i = int(row['plateau_index'])
            tx[row['command_seq']] = i, row['target_pwm_ticks']
            first_tx.setdefault(i, t)
        if row['event'] == 'remote' and row['remote_enabled'] == '1' and row['result'] == '0':
            i, tick = tx[row['command_seq']]
            if row['output_pwm_ticks'] == row['target_pwm_ticks'] == tick:
                periods.setdefault(i, (int(tick), t))
        if row['event'] == 'stop_requested':
            stop_tx = t
        if row['event'] == 'remote' and row['remote_enabled'] == '0':
            stop_rx = t
    assert sorted(periods) == list(range(11)) and stop_tx <= stop_rx
    for i, (tick, start) in periods.items():
        end = periods[i + 1][1] if i + 1 in periods else stop_rx
        yield i, tick, start, end, first_tx.get(i + 1, stop_tx)


def rate(rows):
    n = sum(r['rail_identity_valid'] == '1' for r in rows)
    return f'{n}/{len(rows)} ({100*n/len(rows):.0f}%)' if rows else '—'


def texture_peak(gray, template, center, radius):
    h, w = template.shape
    x, y = np.rint(center - [w/2, h/2]).astype(int)
    lo = np.maximum([0, 0], [x-radius, y-radius])
    hi = np.minimum(gray.shape[::-1], [x+w+radius, y+h+radius])
    scores = cv2.matchTemplate(gray[lo[1]:hi[1], lo[0]:hi[0]], template, cv2.TM_CCOEFF_NORMED)
    _, score, _, (u, v) = cv2.minMaxLoc(scores)
    scores[max(0,v-2):v+3, max(0,u-2):u+3] = -1
    left, top = lo + [u, v]
    return (np.array([left+w/2, top+h/2]), score, score-float(scores.max()),
            gray[top:top+h, left:left+w])


def sharpness(tile):
    return cv2.Laplacian(tile, cv2.CV_32F)[1:-1, 1:-1].var()


def templates(cfg):
    first = min((FIELD/'recordings'/('ruler_'+RUNS[0][1])).glob('*.jpg'))
    base = cv2.imread(str(first), cv2.IMREAD_GRAYSCALE)
    reg = FastUpperEdgeReference(cfg).locate(base)['registration']
    assert reg['reference_set'] == 'primary'
    centers = np.array(reg['current_points'])
    bank = []
    for center, (_, _, w, h) in zip(centers, cfg['boxes']):
        x, y = np.rint(center - [w/2, h/2]).astype(int)
        bank.append((center, base[y:y+h, x:x+w].copy()))
    return bank


def measure(gray, bank, matchers):
    shifts, ratios, scores, margins = [], [], [], []
    for center, tile in bank:
        p, s, m, current = texture_peak(gray, tile, center, 140)
        shifts.append(p - np.rint(center - np.array(tile.shape[::-1])/2) - np.array(tile.shape[::-1])/2)
        ratios.append(float(sharpness(current)/sharpness(tile)))
        scores.append(s)
        margins.append(m)
    estimate = np.array([v[0] for v in bank]) + shifts
    primary = matchers[0].tracks['primary']
    displacements = estimate - primary.points
    wide = matchers[1].tracks['primary']
    peaks, original_scores, original_margins = wide._raw_peaks(gray)
    _, fit = similarity_from_points(primary.points, peaks)
    # 同名印纹的小窗中心由独立现场纹理定位；仅作诊断，不能消去竞争峰用于身份确认。
    correct_dy, primary_local, primary_scores = [], [], []
    for name in ['primary', 'fallback', 'digit']:
        group = matchers[0].tracks[name]
        for j, (_, _, _, _, tile) in enumerate(group.patches):
            predicted = group.points[j] + displacements[j]
            local, score, _, _ = texture_peak(gray, tile, predicted, 5)
            correct_dy.append(float(local[1] - group.points[j,1]))
            if name == 'primary':
                primary_local.append(local)
                primary_scores.append(score)
    valid = [matcher.locate(gray) is not None for matcher in matchers]
    return dict(shift=np.asarray(shifts).tolist(), delta=displacements.tolist(), score=scores,
                margin=margins, sharp=ratios, true_peak_y=correct_dy, original_wide_scores=original_scores,
                original_wide_margins=original_margins, local_primary_scores=primary_scores,
                local_primary_offsets=(np.array(primary_local)-primary.points).tolist(),
                local_fit=similarity_from_points(primary.points, primary_local)[1],
                wide_error=(np.array(peaks)-estimate).tolist(),
                residual=fit['residual_px'], replay=valid)


def summarize(run, folder, cfg, bank, detail):
    track = read_log('track', run)
    assert all(r['timestamp_verified']=='1' and r['rail_binding_id']==BINDING for r in track)
    on_wire = {r['frame']: r for r in read_log('link', run) if r['event']=='tx' and r['frame']}
    assert all(int(on_wire[r['frame']]['capture_ms32'])==int(float(r['uart_frame_time_ms'])) for r in track)
    frames = {round(soe(r), 6): r for r in track}
    matchers = [FastUpperEdgeReference(dict(cfg, search_y=y)) for y in (96,120)]
    jpgs = {}
    for f in sorted((FIELD/'recordings'/('ruler_'+folder)).glob('*.jpg')):
        t = float(f.stem)
        row = frames[t]
        m = measure(cv2.imread(str(f), cv2.IMREAD_GRAYSCALE), bank, matchers)
        m.update(soe=t, frame=int(row['frame']), logged=row['rail_reason'])
        jpgs[t] = m
        if detail:
            print(json.dumps(m))
    if detail:
        return
    print(run, 'frames',len(track),'JPG',len(jpgs),'reasons',Counter(r['rail_reason'] for r in track))
    print('SOE to capture (ms), median/max',
          np.percentile([float(r['capture_monotonic_ms'])-1000*soe(r) for r in track],[50,100]))
    periods = list(plateaus(read_log('experiment', run)))
    print('区间外',rate([r for r in track if not periods[0][2]<=soe(r)<periods[-1][3]]))
    for i,tick,a,b,safe_end in periods:
        within = [r for r in track if a<=soe(r)<b]
        early = [r for r in within if soe(r)<a+.5]
        middle = [r for r in within if a+.5<=soe(r)<a+1]
        settled = [r for r in within if soe(r)>=a+1]
        pics = [m for t,m in jpgs.items() if a+1<=t<safe_end]
        dxdy = np.median([m['shift'] for m in pics],axis=0)[[0,2]]
        print('|', ' | '.join(map(str,[i,tick,f'{a:.3f}',rate(early),rate(middle),rate(settled),
              rate([r for r in settled if soe(r)<safe_end]),len(pics),dxdy.tolist()])), '|')
    for label, subset in [('全部', list(jpgs.values())),
                          ('负侧深段', [m for t,m in jpgs.items() if periods[2][2]<=t<periods[5][2]]),
                          ('正侧三档', [m for t,m in jpgs.items() if periods[6][2]<=t<periods[9][2]])]:
        print(label,'JPG数',len(subset),'96/120配对',Counter(tuple(m['replay']) for m in subset),
              '实拍最小分/峰差',min(min(m['score']) for m in subset),min(min(m['margin']) for m in subset),
              '绝对中心位移xy最小/最大',np.min([m['delta'] for m in subset],axis=(0,1)),
              np.max([m['delta'] for m in subset],axis=(0,1)),
              '三组原模板近邻峰dy极值',np.min([m['true_peak_y'] for m in subset]),np.max([m['true_peak_y'] for m in subset]),
              '右块宽域错位>40px',sum(abs(m['wide_error'][2][0])>40 for m in subset),
              '锐度比中位',np.median([m['sharp'] for m in subset]))
    print('在线与JPG原窗回放一致数',sum(m['replay'][0]==(m['logged']=='confirmed_upper_edge') for m in jpgs.values()))


if __name__ == '__main__':
    cv2.setNumThreads(1)
    config = json.loads((JETSON/'config_static_fast_c.json').read_text())['rail']['upper_reference']
    bank = templates(config)
    for run, folder in RUNS:
        summarize(run, folder, config, bank, '--detail' in sys.argv)
