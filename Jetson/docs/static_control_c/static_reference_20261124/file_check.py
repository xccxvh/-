"""独立核对 main --video 的文件解码路径。输出时间采用文件回放的 30 fps 名义时钟。"""
import csv
import hashlib
import json
import os
import sys
import tempfile
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from contextlib import redirect_stdout
from pathlib import Path
from unittest.mock import patch

os.environ['OPENBLAS_NUM_THREADS'] = '1'
EVIDENCE = Path(__file__).resolve().parent
JET = EVIDENCE.parents[2]
sys.path.insert(0, str(JET))
import main
from static_control_reference import DigitUpperEdgeReference


def read(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check(job):
    tag, path, expected, cm = job
    assert sha(path) == expected
    chosen = []
    locate = DigitUpperEdgeReference.locate

    def trace(ref, gray):
        # 旁路记录生产实现实际发布的参考类型；候选、坐标或门控均由主入口决定。
        evidence = locate(ref, gray)
        chosen.append(evidence['registration'].get('reference_set') if evidence else '')
        return evidence

    config = JET/'config_static_control_ruler_c.json'
    cfg = json.loads(config.read_text())
    with tempfile.TemporaryDirectory() as temp:
        cfg['output'] = dict(recordings_dir=temp, logs_dir=temp)
        argv = ['main.py','--config',str(config),'--csv','--video',str(path),'--fps','30']
        with patch.object(sys,'argv',argv), patch.object(main,'load_config',return_value=cfg), \
             patch.object(DigitUpperEdgeReference,'locate',trace), open(os.devnull,'w') as output, redirect_stdout(output):
            main.main()
        log, = Path(temp).glob('track_*.csv')
        observed = read(log)
    assert len(observed) == len(chosen) == (150 if cm is not None else 60)
    samples = []
    for i,(r,ref) in enumerate(zip(observed,chosen)):
        assert int(r['frame'])==i and r['calibration_valid']=='1'
        value = float(r['rail_x_cm']) if r['cm_valid']=='1' else None
        samples.append(dict(run=tag,frame=i,reference_set=ref,status=r['cm_status'],x_px=r['measurement_x_px'],
                            rail_x_cm=value,error_cm=abs(value-cm) if cm is not None and value is not None else None))
    values = [r['rail_x_cm'] for r in samples if r['rail_x_cm'] is not None]
    errors = [r['error_cm'] for r in samples if r['error_cm'] is not None]
    return (dict(run=tag,known_cm=cm,frames=len(samples),valid_cm=len(values),digit_reference=chosen.count('digit'),
                 digit_cm=sum(r['reference_set']=='digit' and r['rail_x_cm'] is not None for r in samples),
                 mae_cm=sum(errors)/len(errors) if errors else None,max_abs_cm=max(errors) if errors else None,
                 passed=(len(errors)>=143 and max(errors)<=.3) if cm is not None else None,video_sha256=expected),samples)


def save(path, data):
    with path.open('x') as f:
        writer = csv.DictWriter(f,data[0].keys())
        writer.writeheader()
        writer.writerows(data)


if __name__=='__main__':
    root=Path(sys.argv[1])
    jobs=[]
    for r in read(EVIDENCE/'placements.csv'):
        if r['role']=='fit':
            continue
        group, name = r['run'].split('_',1)
        runs=root/('7/motion_20261121/runs' if group=='old' else '8/continuity_20261122/runs')
        jobs.append((r['run'],runs/name/'raw.mjpg',r['video_sha256'],float(r['cm'])))
    # 这段独立补光录像只检查数字参考到实测厘米的通路，无同步逐帧已知刻度。
    jobs.append(('lit_sample',root/'9/static_control_Oj24BU/reflashed/reflash_lit_sample.avi',
                 '874760fbdfa10482a11a0aaef19062ff878c8daa256cf98feddc9336a67440b4',None))
    with ProcessPoolExecutor(max_workers=3) as pool:
        records=list(pool.map(check,jobs))
    save(EVIDENCE/'file_placements.csv',[r for r,_ in records])
    save(EVIDENCE/'file_frames.csv',[row for _,samples in records for row in samples])
