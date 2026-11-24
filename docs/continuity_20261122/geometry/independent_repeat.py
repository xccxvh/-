"""在冻结模型上加入新的独立负五复摆；原视频和绑定全部重新检查。"""
import argparse
from pathlib import Path
from types import SimpleNamespace

from production_fit import MODES, TODAY, YESTERDAY, old, replay, save_new


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('baseline',type=Path)
    parser.add_argument('repeat',help='现场确认球重新摆放的全新目录，禁止使用调参段')
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    assert Path(args.repeat).name==args.repeat and args.repeat!='verify_m5'
    src=args.baseline.resolve()
    dst=args.output.resolve()
    v=old.json.loads((src/'provenance.json').read_text())
    assert all(old.sha256(f)==h for f,h in v['files'].items())
    for f in v['files']:
        recorded=Path(f)
        relative=recorded.relative_to(Path(v['raw_config']).parent)
        assert old.sha256(old.JET/relative)==old.sha256(recorded)
    configs={m:Path(v[m+'_config']) for m in MODES}
    entries=old.read_csv(src/'manifest.csv')
    repeat=TODAY/args.repeat
    digest=old.sha256(repeat/'raw.mjpg')
    assert digest not in {e['video_sha256'] for e in entries}
    count=len(old.read_csv(repeat/'frames.csv'))
    assert count==150
    dst.mkdir()
    pair=replay(repeat,dst/'independent_m5',configs,count,-5)
    reports=[]
    for mode in MODES:
        fits, verifications, checks = [],[],[]
        for e in entries:
            role,run=e['role'],e['run']
            if role not in ('fit','verify','repeat_zero'):
                continue
            folder=(TODAY if run.startswith('new_') else YESTERDAY)/run[4:]
            assert old.sha256(folder/'raw.mjpg')==e['video_sha256']
            d=src/run
            binding=old.json.loads((d/(mode+'.binding.json')).read_text())
            params=SimpleNamespace(video=folder/'raw.mjpg',csv=d/(mode+'.csv'),cm=int(e['cm']),
                                   known_position=True,start_frame=0,end_frame=int(e['frames'])-1)
            session=old.collect(params,binding,d/(mode+'.binding.json'))
            assert binding==pair[mode]['binding']
            if role=='fit':
                fits.append(session)
            else:
                checks.append((run,session))
                # 旧第二轮负五参与过修复分析，只作逐段回归。新负五替代它做独立验证。
                if role=='verify' and run!='old_verify_m5_02':
                    verifications.append(session)
        verifications.append(pair[mode])
        checks.append(('new_'+args.repeat,pair[mode]))
        model=old.build_model(pair[mode]['binding'],fits,verifications)
        passed=True
        for run,s in checks:
            values=[old.map_position(model['model'],r['rail_s_px']) if r['pixel_valid'] and r['reference_valid']
                    and r['predicted'] is False and r['rail_s_px'] is not None else None for r in s['rows']]
            met=old.metrics(s['cm'],values)
            reports.append(dict(mode=mode,run=run,cm=s['cm'],**met))
            passed=passed and met['passed']
        accepted=bool(passed and model['validated'])
        candidate=model if accepted else old.build_model(pair[mode]['binding'],fits)
        target=dst/(mode+'_candidate.json')
        save_new(target,candidate)
        loader=old.RailCalibration(str(target),pair[mode]['binding'])
        assert loader.ready and loader.validated==accepted and (loader.position(0) is not None)==accepted
        old.json_out(dst/(mode+'_decision.json'),dict(status=loader.status,static_pass=accepted,
                     repeat_video_sha256=digest,binding=candidate['rail_binding_id'],model_sha256=old.sha256(target)))
    old.write_rows(dst/'heldouts.csv',reports)


if __name__=='__main__':
    main()
