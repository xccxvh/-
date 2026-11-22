"""在相同实测处理帧号及纳秒时钟上比较固定三块模板组合。"""
import argparse
import csv
from collections import Counter
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('frozen',type=Path)
    parser.add_argument('runs',type=Path)
    parser.add_argument('variant',choices=['print','mixmid','mixright','digits','dual'])
    parser.add_argument('clips',nargs='+')
    args = parser.parse_args()
    sys.path.insert(0,str(args.frozen.resolve()/'Jetson'))
    from motion_observe import decoded, observe, replay

    for clip in args.clips:
        folder=args.runs/clip
        # 动态片以实板实际处理帧号和时刻驱动状态机，静态片没有测量日志，逐帧回放。
        log=folder/'measurements.csv'
        with log.open() if log.is_file() else open(folder/'frames.csv') as f:
            rows=list(csv.DictReader(f))
        times={int(r['frame']):float(r.get('timestamp', r.get('read_s'))) for r in rows}
        def select():
            seen=[]
            for i,ts,jpeg in replay(folder):
                if i in times:
                    assert abs(times[i]-ts)<1e-6
                    seen.append(i)
                    yield i,ts,jpeg
            assert seen==list(times)
        path=args.frozen/(args.variant+'_'+clip+'.csv')
        cfg=args.frozen/'Jetson'/('config_motion_'+args.variant+'_points_20261122.json')
        observe(cfg,decoded(select()),path,False)
        with path.open() as f:
            out=list(csv.DictReader(f))
        assert [int(row['frame']) for row in out]==list(times)
        print(args.variant,clip,len(out),sum(r['pixel_valid']=='True' for r in out),
              Counter(r['rail_reason'] for r in out),flush=True)


if __name__=='__main__':
    main()
