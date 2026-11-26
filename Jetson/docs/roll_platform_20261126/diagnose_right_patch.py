"""Development on the first scan only. Test a frozen right patch without any on-disk config changes."""
from analyze_unloaded_identity import (
    FIELD, JETSON, RUNS, FastUpperEdgeReference, cv2, json, np, plateaus, read_log,
    sys, templates, texture_peak,
)

# All candidates contain the existing digit subbox and use the original reference photo.
BROAD = [(878,642,132,28), (875,645,130,25), (883,644,121,26), (895,644,86,26),
         (910,648,90,20), (902,646,90,23), (888,646,84,24), (907,648,63,21),
         (896,649,83,18), (886,646,104,23), (914,652,46,16), (897,645,111,22),
         (904,647,113,21)]
NARROW = [(x,y,right-x,bottom-y) for x in (911,915,918,919)
          for y in (650,653,655,656) for right in (955,960) for bottom in (667,671)]


def frames(run_id, bank):
    run, folder = RUNS[run_id]
    periods = list(plateaus(read_log('experiment',run)))
    result = []
    for path in sorted((FIELD/'recordings'/('ruler_'+folder)).glob('*.jpg')):
        gray = cv2.imread(str(path),0)
        pos = []
        for center,tile in bank:
            p, _, _, _ = texture_peak(gray,tile,center,140)
            half = np.array(tile.shape[::-1])/2
            pos.append(center + p - np.rint(center-half) - half)
        group = next((i for i,_,lo,hi,_ in periods if lo<=float(path.stem)<hi), -1)
        result.append((path.stem, group, gray, np.array(pos)))
    return result


def bank_for(cfg, box):
    return FastUpperEdgeReference(dict(cfg,boxes=cfg['boxes'][:2]+[list(box)]))


def evaluate(cfg, box, data):
    rd = bank_for(cfg,box)
    source = np.array([[x+w/2,y+h/2] for x,y,w,h in cfg['boxes']])
    records = []
    for stem, group, gray, pos in data:
        bank = rd.tracks['primary']
        peaks, _, _ = bank._raw_peaks(gray)
        # Nearby candidate rectangle coordinates differ slightly from the original primary center.
        shift = pos[2]-source[2]
        raw_error = np.linalg.norm(np.array(peaks[2])-bank.points[2]-shift)
        evidence = rd.locate(gray)
        pose_error = None
        if evidence is not None:
            est = source.dot(np.linalg.inv(evidence['reference_inverse']).T)+evidence['reference_translation']
            pose_error = float(np.linalg.norm(est-pos,axis=1).max())
        records.append((stem, group, pose_error, raw_error))
    return records


def counts(base, new, groups):
    selected = [(a,b) for a,b in zip(base,new) if a[1] in groups]
    return dict(n=len(selected),old=sum(a[2] is not None for a,b in selected),
                new=sum(b[2] is not None for a,b in selected),
                gain=sum(a[2] is None and b[2] is not None for a,b in selected),
                loss=sum(a[2] is not None and b[2] is None for a,b in selected),
                raw_wrong=sum(b[3]>40 for a,b in selected),
                pose_over5=sum(b[2] is not None and b[2]>5 for a,b in selected),
                max_pose=round(max((b[2] for a,b in selected if b[2] is not None),default=0),2))


def negatives(cfg, boxes, data):
    # Each input independently cold-started. Preserve real alternatives elsewhere in the search.
    output = {}
    for kind in ('duplicate','shift_only','blank'):
        for box in boxes:
            rd = bank_for(cfg,box)
            vals = []
            for _, group, frame, pos in data:
                if group not in (0,2,4,7,10):
                    continue
                x,y = np.rint(pos[2]).astype(int)
                # Both true and displaced centers remain in the configured raw window. The wide
                # original right box is fully covered, and the narrow subbox moves with it.
                src = frame[y-24:y+25,x-41:x+42].copy()
                gray = frame.copy()
                if kind in ('blank','shift_only'):
                    gray[y-24:y+25,x-41:x+42] = int(np.median(src))
                if kind != 'blank':
                    gray[y-24:y+25,x+45:x+128] = src
                rd = bank_for(cfg,box)
                vals.append(rd.locate(gray) is not None)
            output[(kind,tuple(box))] = vals
    for kind in ('duplicate','shift_only','blank'):
        a,b = [np.array(output[(kind,tuple(box))]) for box in boxes]
        print('negative',kind,len(a),int(a.sum()),int(b.sum()),'newly accepted',int((~a & b).sum()))


def develop(cfg, data):
    original = cfg['boxes'][2]
    baseline = evaluate(cfg,original,data)
    ranks = []
    for box in BROAD+NARROW:
        new = evaluate(cfg,box,data)
        total = counts(baseline,new,range(-1,11))
        neg = counts(baseline,new,(2,3,4))
        ranks.append((total['pose_over5'], total['loss']-total['gain'], neg['raw_wrong'],
                      -neg['new'],tuple(box),total,neg))
    for row in sorted(ranks)[:5]:
        print(row)


if __name__=='__main__':
    cv2.setNumThreads(1)
    cfg = json.loads((JETSON/'config_static_fast_c.json').read_text())['rail']['upper_reference']
    bank = templates(cfg)
    if sys.argv[1:] == ['--develop']:
        develop(cfg,frames(0,bank))
    else:
        if len(sys.argv)!=5:
            sys.exit('Supply frozen x y w h, or --develop (first scan only)')
        box = tuple(map(int,sys.argv[1:]))
        for j in (0,1):
            data = frames(j,bank)
            orig = cfg['boxes'][2]
            base = evaluate(cfg,orig,data)
            new = evaluate(cfg,box,data)
            print(RUNS[j][0])
            for name,groups in [('all',range(-1,11)),('deep',(2,3,4)),('positive',(6,7,8,9)),
                                ('neutral',(-1,0,5,10)),('1282',(1,))]:
                print(name,counts(base,base,groups),counts(base,new,groups))
            for i in range(-1,11):
                print(i,counts(base,new,[i]))
            negatives(cfg,(orig,box),data)
