"""只验证候选兜底。比较原配置、三模板复验、再加独立数字位置核验的结果。"""
from diagnose import *
from diagnose import peaks, fit, candidate, records, write_table, center, mapped, new_alt

def witnesses_accept(count, error):
    return bool(error is not None and count>=3 and error<=2.5)

def main():
    rows=records(OUT/'frames_evidence.csv')
    ctrl=records(OUT/'controls.csv')
    selected=[]
    for i,row in rows.items():
        rec=dict(frame=i,onboard=row['onboard'],board_reason=row['board_reason'],full_reason=row['all_replay'])
        for label,value in [('left',ctrl[i]['witness_max_px']),('three',row['alt_witness_max_px'])]:
            px=float(value) if value else None
            rec[label+'_added']=witnesses_accept(int(row['witness_count']),px)
        selected.append(rec)
    write_table('guarded_decisions.csv',selected)

    # 使用之前的两段原始录像独立核查；只对冻结失败帧额外取证，也记录没有通过的帧。
    for scene in ('baseline','live_pixel'):
        parent=ROOT.parent/'moving_check'/scene
        edge=UpperEdgeReference(cfg)
        alt=new_alt()
        extra=[];original=0
        for i,ts,im,g,_ in decoded(replay(parent)):
            if edge.locate(g) is not None:
                original+=1
                continue
            raw=[peaks(g,b) for b in boxes]
            proposal,_=fit(boxes,raw)
            alt.matcher.transform=None
            result=candidate(g,alt,raw,proposal)
            witnesses=[(b,peaks(g,b)) for b in CHECK]
            accepted=[(b,v) for b,v in witnesses if v['score']>=.85 and v['margin']>=.05]
            error=None
            if result is not None and accepted:
                pts=np.array([center(b) for b,v in accepted])
                obs=pts+np.array([[v['dx'],v['dy']] for b,v in accepted])
                error=float(np.max(np.linalg.norm(mapped(result,pts)-obs,axis=1)))
            extra.append(dict(frame=i,reason=edge.reason,witnesses=len(accepted),
                              added=witnesses_accept(len(accepted),error)))
        print(scene,i+1,'frozen valid',original,'failures',extra)
        if extra:write_table(scene+'_fallback.csv',extra)
if __name__=='__main__':main()
