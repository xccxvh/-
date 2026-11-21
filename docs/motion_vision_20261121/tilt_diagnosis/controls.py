"""外观对照与只替换左窗的消融试验。只向当前诊断目录写结果。"""
from diagnose import *
from diagnose import fit, mapped, candidate, peaks, geometry_check, write_table, records, new_alt, center

def subpixel(g,x,y,patch):
    h,w=patch.shape
    target=patch.astype(np.float32)
    best=-1.
    for dx in (-.5,0.,.5):
        for dy in (-.5,0.,.5):
            cur=cv2.getRectSubPix(g,(w,h),(x+(w-1)/2+dx,y+(h-1)/2+dy)).astype(np.float32)
            score=float(cv2.matchTemplate(cur,target,cv2.TM_CCOEFF_NORMED)[0,0])
            best=max(best,score)
    return best

def light_effect(template,cur):
    # 交错 8×8 方块两折交叉验证：先拟合全块增益/偏置，再允许沿垂直方向渐变。
    # 在不同像素上评估残差；这项操作仅解释外观，绝不作为身份通过条件。
    h,w=template.shape
    yy,xx=np.indices((h,w))
    X=np.column_stack((template.ravel(),np.ones(w*h),yy.ravel()/(h-1)))
    obs=cur.ravel().astype(float)
    fold=((xx//8+yy//8)%2).ravel()
    rss=[]
    for cols in (2,3):
        err=0.
        for v in (0,1):
            train=fold==v
            coeff=np.linalg.lstsq(X[train,:cols],obs[train],rcond=None)[0]
            err+=np.sum((obs[~train]-X[~train,:cols].dot(coeff))**2)
        rss.append(err)
    return 1-rss[1]/rss[0]

def fiducial(first,gray,box):
    # 当次实拍第0帧作为独立基准，分别跟踪黑挡左端和球面，不参与纸尺配准。
    x,y,w,h=box
    score,margin,(u,v)=RulerReference._peak(gray[y-20:y+h+20,x-20:x+w+20],first[y:y+h,x:x+w])
    return u-20,v-20,score,margin

def main():
    diag=records(OUT/'frames_evidence.csv')
    alt=UpperEdgeReference(dict(cfg,boxes=[ALT[0],boxes[1],boxes[2]]))
    result=[];negative=[]
    for i,ts,im,g,_ in decoded(replay(RUN)):
        d=diag[i]
        if i==0:first=g.copy()
        p=[dict((k,float(d['p%d_%s'%(j,k)])) for k in ('dx','dy','score','margin')) for j in (1,2,3)]
        t,_=fit(boxes,p)
        r={'frame':i,'onboard':d['onboard'],'reason':d['board_reason']}
        for j,((x,y,w,h),v) in enumerate(zip(boxes,p),1):
            xx,yy=x+int(v['dx']),y+int(v['dy'])
            patch=ref[y:y+h,x:x+w]
            r['p%d_gradient_rss_gain'%j]=light_effect(patch,g[yy:yy+h,xx:xx+w])
            r['p%d_best_halfpixel'%j]=subpixel(g,xx,yy,patch)
        chk=[dict((k,float(d['w%d_%s'%(j,k)])) for k in ('dx','dy','score','margin')) for j in (1,2,3,4)]
        ids=[k for k in range(4) if chk[k]['score']>=.85 and chk[k]['margin']>=.05]
        # 独立点横跨尺面的最远一对，计算参考连线在像平面的角度变化。
        left,right=ids[0],ids[-1]
        v0=center(CHECK[right])-center(CHECK[left])
        v1=v0+np.array([chk[right][k]-chk[left][k] for k in ('dx','dy')])
        r['independent_image_angle_deg']=np.degrees(np.arctan2(v1[1],v1[0])-np.arctan2(v0[1],v0[0]))
        for label,b in [('ball',(610,596,36,39)),('stop',(312,577,60,42))]:
            for k,v in zip(['dx','dy','score','margin'],fiducial(first,g,b)):r[label+'_'+k]=v
        eligible=(d['all_replay']!='confirmed_upper_edge' or
                  (d['onboard']=='True' and d['board_reason']!='confirmed_upper_edge'))
        alt.matcher.transform=None
        recover=candidate(g,alt,p,t) if eligible else None
        r['left_only_fallback']=recover is not None
        r['witness_max_px']=None
        if recover is not None:
            xy=[center(CHECK[j]) for j in ids]
            actual=np.array(xy)+np.array([[chk[j]['dx'],chk[j]['dy']] for j in ids])
            r['witness_max_px']=float(np.max(np.linalg.norm(mapped(recover,xy)-actual,axis=1)))
        result.append(r)
        if i%100==0:
            for name in ('occlusion','duplicate','nonrigid'):
                fake=g.copy()
                if name=='occlusion':fake[615:690,365:493]=127
                if name=='duplicate':fake[529:614,200:1150]=g[615:700,200:1150]
                if name=='nonrigid':fake[620:697,613:750]=g[609:686,613:750]
                m=[peaks(fake,b) for b in boxes]
                proposal,_=fit(boxes,m)
                alt.matcher.transform=None
                negative.append(dict(frame=i,case=name,left_only_fallback=candidate(fake,alt,m,proposal) is not None))
    write_table('controls.csv',result)
    write_table('left_only_challenges.csv',negative)
if __name__=='__main__':main()
