"""独立印刷数字核图。采样规则为每 45 帧，加全量回归中所有匹配点变化帧。"""
from bench import *
CHECK=[(302,632,70,18),(488,649,105,28),(758,661,110,17),(989,626,89,40)]
origin=np.array(cfg['rail_coordinates']['origin_px'])
def records(path):
    with path.open() as f:return {int(x['frame']):x for x in csv.DictReader(f)}
def main():
    evidence=[]
    for name in ('moving_free','parked_free','fixed_O_tilt','baseline','live_pixel'):
        path=(ROOT.parent/'moving_check' if name in ('baseline','live_pixel') else ROOT/'runs')/name
        rows=records(HERE/(name+'_all.csv'))
        arc=records(ROOT/('digits_'+name)/'measurements.csv')
        # 旧归档保留完整浮点姿态；同帧新旧三点全部相同时，拟合姿态也相同。
        for i,ts,im,g,_ in decoded(replay(path)):
            prior=prev if i else None
            prev=g
            change=rows[i]['max_point_delta'] not in ('','0.0')
            if not change and i%45:continue
            if rows[i]['new_valid']!='True':continue
            entry=arc[i]
            a=np.deg2rad(float(entry['rotation_deg']))
            mat=float(entry['reference_scale'])*np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
            shift=np.array([float(entry[k]) for k in ('origin_x_px','origin_y_px')])-mat.dot(origin)
            if change:
                old,new=pipeline(Original)[0],pipeline(candidate.RulerReference)[0]
                # 两次从前一帧位姿预热，并对齐全序列归档的原点。
                for rd in (old,new):
                    rd.detect(prior,force=True)
                ro=old.detect(g,force=True);rn=new.detect(g,force=True)
                po,pn=[np.array(rail.registration['current_points']) for rail in (ro,rn)]
                mat=np.linalg.inv(rn.reference_inverse);shift=rn.reference_translation
                np.testing.assert_allclose(mat.dot(origin)+shift,[float(rows[i][k]) for k in ('new_ox','new_oy')],atol=1e-6)
                blocks=[]
                for j,(x,y,w,h) in enumerate(r['boxes']):
                    # 冻结与候选峰位置分别用蓝十字、橙圆圈表示。
                    x0=x-27;y0=y-23
                    cut=im[y0:y+h+23,x0:x+w+27].copy()
                    cut=cv2.resize(cut,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST)
                    u,v=np.rint((po[j]-[x0,y0])*3).astype(int)
                    x2,y2=np.rint((pn[j]-[x0,y0])*3).astype(int)
                    cv2.drawMarker(cut,(u,v),(225,130,25),cv2.MARKER_CROSS,15,1)
                    cv2.circle(cut,(x2,y2),7,(20,130,255),1)
                    cv2.putText(cut,'P%d d=(%.0f, %.0f)'%(j+1,*(pn[j]-po[j])),(7,22),0,.55,(0,255,255),1)
                    blocks.append(cv2.copyMakeBorder(cut,0,265-cut.shape[0],0,0,cv2.BORDER_CONSTANT))
                assert cv2.imwrite(str(HERE/('compare_'+name+'_'+str(i)+'.png')),np.hstack(blocks))
                print(name,i,po.tolist(),pn.tolist(),flush=True)
            for j,(x,y,w,h) in enumerate(CHECK):
                resp=g[y-96:min(720,y+h+96),x-96:x+w+96]
                score,margin,(u,v)=Original._peak(resp,ref[y:y+h,x:x+w])
                point=np.array([x+w/2.,y+h/2.])
                actual=point+[u-96,v-96]
                error=np.linalg.norm(actual-(mat.dot(point)+shift))
                evidence.append(dict(scene=name,frame=i,witness=j+1,score=score,margin=margin,
                                     trusted=score>=.85 and margin>=.05,error_px=error))
            prev=g
    with (HERE/'witnesses.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(evidence[0]));w.writeheader();w.writerows(evidence)
if __name__=='__main__':main()
