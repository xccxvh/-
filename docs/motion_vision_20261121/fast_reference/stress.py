"""合成已知坐标验真、逐块遮挡、异地副本、非刚性移动和越界试验。"""
from bench import *
def mk(cls):
    return cls(ref,r['boxes'],96,96,rotation_recheck=True,adaptive_search=True)
def stamp(dx,dy,angle,scale):
    origin=cfg['rail_coordinates']['origin_px']
    m=cv2.getRotationMatrix2D(tuple(origin),angle,scale)
    m[:,2]+=[dx,dy]
    g=cv2.warpAffine(ref,m,(ref.shape[1],ref.shape[0]))
    return g,m
def check():
    old,new=mk(Original),mk(candidate.RulerReference)
    data=[]
    # 含缓慢变化、跨帧跳变、倾角及比例变化的轨迹。利用已知仿射矩阵核对三个点。
    for i in range(160):
        dx=7*np.sin(i*.11)+(62 if 40<=i<80 else -61 if 90<=i<130 else 0)
        dy=-15+6*np.cos(i*.16)
        angle=3.8*np.sin(i*.075)
        scale=1+.014*np.cos(i*.11)
        g,m=stamp(dx,dy,angle,scale)
        truth=new.points.dot(m[:,:2].T)+m[:,2]
        row=dict(case='motion',index=i)
        for label,tr in [('old',old),('new',new)]:
            o=tr.update(g)
            row[label+'_valid']=o['valid']
            row[label+'_err']=float(np.max(np.linalg.norm(np.array(o['current_points'])-truth,axis=1))) if o['valid'] else None
            if not o['valid']:assert tr.transform is None
        data.append(row)
    frames=decoded(replay(ROOT/'runs/moving_free'))
    _,_,_,initial,_=next(frames)
    _,_,_,warm,_=next(frames)
    frames.close()
    for image in (ref,warm):
        for j,(x,y,w,h) in enumerate(r['boxes']):
            for name in ('occlusion','duplicate','independent_shift','blank','out_of_frame'):
                g=image.copy()
                if name=='occlusion':g[y-20:y+h+20,x-25:x+w+25]=127
                if name=='duplicate':g[y-12-86:y+h+16-86,x-18:x+w+18]=image[y-12:y+h+16,x-18:x+w+18]
                if name=='independent_shift':g[y-20:y+h+20,x-20:x+w+20]=image[y-32:y+h+8,x-20:x+w+20]
                if name=='blank':g[:]=127
                if name=='out_of_frame':g=cv2.warpAffine(image,np.float32([[1,0,0],[0,1,85]]),(1280,720))
                row=dict(case=name,index=j)
                for label,cls in [('old',Original),('new',candidate.RulerReference)]:
                    tr=mk(cls);assert tr.update(image)['valid']
                    o=tr.update(g)
                    row[label+'_valid']=o['valid'];row[label+'_err']=None
                    if not o['valid']:assert tr.to_reference(650,645) is None
                data.append(row)
    with (HERE/'stress.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(data[0]));w.writeheader();w.writerows(data)
    for name in ('motion','occlusion','duplicate','independent_shift','blank','out_of_frame'):
        rows=[d for d in data if d['case']==name]
        print(name,len(rows),[(label,sum(d[label+'_valid'] for d in rows),max((d[label+'_err'] for d in rows if d[label+'_err'] is not None),default=None)) for label in ('old','new')])
if __name__=='__main__':check()
