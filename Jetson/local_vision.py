# -*- coding: utf-8 -*-
"""局部视觉辅助：纸尺身份确认、仅用于搜索的物理ROI、背景相对球顶。

这些参考点不是正式厘米标定。新候选坐标不能通过旧geom标定接口。
"""
import hashlib
from pathlib import Path
import cv2
import numpy as np
from ruler_reference import RulerReference


class PhysicalSearch(object):
    """在参考图中插值搜索边界，再随当前参考变换移动；禁止外推。"""
    def __init__(self, cfg):
        self.enabled = bool(cfg.get('enabled', False))
        self.anchors = np.asarray(cfg.get('anchors', []), dtype=float)
        self.low = float(cfg.get('target_min_cm', -5))-float(cfg.get('recovery_margin_cm', 2))
        self.high = float(cfg.get('target_max_cm', 5))+float(cfg.get('recovery_margin_cm', 2))
        if self.enabled:
            target_low=float(cfg.get('target_min_cm',-5))
            target_high=float(cfg.get('target_max_cm',5))
            margin=float(cfg.get('recovery_margin_cm',2))
            if not np.isfinite([target_low,target_high,margin]).all() or target_low>=target_high or margin<0:
                raise ValueError('ROI requires finite ordered targets and nonnegative margin')
            a = self.anchors
            if (a.ndim != 2 or a.shape[1] != 2 or len(a)<3 or
                    not np.isfinite(a).all() or not np.all(np.diff(a[:,0])>0) or
                    not np.all(np.diff(a[:,1])>0) or self.low>=self.high or
                    float(cfg.get('recovery_margin_cm',2))<0 or
                    self.low<a[0,0] or self.high>a[-1,0]):
                raise ValueError('ROI requires ordered observed anchors covering full search range')

    def bounds(self, rail):
        if not self.enabled:
            return None
        if not rail.valid or not getattr(rail, 'identity_valid', False):
            return None
        x = np.interp([self.low,self.high],self.anchors[:,0],self.anchors[:,1])
        y = np.interp(x,rail.reference_profile[:,0],rail.reference_profile[:,1])
        matrix = np.linalg.inv(rail.reference_inverse)
        pts = np.column_stack((x,y)).dot(matrix.T)+rail.reference_translation
        return float(pts[0,0]),float(pts[1,0])


class UpperEdgeReference(object):
    """纹理匹配确认纸尺身份，梯度只在已确认上缘附近微调，不追踪最强横边。"""
    def __init__(self, cfg):
        path=Path(cfg['image'])
        if not path.is_absolute():path=Path(__file__).resolve().parent/path
        if hashlib.sha256(path.read_bytes()).hexdigest()!=cfg['sha256']:
            raise ValueError('upper-edge reference image changed')
        image=cv2.imread(str(path),cv2.IMREAD_GRAYSCALE)
        self.matcher=RulerReference(image,cfg['boxes'],search_x=int(cfg.get('search_x',45)),
                                    search_y=int(cfg.get('search_y',45)))
        self.profile=np.asarray(cfg['upper_edge_points'],dtype=float)
        if (self.profile.ndim!=2 or self.profile.shape[1]!=2 or len(self.profile)<3 or
                not np.isfinite(self.profile).all() or not np.all(np.diff(self.profile[:,0])>0)):
            raise ValueError('invalid upper edge anchors')
        self.reason='not_updated'
        self.max_adjust=float(cfg.get('max_edge_adjust_px',5))
        self.support_margin=int(cfg.get('support_margin_px',80))
        if not 0<=self.support_margin<=256:
            raise ValueError('support margin must be between 0 and 256 pixels')

    def locate(self, gray):
        info=self.matcher.update(gray)
        self.reason='reference_'+info['reason']
        if not info['valid']:return None
        inv,translation=self.matcher.transform
        points=self.profile.dot(np.linalg.inv(inv).T)+translation
        W=gray.shape[1];xs=np.arange(W,dtype=float)
        expected=np.interp(xs,points[:,0],points[:,1])
        # 只为球冠两侧的实测边缘提供有限取证余量，不外推厘米搜索范围。
        margin=int(self.support_margin)
        for a,b,mask in [(points[0],points[1],xs<points[0,0]),
                         (points[-2],points[-1],xs>points[-1,0])]:
            expected[mask]=a[1]+(xs[mask]-a[0])*(b[1]-a[1])/(b[0]-a[0])
        # 上缘细化只接受亮纸尺进入处的正梯度；多列证据支持才确认几何。
        lo=max(1,int(expected.min()-self.max_adjust-2));hi=min(gray.shape[0]-1,int(expected.max()+self.max_adjust+3))
        if hi-lo<3:
            self.reason='upper_edge_geometry'
            return None
        grad=cv2.Sobel(gray[lo-1:hi+1],cv2.CV_32F,0,1,ksize=3)[1:-1]
        ys=np.arange(lo,hi)[:,None]
        allow=np.abs(ys-expected[None,:])<=self.max_adjust
        grad=np.where(allow,grad,-1e6)
        arg=grad.argmax(axis=0);edge=arg+lo;strength=grad.max(axis=0)
        domain=(xs>=points[0,0])&(xs<=points[-1,0]);good=domain&(strength>35)
        support=(xs>=points[0,0]-margin)&(xs<=points[-1,0]+margin)&(strength>35)
        if good.sum()<.55*domain.sum():
            self.reason='upper_edge_geometry';return None
        # 不让钢珠凸起改变纸尺基线：151列中值窗大于两倍65像素最大球冠宽度。
        # 原51列窗口会让宽浅球冠占多数，使基线爬到球顶并将轮廓切成两个假候选。
        # 参考纸尺曲线仍保留，只抑制局部梯度修正中的球体遮挡。
        correction=np.clip(edge-expected,-self.max_adjust,self.max_adjust).astype(np.float32)
        correction[~support]=0
        correction=(cv2.medianBlur(np.clip(128+4*correction,0,255).astype(np.uint8).reshape(1,-1),151).ravel().astype(float)-128)/4.
        edge=expected+correction
        slope,intercept=np.polyfit(xs[good],edge[good],1)
        self.reason='ok'
        return dict(slope=float(slope),intercept=float(intercept),confidence=float(good.sum()/domain.sum()),
                    edge_profile=edge,edge_support=support,reference_inverse=inv,reference_translation=translation,
                    reference_profile=self.profile,registration=info)


def relative_caps(frame, rail, cfg, x_limits, candidate_type, diagnostics=None):
    """按列估计背景Lab，提取与背景有差异且连接上缘的浅球顶；返回新坐标版本。"""
    H,W=frame.shape[:2]
    edge=getattr(rail,'edge_profile',rail.slope*np.arange(W)+rail.intercept)
    support=getattr(rail, 'edge_support', None)
    if support is None:return [],'local_geometry_unavailable'
    # 搜索区限制测量中心；取证区包含完整球冠，禁止裁剪后伪造中心。
    halo=int(np.ceil(float(cfg.get('max_width_px',65))))+2
    left=max(0,int(np.ceil(x_limits[0]))-halo);right=min(W,int(np.floor(x_limits[1]))+1+halo)
    if right-left<12:return [],'roi_empty'
    top=max(0,int(np.floor(edge[left:right].min()-48)));bottom=min(H,int(np.ceil(edge[left:right].max()+3)))
    if bottom-top<48:return [],'roi_clipped'
    lab=cv2.cvtColor(frame[top:bottom,left:right],cv2.COLOR_BGR2LAB).astype(np.float32)
    columns=np.arange(right-left)
    baseline=edge[left:right]-top
    # 背景条带与球顶分离；每列独立估计以容忍照明不均。颜色不预设为蓝色。
    bgrows=np.clip(np.round(baseline[None,:]+np.arange(-45,-28)[:,None]).astype(int),0,lab.shape[0]-1)
    samples=lab[bgrows,columns[None,:]]
    bg=np.median(samples,axis=0)
    noise=np.median(np.linalg.norm(samples-bg[None,:,:],axis=2),axis=0)
    diff=lab-bg[None,:,:]
    distance=np.sqrt(.35*diff[:,:,0]**2+diff[:,:,1]**2+diff[:,:,2]**2)
    threshold=np.maximum(float(cfg.get('contrast_min',18)),5*noise)
    ys=np.arange(lab.shape[0])[:,None];depth=baseline[None,:]-ys
    # 纸尺顶部3像素为边缘量化/反光容差，不纳入球体；防止薄亮边连接整段轮廓。
    mask=(distance>threshold[None,:])&(depth>=float(cfg.get('rim_exclusion_px',3)))&(depth<=float(cfg.get('max_height_px',28)))
    mask=cv2.morphologyEx(mask.astype(np.uint8),cv2.MORPH_CLOSE,np.ones((2,3),np.uint8))
    n,labels,stats,_=cv2.connectedComponentsWithStats(mask,8)
    out=[]
    for label in range(1,n):
        x,y,w,h,area=[int(v) for v in stats[label]]
        record={'left_px':x+left,'right_px':x+left+w-1,'width':w,'height':h,'area':area,
                'reason':'accepted','center_px':None,
                'footprint_in_search':bool(x+left>=x_limits[0] and x+left+w-1<=x_limits[1])}
        if diagnostics is not None:diagnostics.append(record)
        if not float(cfg.get('min_width_px',12))<=w<=float(cfg.get('max_width_px',65)):
            record['reason']='width';continue
        if not 3<=h<=float(cfg.get('max_height_px',28)) or not 1.2<=w/max(h,1)<=16:
            record['reason']='height_aspect';continue
        # 球体遮挡中间上缘是正常现象，但两侧必须各有当前帧的真实边缘支持。
        # 防止把模板预测的高度当作已观测几何，接受悬空亮斑。
        a,b=x+left,x+left+w
        shoulders=[support[max(0,a-12):max(0,a-2)],support[min(W,b+2):min(W,b+12)]]
        if any(len(side)<6 or np.mean(side)<.6 for side in shoulders):
            record['reason']='edge_support';continue
        component=labels[y:y+h,x:x+w]==label
        py,px=np.where(component);xx=px+x;yy=py+y
        dd=baseline[xx]-yy
        if dd.max()>=float(cfg.get('max_height_px',28))-1 or dd.min()>float(cfg.get('rim_exclusion_px',3))+3:
            record['reason']='vertical_truncation_or_detached';continue
        if area<float(cfg.get('min_area_px',40)) or area/(w*h)<.35:
            record['reason']='area_fill';continue
        first=np.full(w,np.nan)
        for col in range(w):
            v=py[px==col]
            if len(v):first[col]=baseline[x+col]-(y+v.min())
        if not np.isfinite(first).all():
            record['reason']='incomplete_columns';continue
        record['terminal_depth_px']=[float(first[0]),float(first[-1])]
        # 圆弧必须有内部顶点、两侧回落；平纸尺亮边不能仅因面积足够就通过。
        axis=np.linspace(-1,1,w);q=np.polyfit(axis,first,2)
        if q[0]>=-1.5 or abs(-q[1]/(2*q[0]))>.7:
            record['reason']='arc_shape';continue
        if np.median(np.abs(np.polyval(q,axis)-first))>2.0:
            record['reason']='arc_residual';continue
        shoulder=max(np.mean(first[:max(2,w//5)]),np.mean(first[-max(2,w//5):]))
        if np.max(first)-shoulder<1.5:
            record['reason']='arc_shoulders';continue
        weights=np.maximum(first-1,0);cx=float(np.sum((np.arange(w)+x+left)*weights)/weights.sum())
        if x+left<=left or x+w+left>=right:
            record['reason']='extraction_clipped';continue
        record['center_px']=cx
        # 残弧的加权中心可能向未遮挡侧偏移；边界接受还需拟合弧顶与加权中心的范围一致性。
        # 弧顶只作范围一致性门控，不代替relative-cap-v1的原测量坐标，也不钳位。
        apex=float(left+x+(1-q[1]/(2*q[0]))*(w-1)/2.)
        record['fitted_apex_px']=apex
        if not x_limits[0]<=apex<=x_limits[1]:
            record['reason']='arc_apex_outside_search';continue
        if not x_limits[0]<=cx<=x_limits[1]:
            record['reason']='center_outside_search';continue
        cy=float(np.sum((edge[np.arange(w)+x+left]-first)*weights)/weights.sum())
        appearance=float(np.median(distance[yy,xx]))
        confidence=min(.95,.65+.15*min(1.,-q[0]/5)+.1*min(1.,area/200.))
        out.append(candidate_type(cx,w,float(first.max()),confidence,'relative_cap',
                   {'coordinate_version':'relative-cap-v1','measurement_y_px':cy,'identity_valid':True,
                    'in_search_roi':True,'appearance':appearance,'area_px':area,'curvature':float(q[0])}))
    return sorted(out,key=lambda c:c.confidence,reverse=True),'candidates' if out else 'no_cap'
