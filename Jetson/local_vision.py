# -*- coding: utf-8 -*-
"""局部视觉辅助：纸尺身份确认、仅用于搜索的物理ROI、背景相对球顶。

这些参考点不是正式厘米标定。新候选坐标不能通过旧geom标定接口。
"""
from collections import deque, namedtuple
from numbers import Real
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
        # 两套模板从同一张已验哈希的照片取样，复用原配准器及全部搜索和身份门限。
        options = dict(search_x=int(cfg.get('search_x',45)), search_y=int(cfg.get('search_y',45)),
                       rotation_recheck=cfg.get('rotation_recheck',False),
                       adaptive_search=cfg.get('adaptive_search',False))
        self.matcher = RulerReference(image, cfg['boxes'], **options)
        self.other_matcher = None
        self.reference_set = 'primary'
        if 'fallback_boxes' in cfg:
            if len(cfg['boxes']) != 3 or len(cfg['fallback_boxes']) != 3:
                raise ValueError('dual reference requires two complete three-patch sets')
            self.other_matcher = RulerReference(image, cfg['fallback_boxes'], **options)
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
        if self.other_matcher is not None:
            if not info['valid'] and gray is not None and gray.shape == self.matcher.shape and gray.dtype == np.uint8:
                # 仅身份失败才复查另一套完整纹理。交接时清空旧姿态，当前帧独立重定位。
                self.other_matcher.transform = None
                self.other_matcher._last_rectified = False
                other = self.other_matcher.update(gray)
                if other['valid']:
                    # 拒绝强且有竞争峰的原模板，避免备用裁框绕开当前帧的身份歧义。
                    _, scores, margins = self.matcher._raw_peaks(gray)
                    tied = any(s >= self.matcher.min_score and m < self.matcher.min_margin
                               for s, m in zip(scores, margins))
                    if tied:
                        self.other_matcher.transform = None
                        self.reason = 'reference_competing_patch'
                        return None
                    self.matcher, self.other_matcher = self.other_matcher, self.matcher
                    self.reference_set = 'fallback' if self.reference_set == 'primary' else 'primary'
                    info = other
            if info['valid']:
                info['reference_set'] = self.reference_set
                info['registration_mode'] = self.reference_set + '/' + info.get('registration_mode','registered')
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


# Directions used to verify and refine a circle against the observed outer rim.
_angles = np.arange(36) * (2 * np.pi / 36)
_rim_x, _rim_y = np.cos(_angles), np.sin(_angles)


def topview_balls(gray, rail, bounds, candidate_type, diagnostics):
    height, width = gray.shape
    min_radius, max_radius = 18, 32  # Observed diameter range at 1280x720.
    margin = max_radius + 8
    left = max(0, int(np.floor(bounds[0])) - margin)
    right = min(width, int(np.ceil(bounds[1])) + margin + 1)
    if right <= left:
        return [], 'roi_empty'
    edge = rail.edge_profile[left:right]
    top = max(0, int(edge.min()) - 2 * max_radius - 16)
    bottom = min(height, int(edge.max()) + 20)
    if bottom - top < 2 * min_radius:
        return [], 'roi_clipped'
    patch = cv2.GaussianBlur(gray[top:bottom, left:right], (5, 5), 1.2)
    # 先保持静止球的提案顺序；首轮已确认球体时，直接返回，避免改变其测量坐标。
    # 快速移动使边缘沿运动方向拉宽，24 px 抑制距离可能只留下偏到轨唇上的圆。
    primary_distance = 24
    for distance, votes in ((primary_distance, 17), (8, 15)):
        circles = cv2.HoughCircles(patch, cv2.HOUGH_GRADIENT, 1, distance,
                                   param1=70, param2=votes, minRadius=min_radius, maxRadius=max_radius)
        verified = _verify_topview_circles(patch, circles, left, top, rail, bounds, candidate_type)
        if verified:
            balls = []
            for ball, record in sorted(verified, key=lambda item: item[0].confidence, reverse=True):
                # 密集提案可能多次拟合同一个球。只在补搜时合并近于首轮抑制距离的
                # 圆心；保留分开的候选，让下游继续执行歧义检查。
                if distance < primary_distance and any(
                        np.hypot(ball.x - other.x, record['center_y_px'] -
                                 other.extra['measurement_y_px']) < primary_distance
                        for other in balls):
                    continue
                record['hough_min_distance_px'] = distance
                balls.append(ball)
                diagnostics.append(record)
            return balls, 'candidates'
    return [], 'no_circle'


def _verify_topview_circles(patch, circles, left, top, rail, bounds, candidate_type):
    """两轮共用原图圆周、高光和轨沿检验；输入圆心在局部窄带内，输出圆心在全图。"""
    from measurement_contract import TOPVIEW_CIRCLE_VERSION
    edge = rail.edge_profile[left:left + patch.shape[1]]
    verified = []
    for x, y, radius in (() if circles is None else circles[0]):
        if not bounds[0] <= x + left <= bounds[1]:
            continue
        if abs(edge[int(x)] - (y + top) - radius) > 14:
            continue
        if min(x, y, patch.shape[1]-1-x, patch.shape[0]-1-y) < radius + 7:
            continue
        # Search across the proposed rim. A convex dark-to-light boundary can
        # remain visible on both sides even when the tool hides part of the rim.
        distances = radius + np.arange(-5, 6)[:, None]
        xx = x + distances * _rim_x
        yy = y + distances * _rim_y
        outer = patch[np.rint(yy + 2*_rim_y).astype(int), np.rint(xx + 2*_rim_x).astype(int)]
        inner = patch[np.rint(yy - 2*_rim_y).astype(int), np.rint(xx - 2*_rim_x).astype(int)]
        contrast = outer.astype(np.int16) - inner
        peaks = contrast.argmax(axis=0)
        sectors = np.arange(len(_rim_x))
        # The bottom arc can be hidden by the rail lip. Do not fit its straight edge.
        rim_y = yy[peaks, sectors] + top
        lip_y = edge[np.rint(xx[peaks, sectors]).astype(int)]
        supported = (contrast[peaks, sectors] >= 16) & (rim_y < lip_y - 2)
        if supported.sum() < 18 or any(np.count_nonzero(supported & side) < 3
                                       for side in (_rim_x < -.5, _rim_x > .5)):
            continue
        dx = xx[peaks, sectors][supported] - x
        dy = yy[peaks, sectors][supported] - y
        system = np.column_stack((2*dx, 2*dy, np.ones(len(dx))))
        ox, oy, c = np.linalg.lstsq(system, dx*dx + dy*dy, rcond=None)[0]
        if np.hypot(ox, oy) > 5 or c <= 0:
            continue
        rad = float(np.sqrt(c + ox*ox + oy*oy))
        if abs(rad - radius) > 3.5 or np.median(abs(np.hypot(dx-ox, dy-oy)-rad)) > 2.5:
            continue
        cx, cy = float(left+x+ox), float(top+y+oy)
        if not bounds[0] <= cx <= bounds[1]:
            continue
        a, b = int(cx-rad), int(cx+rad)
        if not (rail.edge_support[max(0,a-10):a].any() and rail.edge_support[b+1:b+11].any()):
            continue
        # Exclude the exterior: adjacent white ruler/rail must not supply the highlight.
        half = int(rad * .47)
        u, v = int(round(x+ox)), int(round(y+oy))
        core = patch[v-half:v+half+1, u-half:u+half+1]
        appearance = float(np.percentile(core, 95) - np.median(core))
        if appearance < 35:
            continue
        confidence = .65 + .30 * supported.mean()
        ball = candidate_type(cx, 2*rad, 2*rad, confidence, 'topview_circle',
                              dict(coordinate_version=TOPVIEW_CIRCLE_VERSION, measurement_y_px=cy,
                                   identity_valid=True, in_search_roi=True, appearance=appearance))
        record = dict(center_px=cx, center_y_px=cy, radius_px=rad,
                      supported_sectors=int(supported.sum()), reason='accepted')
        verified.append((ball, record))
    return verified


_CircleObservation = namedtuple('_CircleObservation', 'time x height radius patch')


class MotionCircleRecovery(object):
    """用连续普通圆测量授权短时补搜；所有位置输出仍来自当帧的已验证圆周。"""

    def __init__(self):
        self.history = deque(maxlen=3)
        self.previous_fallback = None

    def reset(self):
        self.history.clear()
        self.previous_fallback = None

    def update(self, gray, rail, bounds, normal, timestamp, candidate_type):
        if (not isinstance(timestamp, Real) or isinstance(timestamp, bool) or
                not np.isfinite(timestamp) or not rail.valid or not rail.identity_valid):
            self.reset()
            return []
        h = self.history
        previous = self.previous_fallback
        if previous and not .012 <= timestamp - previous[0] <= .080:
            self.reset()
            previous = None
        if h and not .012 <= timestamp - h[-1].time <= (.100 if previous else .080):
            self.reset()
            previous = None
        if normal:
            # 歧义或补搜后再次出现普通圆时从头积累，避免拼接不同物体的历史。
            if previous or len(normal) != 1:
                self.reset()
            if len(normal) == 1:
                ball = normal[0]
                x = int(round(ball.x))
                y = int(round(ball.extra['measurement_y_px']))
                patch = gray[y-24:y+25, x-24:x+25]
                if patch.shape == (49, 49):
                    h.append(_CircleObservation(timestamp, ball.x - bounds[0],
                             ball.extra['measurement_y_px'] - rail.edge_profile[x],
                             ball.width / 2, patch.copy()))
                else:
                    self.reset()
            return []

        # 至少三次连续的普通测量才能核对两段速度是否同向。用左界相对坐标
        # 去除纸尺的平移。任一补搜失败即注销授权；首个补搜不刷新普通圆时钟。
        anchors = tuple(h)
        self.reset()
        if len(anchors) != 3:
            return []
        first, middle, last = anchors
        v1 = (middle.x - first.x) / (middle.time - first.time)
        v2 = (last.x - middle.x) / (last.time - middle.time)
        age = timestamp - last.time
        if (age > (.100 if previous else .075) or v1*v2 <= 0 or
                min(abs(v1), abs(v2)) < 150 or max(abs(v1), abs(v2)) > 1200 or
                not .45 <= v2/v1 <= 2.2):
            return []
        expected = last.x + bounds[0] + age*v2
        if not bounds[0] <= expected <= bounds[1]:
            return []
        cx = int(round(expected))
        cy = int(round(rail.edge_profile[cx] + last.height))
        search = gray[cy-34:cy+35, cx-36:cx+37]
        if search.shape != (69, 73):
            return []
        peaks = []
        for anchor, minimum_score in ((middle, .78), (last, .88)):
            response = cv2.matchTemplate(search, anchor.patch, cv2.TM_CCOEFF_NORMED)
            _, score, _, (u, v) = cv2.minMaxLoc(response)
            if score < minimum_score or u in (0, 24) or v in (0, 20):
                return []
            peaks.append((cx-12+u, cy-10+v))
        if np.hypot(peaks[1][0]-peaks[0][0], peaks[1][1]-peaks[0][1]) > 7:
            return []
        # 两张实测外观分别复核当前帧。只在一致的小窗中提议圆，再用原有
        # 拟合器检查圆周、轨沿、ROI 和高光；模板峰及线性预测均不作为测量。
        x, y = peaks[-1]
        left, top = max(0, x-42), max(0, y-42)
        patch = cv2.GaussianBlur(gray[top:y+43, left:x+43], (5, 5), 1.2)
        seeds = _motion_glint_seeds(patch, rail.edge_profile[left:left+patch.shape[1]] - top)
        hits = _verify_topview_circles(patch, seeds, left, top, rail, bounds, candidate_type)
        if not hits:
            return []
        ball, record = max(hits, key=lambda hit: hit[0].confidence)
        rel_x = ball.x - bounds[0]
        if (any(np.hypot(ball.x-u, record['center_y_px']-v) > 7 for u, v in peaks) or
                not .5 <= (rel_x-last.x)/(age*v2) <= 1.8):
            return []
        if previous:
            if not .5 <= (rel_x-previous[1])/((timestamp-previous[0])*v2) <= 1.8:
                return []
        else:
            h.extend(anchors)
            self.previous_fallback = (timestamp, rel_x)
        # 只有通过当帧全部复核的实测候选才携带来源，供调用方识别观测路径。
        ball.extra['proposal_source'] = 'motion_blur'
        record['proposal_source'] = 'motion_blur'
        return [(ball, record)]


def _motion_glint_seeds(patch, edge):
    """只接受窄窗内唯一的一对横向高光。多对竞争或单个高光均不生成初值。"""
    glints = cv2.morphologyEx(patch, cv2.MORPH_TOPHAT, np.ones((11, 1), np.uint8))
    _, _, stats, centers = cv2.connectedComponentsWithStats((glints > 38).astype(np.uint8))
    streaks = []
    for (_, _, w, h, area), (x, y) in zip(stats[1:], centers[1:]):
        if (16 <= w <= 56 and 3 <= h <= 13 and w >= 2*h and area >= 35 and
                10 <= edge[int(x)] - y <= 52):
            streaks.append((x, y, w))
    pair = None
    for x, y, w in streaks:
        for u, v, width in streaks:
            if 7 <= v-y <= 16 and abs(u-x) <= 7 and min(w, width) >= .65*max(w, width):
                if pair is not None:
                    return None
                pair = ((x+u)/2, v+1)
    if pair is None:
        return None
    return np.asarray([[(pair[0], pair[1], radius) for radius in (24, 27)]], np.float32)
