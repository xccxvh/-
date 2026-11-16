"""Per-frame ruler reference registration; no camera or actuator ownership.

Three or more textured patches must agree on a 2-D similarity transform.
This corrects image translation, in-plane rotation and uniform scale, not
arbitrary perspective. A failed update clears the transform immediately.
Coordinates are reference-image pixels, never implicitly centimeters.
"""
import math
import cv2
import numpy as np


class RulerReference(object):
    def __init__(self, gray, boxes, search_x=32, search_y=24,
                 min_score=.85, min_margin=.05, max_residual_px=2.5,
                 rotation_recheck=False):
        if gray is None or gray.ndim != 2 or gray.dtype != np.uint8:
            raise ValueError('reference must be uint8 grayscale')
        # 必须显式布尔启用；旧调用默认保留原匹配及提前失败行为。
        if type(rotation_recheck) is not bool:
            raise ValueError('rotation_recheck must be boolean')
        self.rotation_recheck = rotation_recheck
        self.shape = gray.shape
        self.search_x, self.search_y = int(search_x), int(search_y)
        self.min_score, self.min_margin = float(min_score), float(min_margin)
        self.max_residual = float(max_residual_px)
        if (self.search_x < 1 or self.search_y < 1 or
                not 0 < self.min_score <= 1 or not 0 <= self.min_margin < 1 or
                not math.isfinite(self.max_residual) or self.max_residual <= 0):
            raise ValueError('invalid registration limits')
        self.patches = []
        points = []
        for box in boxes:
            x, y, w, h = [int(v) for v in box]
            if x < 0 or y < 0 or w < 8 or h < 8 or x+w > gray.shape[1] or y+h > gray.shape[0]:
                raise ValueError('invalid reference patch')
            patch = gray[y:y+h, x:x+w].copy()
            if float(patch.std()) < 5:
                raise ValueError('reference patch lacks texture')
            self.patches.append((x, y, w, h, patch))
            points.append((x+w/2.0, y+h/2.0))
        self.points = np.asarray(points, dtype=np.float64)
        if len(points) < 3 or np.ptp(self.points[:, 0]) < 100:
            raise ValueError('need at least three separated ruler patches')
        self.center = self.points.mean(axis=0)
        self.centered = self.points-self.center
        self.denominator = float((self.centered**2).sum())
        self.transform = None

    def update(self, gray):
        self.transform = None
        if gray is None or gray.shape != self.shape or gray.dtype != np.uint8:
            return {'valid': False, 'reason': 'resolution_or_format'}
        points, scores, margins = [], [], []
        H, W = self.shape
        for x, y, w, h, patch in self.patches:
            x0, x1 = max(0, x-self.search_x), min(W, x+w+self.search_x)
            y0, y1 = max(0, y-self.search_y), min(H, y+h+self.search_y)
            response = cv2.matchTemplate(gray[y0:y1, x0:x1], patch,
                                         cv2.TM_CCOEFF_NORMED)
            _, score, _, loc = cv2.minMaxLoc(response)
            xx, yy = loc
            # Exclude the local correlation peak before comparing other
            # ruler ticks/labels. Ambiguous repetitive texture is invalid.
            rx, ry = 2, 2
            response[max(0,yy-ry):yy+ry+1, max(0,xx-rx):xx+rx+1] = -1
            margin = score-float(response.max())
            if (not math.isfinite(score) or not math.isfinite(margin) or
                    score < self.min_score or margin < self.min_margin):
                if self.rotation_recheck:
                    return self._rotation_recheck(gray)
                return {'valid': False, 'reason': 'weak_or_ambiguous_patch'}
            points.append((x0+xx+w/2.0, y0+yy+h/2.0))
            scores.append(score); margins.append(margin)
        current = np.asarray(points, dtype=np.float64)
        center = current.mean(axis=0)
        v = current-center
        u = self.centered
        a = float((u*v).sum()/self.denominator)
        b = float((u[:,0]*v[:,1]-u[:,1]*v[:,0]).sum()/self.denominator)
        scale = math.hypot(a,b)
        angle = math.degrees(math.atan2(b,a))
        matrix = np.array([[a,-b],[b,a]])
        translation = center-matrix.dot(self.center)
        residual = float(np.max(np.linalg.norm(self.points.dot(matrix.T)+translation-current,axis=1)))
        if not .9 <= scale <= 1.1 or abs(angle) > 5 or residual > self.max_residual:
            return {'valid': False, 'reason': 'inconsistent_reference'}
        self.transform = (matrix.T/(scale*scale), translation)
        return {'valid': True, 'reason': 'ok', 'min_score': min(scores),
                'min_margin': min(margins), 'residual_px': residual,
                'scale': scale, 'image_rotation_deg': angle,
                # 同帧匹配点供可选点畸变矫正使用；不能把raw变换直接用于矫正坐标。
                'reference_points': self.points.tolist(), 'current_points': current.tolist()}

    @staticmethod
    def _peak(window, patch):
        """只提取原模板峰值；完整竞争范围与原2像素局部峰排除规则一致。"""
        response = cv2.matchTemplate(window, patch, cv2.TM_CCOEFF_NORMED)
        _, score, _, loc = cv2.minMaxLoc(response)
        xx, yy = loc
        response[max(0,yy-2):yy+3, max(0,xx-2):xx+3] = -1
        margin = score - float(response.max())
        return score, margin, loc

    def _rectified_search(self, gray, box, matrix, translation):
        """把完整参考搜索窗对应的当前小块逆采样，不缩窄竞争峰范围。

        参考窗和变换后四角都须完整在图内；额外留一像素给双线性采样，
        避免常量黑边/裁剪补齐成为虚假模板证据。仿射窗内部坐标是四角凸组合。
        """
        x, y, w, h, _ = box
        x0, y0 = x-self.search_x, y-self.search_y
        ww, hh = w+2*self.search_x, h+2*self.search_y
        H, W = self.shape
        if x0 < 0 or y0 < 0 or x0+ww > W or y0+hh > H:
            return None
        # 保留原“当前图像”搜索域的全部竞争中心；只固定warp后窗尺寸会
        # 随提议平移丢掉另一侧的重复峰。逆映射中心域并加完整模板足迹，
        # 与原参考完整窗取并集外接矩形。宁可多查竞争峰，不做mask缩窄域。
        cx, cy = x+w/2., y+h/2.
        centers = np.array([[cx-self.search_x,cy-self.search_y],
                            [cx+self.search_x,cy-self.search_y],
                            [cx-self.search_x,cy+self.search_y],
                            [cx+self.search_x,cy+self.search_y]])
        centers = (centers-translation).dot(np.linalg.inv(matrix).T)
        low_ref = np.floor(np.minimum([x0,y0], centers.min(axis=0)-[w/2.,h/2.])).astype(int)
        high_ref = np.ceil(np.maximum([x0+ww,y0+hh], centers.max(axis=0)+[w/2.,h/2.])).astype(int)
        x0, y0 = low_ref
        ww, hh = high_ref-low_ref
        if x0 < 0 or y0 < 0 or x0+ww > W or y0+hh > H:
            return None
        corners = np.array([[x0,y0],[x0+ww-1,y0],
                            [x0,y0+hh-1],[x0+ww-1,y0+hh-1]], dtype=float)
        current = corners.dot(matrix.T)+translation
        if (not np.isfinite(current).all() or np.any(current < 1) or
                np.any(current[:,0] > W-2) or np.any(current[:,1] > H-2)):
            return None
        # 只warp包含完整采样足迹的小块；矩阵将局部输出像素直接映到源crop。
        low = np.floor(current.min(axis=0)).astype(int)-1
        high = np.ceil(current.max(axis=0)).astype(int)+2
        crop = gray[low[1]:high[1], low[0]:high[0]]
        offset = matrix.dot(np.array([x0,y0],dtype=float))+translation-low
        affine = np.column_stack((matrix, offset))
        window = cv2.warpAffine(crop, affine, (int(ww),int(hh)),
                                flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
                                borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        return window, low_ref

    def _rotation_recheck(self, gray):
        """失败峰值仅提议姿态；每个完整窗通过原强度/歧义门限后才提交变换。"""
        H, W = self.shape
        proposed, proposal_scores = [], []
        # 原快速路径提前失败，此处重新收集全部patch（包括原失败patch）的峰值。
        for x, y, w, h, patch in self.patches:
            x0, y0 = x-self.search_x, y-self.search_y
            x1, y1 = x+w+self.search_x, y+h+self.search_y
            if x0 < 0 or y0 < 0 or x1 > W or y1 > H:
                return {'valid':False, 'reason':'rotation_recheck_clipped'}
            score, margin, (xx,yy) = self._peak(gray[y0:y1,x0:x1], patch)
            if not math.isfinite(score) or not math.isfinite(margin):
                return {'valid':False, 'reason':'rotation_proposal_untrusted'}
            proposed.append((x0+xx+w/2., y0+yy+h/2.))
            proposal_scores.append(score)
        # 不为全无可信分数的纹理估计姿态；不降低最终min_score/min_margin。
        if max(proposal_scores) < self.min_score:
            return {'valid':False, 'reason':'rotation_proposal_untrusted'}
        proposal, info = similarity_from_points(self.points, proposed, self.max_residual)
        if proposal is None:
            return {'valid':False, 'reason':'rotation_proposal_'+info['reason']}
        inverse, translation = proposal
        matrix = np.linalg.inv(inverse)
        current, scores, margins = [], [], []
        for box in self.patches:
            x, y, w, h, patch = box
            rectified_search = self._rectified_search(gray, box, matrix, translation)
            if rectified_search is None:
                return {'valid':False, 'reason':'rotation_recheck_clipped'}
            window, origin = rectified_search
            score, margin, (xx,yy) = self._peak(window, patch)
            if (not math.isfinite(score) or not math.isfinite(margin) or
                    score < self.min_score or margin < self.min_margin):
                return {'valid':False, 'reason':'rotation_recheck_weak_or_ambiguous_patch'}
            rectified = origin + np.array([xx+w/2., yy+h/2.])
            current.append(matrix.dot(rectified)+translation)
            scores.append(score); margins.append(margin)
        current = np.asarray(current, dtype=float)
        # 复核不能把搜索预算加倍；最终中心仍须落在原始search_x/y位移范围内。
        if np.any(np.abs(current-self.points) > np.array([self.search_x,self.search_y])):
            return {'valid':False, 'reason':'rotation_recheck_outside_search'}
        transform, info = similarity_from_points(self.points, current, self.max_residual)
        if transform is None:
            return {'valid':False, 'reason':'rotation_recheck_'+info['reason']}
        # 此处之前不设置self.transform；update开头已清空旧状态。
        self.transform = transform
        return dict(valid=True, reason='ok', min_score=min(scores), min_margin=min(margins),
                    residual_px=info['residual_px'], scale=info['scale'],
                    image_rotation_deg=info['image_rotation_deg'],
                    reference_points=self.points.tolist(), current_points=current.tolist(),
                    rotation_rechecked=True)

    def to_reference(self, x, y):
        if self.transform is None or not math.isfinite(x) or not math.isfinite(y):
            return None
        inverse, translation = self.transform
        point = inverse.dot(np.array([x,y])-translation)
        return float(point[0]), float(point[1])


def map_measurement(reference, out, candidates):
    if not out['valid'] or out.get('predicted',False):
        return None
    selected=[c for c in candidates if c.x==out['x_px'] and c.method==out['method']]
    if len(selected)!=1 or 'measurement_y_px' not in selected[0].extra:
        return None
    return reference.to_reference(out['x_px'],selected[0].extra['measurement_y_px'])



def similarity_from_points(reference, current, max_residual_px=2.5):
    """对已确认的对应点复用相同相似变换数学和保护阈值，不执行新的图像匹配。

    仅用于点矫正后重新估计变换；不是共线单应性或机械倾角估计。
    """
    a=np.asarray(reference,dtype=np.float64);b=np.asarray(current,dtype=np.float64)
    if a.ndim!=2 or a.shape[1]!=2 or a.shape!=b.shape or len(a)<3 or not np.isfinite(a).all() or not np.isfinite(b).all():
        return None,{'valid':False,'reason':'invalid_reference_points'}
    center=a.mean(axis=0);u=a-center;v=b-b.mean(axis=0);denominator=float((u*u).sum())
    if denominator<=1e-9:return None,{'valid':False,'reason':'degenerate_reference_points'}
    aa=float((u*v).sum()/denominator);bb=float((u[:,0]*v[:,1]-u[:,1]*v[:,0]).sum()/denominator)
    scale=math.hypot(aa,bb);angle=math.degrees(math.atan2(bb,aa));matrix=np.array([[aa,-bb],[bb,aa]])
    translation=b.mean(axis=0)-matrix.dot(center)
    residual=float(np.max(np.linalg.norm(a.dot(matrix.T)+translation-b,axis=1)))
    info=dict(valid=False,reason='inconsistent_reference',scale=scale,image_rotation_deg=angle,residual_px=residual)
    if not .9<=scale<=1.1 or abs(angle)>5 or residual>max_residual_px:return None,info
    info.update(valid=True,reason='ok')
    return (matrix.T/(scale*scale),translation),info
