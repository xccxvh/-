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
                 rotation_recheck=False, adaptive_search=False):
        if gray is None or gray.ndim != 2 or gray.dtype != np.uint8:
            raise ValueError('reference must be uint8 grayscale')
        # 两个开关独立且只接受真正的布尔值；旧配置默认逐帧固定窗匹配。
        if type(rotation_recheck) is not bool:
            raise ValueError('rotation_recheck must be boolean')
        if type(adaptive_search) is not bool:
            raise ValueError('adaptive_search must be boolean')
        self.rotation_recheck = rotation_recheck
        self.adaptive_search = adaptive_search
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
        self.transform = None

    def update(self, gray):
        # 上一姿态仅用于安排本帧搜索。分辨率变化、遮挡或几何失败时，
        # 对外映射立即失效，下一帧从参考位置独立重定位。
        previous = self.transform
        self.transform = None
        if gray is None or gray.shape != self.shape or gray.dtype != np.uint8:
            return {'valid': False, 'reason': 'resolution_or_format'}
        if self.adaptive_search and previous is not None:
            inverse, translation = previous
            predicted = self.points.dot(np.linalg.inv(inverse).T) + translation
            offsets = np.rint(predicted - self.points).astype(int)
            result = self._match(gray, offsets)
            if result['valid']:
                result['registration_mode'] = 'track'
                return result

        # 小窗失败后在同一帧检查完整配置搜索域。旋转提议及复核继续使用
        # 原图模板和原阈值，禁止将历史姿态直接当作本帧观测。
        result = self._match(gray)
        if self.rotation_recheck and result['reason'] == 'weak_or_ambiguous_patch':
            result = self._rotation_recheck(gray)
        if self.adaptive_search and result['valid']:
            result['registration_mode'] = 'relocalize'
        return result

    def _match(self, gray, offsets=None):
        """在原图像坐标下匹配各标记，所有模板通过后才发布相似变换。"""
        points, scores, margins = [], [], []
        H, W = self.shape
        for i, (x, y, w, h, patch) in enumerate(self.patches):
            x0, x1 = max(0, x-self.search_x), min(W, x+w+self.search_x)
            y0, y1 = max(0, y-self.search_y), min(H, y+h+self.search_y)
            # 第一个模板始终检查整个位移范围的竞争峰。当另一把相同的尺子
            # 出现在小窗之外时，仅用历史姿态无法确认身份，必须回到完整复核。
            if offsets is not None and i > 0:
                # 12 像素是帧间小窗半径，包含峰值外的竞争刻度。与绝对
                # 搜索域取交集，避免连续小位移使匹配位置漂出重定位范围。
                dx, dy = offsets[i]
                x0, x1 = max(x0, x+dx-12), min(x1, x+dx+w+12)
                y0, y1 = max(y0, y+dy-12), min(y1, y+dy+h+12)
                if x1-x0 < w or y1-y0 < h:
                    return {'valid': False, 'reason': 'outside_search'}
            score, margin, (xx, yy) = self._peak(gray[y0:y1, x0:x1], patch)
            if (not math.isfinite(score) or not math.isfinite(margin) or
                    score < self.min_score or margin < self.min_margin):
                return {'valid': False, 'reason': 'weak_or_ambiguous_patch'}
            points.append([x0+xx+w/2.0, y0+yy+h/2.0])
            scores.append(score)
            margins.append(margin)

        transform, info = similarity_from_points(self.points, points, self.max_residual)
        if transform is None:
            return {'valid': False, 'reason': info['reason']}
        self.transform = transform
        return dict(info, min_score=min(scores), min_margin=min(margins),
                    reference_points=self.points.tolist(), current_points=points)

    @staticmethod
    def _peak(window, patch, eligible=None):
        """只提取原模板峰值；完整竞争范围与原2像素局部峰排除规则一致。"""
        response = cv2.matchTemplate(window, patch, cv2.TM_CCOEFF_NORMED)
        if eligible is not None:
            # 补边参与的窗口不可作为最佳峰或竞争峰；仅接受完整实测纹理。
            response[~eligible] = -1
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

    def _visible_rectified_search(self, gray, box, matrix, translation):
        """底边附近仅剔除采样不完整的位置，保留大窗内全部可见竞争峰。"""
        x, y, w, h, _ = box
        H, W = self.shape
        half = np.array([w/2., h/2.])
        center = np.array([x, y]) + half
        radius = np.array([self.search_x, self.search_y])
        # 仿射模板四角相对中心的偏移决定了完整采样的中心范围。先和
        # 配置的大窗取交集，故配置半径即使大于画面也不会分配超大窗。
        corners = np.array([[0, 0], [w-1, 0], [0, h-1], [w-1, h-1]])
        footprint = (corners - half).dot(matrix.T)
        low = np.maximum(center-radius, -footprint.min(axis=0))
        high = np.minimum(center+radius, [W-1, H-1]-footprint.max(axis=0))
        if np.any(high < low):
            return None
        vertices = np.array([low, [high[0], low[1]], [low[0], high[1]], high])
        top_left = (vertices-translation).dot(np.linalg.inv(matrix).T) - half
        origin = np.floor(top_left.min(axis=0)).astype(int)
        nx, ny = np.ceil(top_left.max(axis=0)).astype(int) - origin + 1

        # 矩形外接窗旋转后多出四个角；逐候选检查模板中心，阻止这些
        # 位置的黑边或超预算位移影响相关性排名。每个真值均对应完整模板。
        yy, xx = np.ogrid[:ny, :nx]
        mapped_center = matrix.dot(origin + half) + translation
        cx = mapped_center[0] + matrix[0, 0]*xx + matrix[0, 1]*yy
        cy = mapped_center[1] + matrix[1, 0]*xx + matrix[1, 1]*yy
        eligible = (cx >= low[0]) & (cx <= high[0]) & (cy >= low[1]) & (cy <= high[1])
        if not eligible.any():
            return None
        affine = np.column_stack((matrix, matrix.dot(origin) + translation))
        window = cv2.warpAffine(gray, affine, (int(nx+w-1), int(ny+h-1)),
                                flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)
        return window, origin, eligible

    def _rotation_recheck(self, gray):
        """失败峰值仅提议姿态；每个完整窗通过原强度/歧义门限后才提交变换。"""
        H, W = self.shape
        proposed, proposal_scores = [], []
        # 原快速路径提前失败，此处重新收集全部patch（包括原失败patch）的峰值。
        for x, y, w, h, patch in self.patches:
            x0, y0 = x-self.search_x, y-self.search_y
            x1, y1 = x+w+self.search_x, y+h+self.search_y
            if self.adaptive_search:
                # 提议仅搜索图中可完整放置原模板的位置，保留真实左上角坐标。
                x0, y0 = max(0, x0), max(0, y0)
                x1, y1 = min(W, x1), min(H, y1)
            elif x0 < 0 or y0 < 0 or x1 > W or y1 > H:
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
            eligible = None
            if self.adaptive_search:
                search = self._visible_rectified_search(gray, box, matrix, translation)
                if search is None:
                    return {'valid':False, 'reason':'rotation_recheck_clipped'}
                window, origin, eligible = search
            else:
                search = self._rectified_search(gray, box, matrix, translation)
                if search is None:
                    return {'valid':False, 'reason':'rotation_recheck_clipped'}
                window, origin = search
            score, margin, (xx,yy) = self._peak(window, patch, eligible)
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

    原始匹配、旋转复核和点矫正共用同一拟合及残差判据。
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
