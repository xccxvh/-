"""Offline current-frame contour measurements on the 1280x720 top-view rig.

The reported center is a least-squares circle center of observed dark-to-light
body boundary samples. Horizontal motion integrates a range of positions over
the exposure. Its instantaneous endpoint cannot be recovered from this fit.
"""
import cv2
import numpy as np

from ball_detector import BallCandidate
from measurement_contract import TOPVIEW_CIRCLE_VERSION


_ANGLE = np.deg2rad(np.arange(0, 360, 7.5))
_NX, _NY = np.cos(_ANGLE), np.sin(_ANGLE)
_RADII = np.arange(18, 34)[:, None]
_DX = np.rint(_RADII * _NX).astype(int)
_DY = np.rint(_RADII * _NY).astype(int)
_VERTICAL = np.ones((11, 1), np.uint8)
_HORIZONTAL = np.ones((1, 7), np.uint8)


def _glint_pairs(patch, edge):
    # Short horizontal closing joins the two halves of a specular reflection
    # as they merge into one streak during an exposure.
    glint = cv2.morphologyEx(patch, cv2.MORPH_TOPHAT, _VERTICAL)
    depth = edge[None, :] - np.arange(patch.shape[0])[:, None]
    mask = np.uint8((glint > 32) & (depth >= 9) & (depth <= 58))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _HORIZONTAL)
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    rows = []
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if 7 <= w <= 58 and 3 <= h <= 16 and cv2.contourArea(contour) >= 17:
            rows.append((x + .5*(w-1), y + .5*(h-1), w))
    for x, y, w in rows:
        for u, v, width in rows:
            if (8 <= v-y <= 17 and abs(u-x) <= 18 and
                    max(w, width) >= 18 and min(w, width) >= .30*max(w, width)):
                yield round((w*x+width*u)/(w+width)), round(v-1)


def _body_circle(patch, seed_x, seed_y, edge):
    # Sobel on the original smoothed pixels. Ignore the straight rail lip,
    # and require the local gradient to face outwards at the proposed rim.
    half = 35
    tile = patch[seed_y-half:seed_y+half+1, seed_x-half:seed_x+half+1]
    if tile.shape != (2*half+1, 2*half+1):
        return None
    gx = cv2.Sobel(tile, cv2.CV_32F, 1, 0)[half+_DY, half+_DX]
    gy = cv2.Sobel(tile, cv2.CV_32F, 0, 1)[half+_DY, half+_DX]
    outward = gx*_NX + gy*_NY
    tangent = gy*_NX - gx*_NY
    eligible = ((outward >= 18) & (abs(tangent) <= .55*outward) &
                (seed_y+_DY < edge[seed_x+_DX]-3))
    peaks = np.where(eligible, outward, -1).argmax(axis=0)
    sectors = np.arange(len(_ANGLE))
    supported = eligible[peaks, sectors]
    dx, dy = _DX[peaks, sectors], _DY[peaks, sectors]
    # An axis-aligned tool can support four straight sides. Demand actual
    # oblique boundary normals distributed around the curved body as well.
    px, py = gx[peaks, sectors], gy[peaks, sectors]
    oblique = np.minimum(abs(px), abs(py)) > .35*np.maximum(abs(px), abs(py))
    for iteration in range(2):
        if (supported.sum() < 20 or
                min(np.count_nonzero(supported & (s*_NX > .65)) for s in (-1, 1)) < 4 or
                sum(np.any(supported & oblique & (sx*_NX > .32) & (sy*_NY > .32))
                    for sx in (-1, 1) for sy in (-1, 1)) < 3):
            return None
        xx, yy = dx[supported], dy[supported]
        system = np.column_stack((2*xx, 2*yy, np.ones(len(xx))))
        ox, oy, c = np.linalg.lstsq(system, xx*xx+yy*yy, rcond=None)[0]
        if c <= 0:
            return None
        radius = np.sqrt(c+ox*ox+oy*oy)
        error = abs(np.hypot(dx-ox, dy-oy)-radius)
        if iteration == 0:
            supported &= error <= 3.5
    if not 19 <= radius <= 31 or np.hypot(ox, oy) > 7 or np.median(error[supported]) > 2.2:
        return None
    # Four times the normal angle maps both perpendicular tool faces to the
    # same direction. A circular arc supplies a spread of normals even when
    # the lip obscures its bottom. This test also covers rotated rectangles.
    theta = np.arctan2(py[supported], px[supported])
    weights = np.hypot(px[supported], py[supported])
    concentration = np.hypot(np.average(np.cos(4*theta), weights=weights),
                             np.average(np.sin(4*theta), weights=weights))
    if concentration > .40:
        return None
    return seed_x+ox, seed_y+oy, radius, int(supported.sum())


def contour_candidates(gray, rail, bounds):
    """Return all spatially distinct pixel observations given current rail and x bounds."""
    if (not getattr(rail, 'valid', False) or not getattr(rail, 'identity_valid', False)
            or bounds is None or gray is None or gray.ndim != 2 or gray.dtype != np.uint8):
        return []
    height, width = gray.shape
    limits = np.asarray(bounds)
    profile = np.asarray(getattr(rail, 'edge_profile', None))
    support = np.asarray(getattr(rail, 'edge_support', None))
    if (limits.shape != (2,) or not np.isfinite(limits).all() or limits[0] >= limits[1]
            or profile.shape != (width,) or support.shape != (width,)):
        return []
    left = max(0, int(np.floor(limits[0]))-45)
    right = min(width, int(np.ceil(limits[1]))+46)
    if right-left < 71:
        return []
    edge = profile[left:right]
    if not np.isfinite(edge).all():
        return []
    top = max(0, int(np.floor(edge.min()))-90)
    bottom = min(height, int(np.ceil(edge.max()))+35)
    if bottom-top < 71:
        return []
    patch = cv2.GaussianBlur(gray[top:bottom, left:right], (5, 5), 1.2)
    edge = edge-top
    proposals = []
    for seed_x, seed_y in _glint_pairs(patch, edge):
        if not (35 <= seed_x < patch.shape[1]-35 and 35 <= seed_y < patch.shape[0]-35):
            continue
        circle = _body_circle(patch, seed_x, seed_y, edge)
        if circle is None:
            continue
        x, y, radius, count = circle
        cx, cy = float(x+left), float(y+top)
        if not limits[0] <= cx <= limits[1] or abs(profile[round(cx)]-cy-radius) > 14:
            continue
        a, b = int(cx-radius), int(cx+radius)
        if not (support[max(0, a-10):a].any() and support[b+1:b+11].any()):
            continue
        half = int(.47*radius)
        u, v = round(x), round(y)
        core = patch[v-half:v+half+1, u-half:u+half+1]
        appearance = float(np.percentile(core, 95)-np.median(core))
        if appearance < 35:
            continue
        extra = dict(measurement_y_px=cy, coordinate_version=TOPVIEW_CIRCLE_VERSION,
                     identity_valid=True, in_search_roi=True, appearance=appearance,
                     proposal_source='contour')
        proposals.append(BallCandidate(cx, 2*radius, 2*radius, .65+.30*count/len(_ANGLE),
                                       'topview_circle', extra))
    distinct = []
    for ball in sorted(proposals, key=lambda b: b.confidence, reverse=True):
        if all(np.hypot(ball.x-b.x, ball.extra['measurement_y_px']-b.extra['measurement_y_px'])
               > 12 for b in distinct):
            distinct.append(ball)
    return distinct
