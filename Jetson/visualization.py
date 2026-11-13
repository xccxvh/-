# -*- coding: utf-8 -*-
"""实时可视化叠加。

显示: 轨道检测区域、轨道上缘、钢珠候选、最终位置、厘米坐标、
处理帧率、识别状态。
"""
import cv2

STATE_COLORS = {
    "DETECTED": (0, 255, 0),
    "UNCERTAIN": (0, 165, 255),
    "LOST": (0, 0, 255),
}


def _panel(vis, lines, origin=(10, 10), scale=1.0):
    x, y = [int(v*scale) for v in origin]
    line_h = max(1, int(26*scale))
    font = .62*scale
    thickness = max(1, int(round(2*scale)))
    width = 0
    for text, _ in lines:
        (tw, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font, thickness)
        width = max(width, tw)
    height = line_h * len(lines) + int(14*scale)
    # Only the panel changes; blending the entire frame needlessly copies
    # and processes 1280x720 pixels for this small rectangle.
    x0, y0 = max(0, x - int(6*scale)), max(0, y - int(6*scale))
    x1 = min(vis.shape[1], x + width + int(10*scale) + 1)
    y1 = min(vis.shape[0], y + height + 1)
    if x1 > x0 and y1 > y0:
        roi = vis[y0:y1, x0:x1]
        cv2.addWeighted(roi, 0.45, roi, 0.0, 0, roi)
    for i, (text, color) in enumerate(lines):
        cv2.putText(vis, text, (x, y + int(22*scale) + i * line_h),
                    cv2.FONT_HERSHEY_SIMPLEX, font, color, thickness, cv2.LINE_AA)


def draw_overlay(frame, rail, candidates, track, fps, algo_ms,
                 x_cm=None, calibrated=False, show_band=True, output_size=None):
    source_h, source_w = frame.shape[:2]
    vis = (frame.copy() if output_size is None or output_size == (source_w,source_h)
           else cv2.resize(frame, output_size, interpolation=cv2.INTER_AREA))
    H, W = vis.shape[:2]
    sx, sy = W/float(source_w), H/float(source_h)
    scale = min(sx, sy)
    thickness = max(1, int(round(2*scale)))

    if rail is not None and rail.valid:
        y0 = int(rail.y_at(0)*sy)
        y1 = int(rail.y_at(source_w - 1)*sy)
        cv2.line(vis, (0, y0), (W - 1, y1), (0, 255, 0), thickness)
        if show_band:
            up = int(0.05 * H)
            down = int(0.012 * H)
            cv2.line(vis, (0, y0 - up), (W - 1, y1 - up), (0, 110, 0), 1)
            cv2.line(vis, (0, y0 + down), (W - 1, y1 + down), (0, 110, 0), 1)

    for c in (candidates or [])[:5]:
        yl = int(rail.y_at(c.x)*sy) if (rail is not None and rail.valid) else H // 2
        if c.method == "color":
            color = (0, 255, 255)
        elif c.method == "geom":
            color = (255, 255, 0)
        else:
            color = (0, 255, 0)
        x1 = int((c.x - c.width / 2.0)*sx)
        x2 = int((c.x + c.width / 2.0)*sx)
        y1 = max(0, int(yl - c.height*sy))
        cv2.rectangle(vis, (x1, y1), (x2, yl), color, thickness)

    state = track.get("state", "LOST") if track else "LOST"
    color = STATE_COLORS.get(state, (255, 255, 255))
    if track and track.get("x_px") is not None:
        cx = int(max(0, min(W - 1, track["x_px"]*sx)))
        cy = int(rail.y_at(track["x_px"])*sy) if (rail is not None and rail.valid) else H // 2
        cy = int(max(0, min(H - 1, cy)))
        if track.get("predicted"):
            cv2.drawMarker(vis, (cx, cy), color, cv2.MARKER_TRIANGLE_UP, max(1,int(20*scale)), thickness)
        else:
            cv2.drawMarker(vis, (cx, cy), color, cv2.MARKER_CROSS, max(1,int(28*scale)), thickness)
            cv2.circle(vis, (cx, cy), max(1,int(13*scale)), color, thickness)

    x_px = track.get("x_px") if track else None
    conf = track.get("confidence", 0.0) if track else 0.0
    method = (track.get("method") if track else None) or "-"
    if x_px is None:
        x_px_text = "--"
    else:
        x_px_text = "%.1f" % x_px
    if x_cm is None:
        x_cm_text = "-- (%s)" % track["cm_status"] if track and track.get("cm_status") else "--" + ("" if calibrated else " (uncalibrated)")
    else:
        x_cm_text = "%+.2f cm" % x_cm
        if track and track.get("reference_preview"):
            x_cm_text += " PREVIEW"

    lines = [
        ("state: %s" % state, color),
        ("x_px: %s" % x_px_text, (255, 255, 255)),
        ("x_cm: %s" % x_cm_text, (255, 255, 255)),
        ("conf: %.2f  method: %s" % (conf, method), (200, 200, 200)),
        ("FPS: %.1f  algo: %.1f ms" % (fps, algo_ms), (200, 200, 200)),
    ]
    _panel(vis, lines, scale=scale)
    return vis
