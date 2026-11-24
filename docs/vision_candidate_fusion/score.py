"""Score frozen A/B/C/D/E replay logs against independent image labels on capture IDs."""
import argparse
import csv
import hashlib
import math
from collections import Counter, defaultdict
from itertools import combinations
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = Path("/home/user/桌面/jetson_nano/8")
TRUTH = HERE.parent / "continuity_20261122/truth"
MODES = "ABCDE"
DEV = ("fixed_m5_final", "parked_final")
HELD = "moving_final"
LABELS = ("inside", "outside", "border", "uncertain")
TIMES = ("ref_ms", "hough_ms", "extra_ms", "total_ms")
TOL = 12
SPEED = ("slow", "medium", "fast", "unknown")
FIELDS = "frame timestamp processed mode pixel_valid x_px y_px confidence candidates tracker_reason source rail_valid".split() + list(TIMES)


def rows(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def indexed(path):
    data = rows(path)
    index = {int(r["frame"]): r for r in data}
    if len(data) != len(index):
        raise ValueError(f"{path}: duplicate frame")
    return index


def sha(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def finite(x):
    value = float(x)
    if not math.isfinite(value):
        raise ValueError(f"nonfinite value {x}")
    return value


def flag(x):
    return {"0": False, "1": True, "false": False, "true": True}[x.strip().lower()]


def clock_check(f, timestamp, clocks):
    if f not in clocks or abs(finite(timestamp) - clocks[f]) > 1e-6:
        raise ValueError(f"capture ID or timestamp mismatch at frame {f}")


def quant(xs, percentile):
    return round(float(np.percentile(xs, percentile)), 4) if xs else None


def stats(xs):
    return dict(n=len(xs), mean=round(sum(xs)/len(xs), 4) if xs else None,
                p50=quant(xs, 50), p95=quant(xs, 95), max=round(max(xs), 4) if xs else None)


def save(path, data, fields=None):
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields or list(data[0]))
        writer.writeheader()
        writer.writerows(data)


def raw_inputs(scene, runs, truth_dir, log_path, all_frames, modes):
    base = runs / scene
    paths = dict(capture=base/"frames.csv", schedule=base/"measurements.csv",
                 truth=truth_dir/(scene+".csv"), log=log_path, video=base/"raw.mjpg")
    clocks = {f: finite(r["read_s"]) for f, r in indexed(paths["capture"]).items()}
    if list(clocks) != list(range(len(clocks))) or any(clocks[f] <= clocks[f-1] for f in list(clocks)[1:]):
        raise ValueError(f"{scene}: incomplete or nonmonotone capture clock")
    schedule = indexed(paths["schedule"])
    for f, r in schedule.items():
        clock_check(f, r["timestamp"], clocks)
    truth = indexed(paths["truth"])
    for f, r in truth.items():
        clock_check(f, r["read_s"], clocks)
    logs = defaultdict(dict)
    with log_path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        if not set(FIELDS) <= set(reader.fieldnames or []):
            raise ValueError(f"{log_path}: missing columns {set(FIELDS)-set(reader.fieldnames or [])}")
        for r in reader:
            f, mode = int(r["frame"]), r["mode"]
            clock_check(f, r["timestamp"], clocks)
            if mode not in tuple(modes) or f in logs[mode]:
                raise ValueError(f"{log_path}: unexpected or repeated {mode}/{f}")
            if flag(r["processed"]) and not all_frames and f not in schedule:
                raise ValueError(f"{log_path}: processed off-schedule frame {f}")
            if not flag(r["processed"]) and r["pixel_valid"] and flag(r["pixel_valid"]):
                raise ValueError(f"{log_path}: published skipped frame {f}")
            logs[mode][f] = r
    if set(logs) != set(modes):
        raise ValueError(f"{log_path}: missing mode")
    return paths, clocks, set(clocks if all_frames else schedule), truth, logs


def independent_speed(truth, clocks, f):
    # Symmetric 2-frame stencil; central difference in a ruler-relative horizontal coordinate.
    a, b = max(0, f-1), min(len(clocks)-1, f+1)
    position = []
    for i in (a, f, b):
        r = truth.get(i, {})
        if not all(r.get(k) for k in ("center_x_px", "roi_left_px", "roi_right_px")) or r.get("label") not in LABELS[:3]:
            return None, "unknown"
        position.append(finite(r["center_x_px"]) - (finite(r["roi_left_px"])+finite(r["roi_right_px"]))/2)
    if a == b:
        return None, "unknown"
    speed = abs(position[-1] - position[0]) / (clocks[b] - clocks[a])
    return speed, "slow" if speed < 50 else "medium" if speed < 200 else "fast"


def frame_rows(scene, clocks, scheduled, truth, logs):
    result = []
    for f, t in clocks.items():
        tr = truth.get(f, {})
        label = tr.get("label") or "uncertain"
        if label == "unknown":
            label = "uncertain"
        if label not in LABELS:
            raise ValueError(f"unexpected label {label}")
        speed, band = independent_speed(truth, clocks, f)
        for mode in sorted(logs):
            obs = logs[mode].get(f, {})
            processed = bool(obs) and flag(obs["processed"])
            state = ("processed" if processed else "skipped") if obs else "missing"
            valid = processed and flag(obs["pixel_valid"])
            x, y = (finite(obs["x_px"]), finite(obs["y_px"])) if valid else (None, None)
            dx = dy = dist = None
            if valid and tr.get("center_x_px") and tr.get("center_y_px"):
                dx, dy = x - finite(tr["center_x_px"]), y - finite(tr["center_y_px"])
                dist = math.hypot(dx, dy)
            result.append(dict(scene=scene, frame=f, timestamp=t, scheduled=f in scheduled,
                               mode=mode, state=state, label=label, center_x=tr.get("center_x_px"),
                               center_y=tr.get("center_y_px"), roi_left=tr.get("roi_left_px"),
                               roi_right=tr.get("roi_right_px"), speed_px_s=speed, speed_group=band,
                               valid=valid, x=x, y=y, dx=dx, dy=dy, error=dist,
                               correct=dist is not None and dist <= TOL,
                               horizontal_correct=dx is not None and abs(dx) <= TOL,
                               **{k: obs.get(k, "") for k in ("confidence","candidates","source","tracker_reason","rail_valid",*TIMES)}))
    return result


def describe(data, **key):
    valid = [r for r in data if r["valid"]]
    errors = [r["error"] for r in valid if r["error"] is not None]
    xerr = [abs(r["dx"]) for r in valid if r["dx"] is not None]
    states = Counter(r["state"] for r in data)
    return dict(**key, frames=len(data), **{k:states[k] for k in ("processed","missing","skipped")},
                valid=len(valid), invalid=states["processed"]-len(valid),
                correct=sum(r["correct"] for r in valid), over12=sum(e>TOL for e in errors),
                horizontal_correct=sum(r["horizontal_correct"] for r in valid),
                no_center=len(valid)-len(errors), **{"error_"+k:v for k,v in stats(errors).items()},
                abs_dx_mean=stats(xerr)["mean"], abs_dx_p95=quant(xerr,95))


def paired(data, scene, modes):
    index = {m:{r["frame"]:r for r in data if r["mode"] == m} for m in modes}
    report = []
    for a, b in combinations(modes, 2):
        for label in LABELS:
            left = [r for r in index[a].values() if r["label"] == label]
            pairs = [(r,index[b][r["frame"]]) for r in left]
            common = [(r,s) for r,s in pairs if r["error"] is not None and s["error"] is not None]
            delta = [s["error"] - r["error"] for r,s in common]
            report.append(dict(scene=scene, label=label, first=a, second=b, frames=len(left),
                               gained=sum(s["correct"] and not r["correct"] for r,s in pairs),
                               lost=sum(r["correct"] and not s["correct"] for r,s in pairs),
                               valid_gained=sum(s["valid"] and not r["valid"] for r,s in pairs),
                               valid_lost=sum(r["valid"] and not s["valid"] for r,s in pairs),
                               common=len(common), first_mean=stats([r["error"] for r,s in common])["mean"],
                               second_mean=stats([s["error"] for r,s in common])["mean"],
                               first_p95=quant([r["error"] for r,s in common],95),
                               second_p95=quant([s["error"] for r,s in common],95),
                               mean_paired_delta=stats(delta)["mean"], median_paired_delta=quant(delta,50),
                               mean_paired_abs_dx_delta=stats([abs(s["dx"])-abs(r["dx"]) for r,s in common])["mean"]))
    return report


def reentries(scene, mode, data):
    # Outside, optional border, first inside. Unknown breaks evidence. Entire raw axis defines events.
    events, outside, active = [], False, None
    for r in data:
        label = r["label"]
        if active is not None and label != "inside":
            active["status"] = "censored_"+label
            active = None
        if label == "inside":
            if outside:
                active = dict(scene=scene, mode=mode, first_inside=r["frame"], end=r["frame"],
                              opportunities=0, first_correct=None, status="censored_end", latency_ms=None, observed_ms=0.)
                events.append(active)
                onset = r["timestamp"]
            outside = False
            if active is not None:
                active["end"] = r["frame"]
                active["observed_ms"] = round(1000*(r["timestamp"]-onset),3)
                active["opportunities"] += int(r["scheduled"])
                if r["scheduled"] and r["correct"]:
                    active.update(first_correct=r["frame"], status="observed", latency_ms=active["observed_ms"])
                    active = None
        elif label != "border":
            outside = label == "outside"
    return events


def timings(scene, mode, data):
    report = []
    for k in TIMES:
        values = [finite(r[k]) for r in data if r["state"]=="processed" and r[k]!=""]
        if any(v < 0 for v in values):
            raise ValueError(f"negative timing {scene}/{mode}/{k}")
        report.append(dict(scene=scene,mode=mode,stage=k,**stats(values),p99=quant(values,99)))
    return report


def audit(scene, data, paths, truth_dir, out, manual, modes):
    truth_sha = sha(paths["truth"])
    old = truth_dir / "manual_identity.csv"
    decisions = {}
    for source in (old, manual):
        if source and source.is_file():
            for r in rows(source):
                if r["scene"] == scene and r["truth_sha256"] == truth_sha:
                    key = (int(r["frame"]), int(r["output_x_round_px"]), int(r["output_y_round_px"]))
                    decisions[key] = r["manual_identity"]

    groups = defaultdict(list)
    for r in data:
        if r["valid"] and (r["label"] != "inside" or not r["correct"]):
            groups[r["frame"], round(r["x"]), round(r["y"])].append(r)
    by_frame = defaultdict(dict)
    for r in data:
        by_frame[r["frame"]][r["mode"]] = r
    packets = paths["video"].read_bytes().split(b"\xff\xd8\xff\xe0")[1:]
    if len(packets) != len(indexed(paths["capture"])):
        raise ValueError(f"{scene}: JPEG count mismatch")

    def decode(frame):
        buf = np.frombuffer(b"\xff\xd8\xff\xe0"+packets[frame], np.uint8)
        image = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f"{scene}: unreadable frame {frame}")
        return image

    def tile(r, caption):
        original = decode(r["frame"])
        x, y = round(r["x"]), round(r["y"])
        tx, ty = (round(float(r["center_x"])),round(float(r["center_y"]))) if r["center_x"] else (x,y)
        annotated = original.copy()
        if r["center_x"]:
            cv2.drawMarker(annotated, (tx,ty), (60,250,60), cv2.MARKER_CROSS,9,1)
        cv2.drawMarker(annotated, (x,y), (255,40,245), cv2.MARKER_TILTED_CROSS,9,1)
        panels = [cv2.getRectSubPix(im,(76,76),(float(cx),float(cy)))
                  for im,cx,cy in [(original,tx,ty),(annotated,tx,ty),(annotated,x,y)]]
        t = cv2.copyMakeBorder(np.hstack(panels),0,20,0,0,cv2.BORDER_CONSTANT)
        cv2.putText(t,caption,(3,90),0,.42,(255,255,255),1,cv2.LINE_AA)
        return t

    def sheet(path, tiles):
        blank = np.zeros_like(tiles[0])
        pads = tiles + [blank]*((-len(tiles))%4)
        if not cv2.imwrite(str(path), np.vstack([np.hstack(pads[i:i+4]) for i in range(0,len(pads),4)])):
            raise OSError(path)

    output, tiles = [], []
    for idx, ((frame,x,y), obs) in enumerate(sorted(groups.items())):
        file = f"{scene}_audit_{idx//40+1:02}.jpg"
        decision = decisions.get((frame,x,y), "unreviewed")
        for r in obs:
            output.append(dict(scene=scene,frame=frame,mode=r["mode"],source=r["source"],label=r["label"],
                               x=r["x"],y=r["y"],error=r["error"],output_x_round_px=x,
                               output_y_round_px=y,manual_identity=decision,truth_sha256=truth_sha,
                               sheet=file,tile=idx%40))
        r = obs[0]
        tiles.append(tile(r,f"#{frame} {''.join(v['mode'] for v in obs)} {r['label']} ({x},{y})"))
        if len(tiles)==40 or idx==len(groups)-1:
            sheet(out/file,tiles)
            tiles = []
    # All frames with a different published position or validity. Each distinct current
    # position gets a tile, with A–E attribution and the measured distance to truth.
    tiles = []
    for frame, observations in sorted(by_frame.items()):
        centers = defaultdict(list)
        for m in modes:
            r = observations[m]
            centers[(r["x"], r["y"]) if r["valid"] else None].append(m)
        if len(centers) == 1:
            continue
        for xy, names in centers.items():
            if xy is None:
                continue
            r = observations[names[0]]
            error = f"{r['error']:.1f}px" if r["error"] is not None else "?"
            names = "".join(names)
            caption = f"#{frame} {names} {r['source']} {error}"
            tiles.append(tile(r, caption))
    for start in range(0, len(tiles), 40):
        name = scene+"_paired.jpg" if start == 0 else f"{scene}_paired_{start//40+1:02}.jpg"
        sheet(out/name,tiles[start:start+40])
    return output


def report(out, summaries, pairs, speeds, events, timing, review, all_frames, modes):
    lines = ["# 候选融合逐帧评分", "",
             "按全部采集帧独立回放。" if all_frames else "按现场 measurements.csv 指定的处理帧评分。原始时轴的每个模式和帧号保存在 frames.csv。",
             "A 为现有输出，B 加轮廓补测，C 加运动补测，D 合并两类补测。E 在 D 的多候选竞争帧上使用近期实测轨迹关联。"
             if "E" in modes else "A 为现有输出，B 加轮廓补测，C 加运动补测，D 合并两类补测。",
             "开发场景为 fixed_m5_final、parked_final；moving_final 为本次冻结后评估的留出场景。",
             "球心采用原图二维欧氏误差 ≤12 px。另列水平绝对误差 |Δx|≤12 px。内外标签沿用原始真值的水平搜索边界、有符号裕量和边界带，边界及不确定帧单列。",
             "fixed_m5_final 从第 410 帧起仅有实际像素位置真值。距离误差均为原始像素。", "",
             "| 场景 | 模式 | 界内正确 / 界内 | 界内超差 | 水平正确 / 界内 | 界外有效 / 界外 | 边界有效 / 边界 | 不确定有效 / 不确定 |",
             "|---|---|---:|---:|---:|---:|---:|---:|"]
    get = {(r["scene"],r["mode"],r["label"]):r for r in summaries}
    scenes = sorted({r["scene"] for r in summaries})
    for scene in scenes:
        for m in modes:
            a,*rest = [get[scene,m,lab] for lab in LABELS]
            row = [scene,m,f"{a['correct']}/{a['frames']}",a["over12"],f"{a['horizontal_correct']}/{a['frames']}"]
            row += [f"{r['valid']}/{r['frames']}" for r in rest]
            lines.append("| "+" | ".join(map(str,row))+" |")
    lines += ["", "## 同帧成对消融", "",
              "配对误差只用两模式均有效且有可读真值的明确界内帧。新增和丢失均基于同一现场或全帧日程中的二维正确输出。",
              "", "| 场景 | 模式 | 新增 / 丢失 | 共同帧 | 前 / 后平均二维误差 px | 后 − 前的配对均差 px |",
              "|---|---|---:|---:|---|---:|"]
    for r in pairs:
        if r["label"]=="inside":
            lines.append(f"| {r['scene']} | {r['first']}→{r['second']} | {r['gained']}/{r['lost']} | {r['common']} | {r['first_mean']} / {r['second_mean']} | {r['mean_paired_delta']} |")
    lines += ["","## 速度、回入和耗时","",
              "速度从原始时轴独立标注的球心 x 减去当帧两条尺界的中点计算，取相邻原始帧的中心差分绝对值，端点取单边差分。慢速 <50、中速 50–<200、快速 ≥200 px/s。未知不并入慢速。下表速度列为界内二维正确数/该组界内数。",
              "真球明确界外后可经过边界带，首次明确界内的原始时间为回入起点。连续界内期间首个二维正确输出结束计时；再次离开明确界内或片段结束时，在最后一个明确界内帧右删失。缺行和跳帧不产生成功事件。耗时取离线回放每模式的阶段用时合计，含解码、参考、检测、跟踪和坐标处理。",
              "", "| 场景 | 模式 | 慢 | 中 | 快 | 回入成功/事件 | 成功延迟中位 ms | 总耗时中位/P95 ms |",
              "|---|---|---:|---:|---:|---:|---:|---|"]
    for scene in scenes:
        for m in modes:
            group=[r for r in speeds if r["scene"]==scene and r["mode"]==m and r["label"]=="inside"]
            ev=[r for r in events if r["scene"]==scene and r["mode"]==m]
            delays=[r["latency_ms"] for r in ev if r["status"]=="observed"]
            tt=next(r for r in timing if r["scene"]==scene and r["mode"]==m and r["stage"]=="total_ms")
            row=[scene,m]+[f"{r['correct']}/{r['frames']}" for r in group if r["speed_group"] in SPEED[:3]]
            row += [f"{len(delays)}/{len(ev)}",str(quant(delays,50)) if delays else "—",f"{tt['p50']}/{tt['p95']}"]
            lines.append("| "+" | ".join(row)+" |")
    lines += ["","## 异常标记的原片身份核对","",
              "每个有效的非明确界内输出及界内超差输出均进入 review.csv。图块从左至右为真值处原片、真值处叠加、输出处叠加。绿色十字标真值，紫色叉标输出。*_paired.jpg 汇集同帧有效位或坐标不同的实测标记，图注给出帧号、发布模式、来源和误差。既有人工记录按真值哈希、帧号及四舍五入的原图坐标匹配。",
              "", "| 场景 | 模式 | 球体或拖影 | 确认错物 | 身份有疑 | 待看原片 |","|---|---|---:|---:|---:|---:|"]
    pending = {(r["scene"],r["frame"],r["output_x_round_px"],r["output_y_round_px"])
               for r in review if r["manual_identity"] == "unreviewed"}
    prefix = "全帧" if all_frames else "现场处理日程"
    lines[-3:-3] = [f"{prefix}还有 {len(pending)} 个独立图像落点待目视判读。" if pending
                    else f"{prefix}全部异常落点已有原片判读。"]
    for scene in scenes:
        for m in modes:
            counts=Counter(r["manual_identity"] for r in review if r["scene"]==scene and r["mode"]==m)
            ball,wrong,todo=[counts[k] for k in ("on_ball_or_blurred_rim","wrong_object","unreviewed")]
            lines.append(f"| {scene} | {m} | {ball} | {wrong} | {sum(counts.values())-ball-wrong-todo} | {todo} |")
    lines += ["","逐项明细见 summary.csv、pairs.csv、speed.csv、reentry.csv、timing.csv、review.csv；输入哈希见 inputs.csv。"]
    (out/"report.md").write_text("\n".join(lines)+"\n")


REVIEW_FIELDS = "scene frame mode source label x y error output_x_round_px output_y_round_px manual_identity truth_sha256 sheet tile".split()
EVENT_FIELDS = "scene mode first_inside end opportunities first_correct status latency_ms observed_ms".split()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--logs", type=Path, default=ROOT/"fusion_20261122/logs")
    p.add_argument("--runs", type=Path, default=ROOT/"continuity_20261122/runs")
    p.add_argument("--truth", type=Path, default=TRUTH)
    p.add_argument("--out", type=Path, help="result directory; defaults separated by schedule and heldout")
    p.add_argument("--all-frames", action="store_true", help="score all_frames logs separately")
    p.add_argument("--include-heldout-after-freeze", action="store_true",
                   help="include moving_final only after external freeze confirmation")
    p.add_argument("--modes", choices=("ABCD", "ABCDE"), default=MODES,
                   help="use ABCD only for historical four-mode development logs")
    p.add_argument("--manual", type=Path, default=HERE/"results"/"manual_identity.csv", help="additional image-reviewed identities, same keys as manual_identity.csv")
    args=p.parse_args()
    modes=args.modes
    if args.include_heldout_after_freeze and modes != MODES:
        p.error("frozen comparison requires ABCDE")
    scenes=(*DEV,HELD) if args.include_heldout_after_freeze else DEV
    schedule="all_frames" if args.all_frames else "processed"
    out=args.out or HERE/"results"/("frozen" if args.include_heldout_after_freeze else "development")/schedule
    out.mkdir(parents=True, exist_ok=True)
    summary, speed, pairs, events, timing, review, frames, inputs = [], [], [], [], [], [], [], []
    for scene in scenes:
        log_root=args.logs
        if args.all_frames:
            log_root=log_root/"all_frames" if (log_root/"all_frames").is_dir() else log_root.parent/"all_frames"
        paths,clocks,scheduled,truth,logs=raw_inputs(scene,args.runs,args.truth,log_root/(scene+".csv"),args.all_frames,modes)
        data=frame_rows(scene,clocks,scheduled,truth,logs)
        frames.extend(data)
        selected=[r for r in data if r["scheduled"]]
        pairs.extend(paired(selected,scene,modes))
        for m in modes:
            group=[r for r in selected if r["mode"]==m]
            for label in LABELS:
                items=[r for r in group if r["label"]==label]
                summary.append(describe(items,scene=scene,mode=m,label=label))
                for b in SPEED:
                    speed.append(describe([r for r in items if r["speed_group"]==b],
                                          scene=scene,mode=m,label=label,speed_group=b))
            timing.extend(timings(scene,m,group))
            events.extend(reentries(scene,m,[r for r in data if r["mode"]==m]))
        review.extend(audit(scene,selected,paths,args.truth,out,args.manual,modes))
        for k,path in paths.items():
            inputs.append(dict(scene=scene,kind=k,path=str(path),sha256=sha(path)))
    for k,path in (("scorer",Path(__file__)),("prior_review",args.truth/"manual_identity.csv"),("review",args.manual)):
        if path and path.is_file():
            inputs.append(dict(scene="*",kind=k,path=str(path),sha256=sha(path)))
    for name,table in [("summary",summary),("frames",frames),("speed",speed),("pairs",pairs),
                       ("timing",timing),("inputs",inputs)]:
        save(out/(name+".csv"),table)
    save(out/"reentry.csv",events,EVENT_FIELDS)
    save(out/"review.csv",review,REVIEW_FIELDS)
    report(out,summary,pairs,speed,events,timing,review,args.all_frames,modes)
    print(out/"report.md")


if __name__ == "__main__":
    main()
