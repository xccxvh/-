"""按原始帧号及采集时钟比较有效像素输出。只使用 Python 标准库，无配置或标定绑定。"""
import argparse
import csv
import hashlib
import math
from pathlib import Path


HERE = Path(__file__).resolve().parent
SCENES = ("moving_fast_confirmed", "parked_free", "moving_free", "fixed_O_tilt")
PROFILES = ("live", "baseline", "release_digits", "release_points")


def read_index(path):
    with path.open() as stream:
        rows = {}
        for row in csv.DictReader(stream):
            frame = int(row["frame"])
            if frame in rows:
                raise ValueError(f"{path}: duplicate frame {frame}")
            rows[frame] = row
        return rows


def number(value):
    n = float(value)
    if not math.isfinite(n):
        raise ValueError(f"non-finite value {value}")
    return n


def flag(value):
    if value.lower() not in {"true", "false", "0", "1"}:
        raise ValueError(f"invalid pixel_valid {value}")
    return value.lower() in {"true", "1"}


def aligned(rows, clocks, key, source):
    for f, row in rows.items():
        if f not in clocks or abs(number(row[key])-clocks[f]) > 1e-6:
            raise ValueError(f"{source}: frame {f} has mismatched capture ID or clock")


def point(row, x, y):
    if row.get(x, "") == "" or row.get(y, "") == "":
        return None
    return number(row[x]), number(row[y])


def score(scene, profile, view, clocks, truth, log, selected, tol, identity=None):
    # 全时轴迭代保证未选中的越界、边界或未知帧也会切断连续缺测。
    buckets = ("inside outside border unknown inside_correct inside_wrong inside_valid "
               "inside_invalid inside_skipped inside_unscorable outside_valid outside_on_ball "
               "outside_wrong outside_unscorable border_valid unknown_valid valid skipped "
               "flagged_identity reviewed_ball reviewed_wrong_object identity_unknown").split()
    scores = dict.fromkeys(buckets, 0)
    run = []
    episodes = []

    def finish():
        if run:
            episodes.append((len(run), clocks[run[-1]]-clocks[run[0]], run[0], run[-1]))
            run.clear()

    for f in clocks:
        tr = truth.get(f, {})
        label = tr.get("label", "unknown")
        if label in {"uncertain", "unknown", ""}:
            label = "unknown"
        if label not in {"inside", "outside", "border", "unknown"}:
            raise ValueError(f"unrecognized truth label {label}")
        center = point(tr, "center_x_px", "center_y_px")
        if label != "inside" or center is None:
            finish()
        if f not in selected:
            continue
        scores[label] += 1
        obs = log.get(f)
        missing = obs is None
        valid = not missing and flag(obs["pixel_valid"])
        xy = point(obs, "x_px", "y_px") if valid else None
        if valid and xy is None:
            raise ValueError(f"{scene}/{profile} frame {f}: valid without finite center")
        scores["valid"] += int(valid)
        scores["skipped"] += int(missing)
        correct = valid and center is not None and math.hypot(xy[0]-center[0], xy[1]-center[1]) <= tol
        if valid and (label != "inside" or not correct):
            scores["flagged_identity"] += 1
            key = (scene, f, round(xy[0]), round(xy[1]))
            decision = (identity or {}).get(key, "unreviewed")
            bucket = {"on_ball_or_blurred_rim":"reviewed_ball","wrong_object":"reviewed_wrong_object"}.get(decision,"identity_unknown")
            scores[bucket] += 1
        if label == "inside":
            scores["inside_valid"] += int(valid)
            if valid:
                outcome = "inside_unscorable" if center is None else "inside_correct" if correct else "inside_wrong"
            else:
                outcome = "inside_skipped" if missing else "inside_invalid"
            scores[outcome] += 1
            if correct:
                finish()
            elif center is not None:
                run.append(f)
        elif valid:
            scores[label+"_valid"] += 1
            if label == "outside":
                outcome = "outside_unscorable" if center is None else "outside_on_ball" if correct else "outside_wrong"
                scores[outcome] += 1
    finish()
    result = dict(scene=scene,profile=profile,view=view,n_frames=len(selected),tol_px=tol,**scores)
    for suffix, index in [("count",0),("time",1)]:
        # 并列先比另一个指标，再取较早的区间；跨度取末次与首次缺测的采集时差。
        best = max(episodes,key=lambda a:(a[index],a[1-index],-a[2]),default=None)
        names = ("n","span_s","first_frame","last_frame","first_read_s","last_read_s")
        values = (*best,clocks[best[2]],clocks[best[3]]) if best else (0,0,"","","","")
        result.update({"longest_by_"+suffix+"_"+k:v for k,v in zip(names,values)})
    return result


def markdown(rows):
    lines = ["# 同帧钢珠像素输出评分", "",
             f"正确位置采用原始 1280×720 图像球心欧氏距离 ≤ {rows[0]['tol_px']:g} px。"
             "界内正确率分母含该视图全部明确界内帧。界外有效表示真球心已越界仍发出有效位置。"
             "原始时轴中的跳帧计缺测。处理子集取现场 measurements.csv 帧号。", ""]
    for view, title in [("raw","全部采集帧"),("processed","现场处理子集")]:
        lines += ["## "+title, "", "| 场景 | 输出 | 帧数 | 界内正确/界内 | 界内有效但错位 | 界外有效/界外 | 边界有效/边界 | 未知有效/未知 | 最长连续界内缺测（帧数，首末帧，采集跨度） |",
                  "|---|---|---:|---:|---:|---:|---:|---:|---|"]
        for r in rows:
            if r["view"] != view:
                continue
            gap = (f"{r['longest_by_count_n']}，{r['longest_by_count_first_frame']}–"
                   f"{r['longest_by_count_last_frame']}，{r['longest_by_count_span_s']:.3f} s"
                   if r["longest_by_count_n"] else "0")
            lines.append(f"| {r['scene']} | {r['profile']} | {r['n_frames']} | "
                         f"{r['inside_correct']}/{r['inside']} | {r['inside_wrong']} | "
                         f"{r['outside_valid']}/{r['outside']} | {r['border_valid']}/{r['border']} | "
                         f"{r['unknown_valid']}/{r['unknown']} | {gap} |")
        lines.append("")
    lines += ["## 超容差或非明确界内的有效标记人工核对", "",
              "| 场景 | 输出 | 需核对 | 真球或球缘拖影 | 确认错物 | 未确认身份 |",
              "|---|---|---:|---:|---:|---:|"]
    for r in rows:
        if r["view"]=="raw":
            lines.append(f"| {r['scene']} | {r['profile']} | {r['flagged_identity']} | {r['reviewed_ball']} | {r['reviewed_wrong_object']} | {r['identity_unknown']} |")
    lines.append("")
    if all(r["profile"] == "live" for r in rows):
        return "\n".join(lines)+"\n"
    lines += ["## 相对 baseline 的配对变化", "",
              "每行比较同一现场处理子集。增减为帧数；全部采集帧视图包含共同跳过的帧。", "",
              "| 场景 | 配置 | 界内正确变化 | 界内错位变化 | 界外有效变化 | 最长连续界内缺测变化（帧） |",
              "|---|---|---:|---:|---:|---:|"]
    for r in rows:
        if r["profile"] in {"baseline","live"} or r["view"] != "processed":
            continue
        base = next(x for x in rows if x["scene"]==r["scene"] and x["profile"]=="baseline" and x["view"]=="processed")
        diff = [r[k]-base[k] for k in ("inside_correct","inside_wrong","outside_valid","longest_by_count_n")]
        lines.append(f"| {r['scene']} | {r['profile']} | "+" | ".join(f"{d:+d}" for d in diff)+" |")
    lines += ["", "完整 CSV 同时列出界内有效、明确无效、跳帧、界外命中真球或错位、"
              "无中心的有效输出、按帧数和按时长各自最严重的区间及其两端原始 read_s。"
              "非界内帧始终中断区间。子集跨度可跨过未处理的界内帧，首末同帧的跨度为零。"]
    return "\n".join(lines)+"\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs",type=Path,required=True)
    parser.add_argument("--paired",type=Path,help="三组回放的根目录；省略时只评分现场输出")
    parser.add_argument("--truth",type=Path,default=HERE)
    parser.add_argument("--out",type=Path,default=HERE)
    parser.add_argument("--scene",action="append")
    parser.add_argument("--tol-px",type=float,default=12)
    parser.add_argument("--release-manifest",type=Path,help="随冻结回放保存的源代码清单，记录其 SHA-256")
    parser.add_argument("--points-only-comparison",action="store_true",help="只比较已提供的 baseline 与 release_points，同时保留现场日志")
    args = parser.parse_args()
    if not math.isfinite(args.tol_px) or args.tol_px <= 0:
        parser.error("tol-px must be finite and positive")
    results, manifest = [], []
    for scene in args.scene or SCENES:
        base = args.runs/scene
        times = read_index(base/"frames.csv")
        clocks = {f:number(times[f]["read_s"]) for f in sorted(times)}
        if not clocks or list(clocks) != list(range(len(clocks))) or any(
                b<=a for a,b in zip(clocks.values(),list(clocks.values())[1:])):
            raise ValueError(f"{scene}: expected complete ascending capture sequence")
        truth_file = args.truth/(scene+".csv")
        truth = read_index(truth_file)
        aligned(truth,clocks,"read_s",truth_file)
        identity_path = args.truth/"manual_identity.csv"
        identity = {}
        if identity_path.is_file():
            truth_sha = hashlib.sha256(truth_file.read_bytes()).hexdigest()
            with identity_path.open() as stream:
                for r in csv.DictReader(stream):
                    if r["scene"]==scene and r["truth_sha256"]==truth_sha:
                        key = (scene,int(r["frame"]),int(r["output_x_round_px"]),int(r["output_y_round_px"]))
                        identity[key] = r["manual_identity"]
        paths = {"live":base/"measurements.csv"}
        if args.paired:
            paths.update({p:args.paired/p/(scene+".csv") for p in (("baseline","release_points") if args.points_only_comparison else PROFILES[1:])})
        logs = {p:read_index(path) for p,path in paths.items()}
        for p,log in logs.items():
            aligned(log,clocks,"timestamp",paths[p])
            if set(log) != set(logs["live"]) and set(log) != set(clocks):
                raise ValueError(f"{scene}/{p}: incomplete or different processed subset")
            for view,selected in [("raw",set(clocks)),("processed",set(logs["live"]))]:
                results.append(score(scene,p,view,clocks,truth,log,selected,args.tol_px,identity))
        if identity_path.is_file():
            manifest.append(dict(scene=scene,input="manual identity audit",sha256=hashlib.sha256(identity_path.read_bytes()).hexdigest()))
        if args.release_manifest:
            manifest.append(dict(scene=scene,input="release source manifest",
                                 sha256=hashlib.sha256(args.release_manifest.read_bytes()).hexdigest()))
        for kind,path in {**paths,"truth":truth_file,"capture":base/"frames.csv"}.items():
            manifest.append(dict(scene=scene,input=kind,sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    args.out.mkdir(exist_ok=True,parents=True)
    for name,data in [("scoreboard.csv",results),("score_inputs.csv",manifest)]:
        with (args.out/name).open("w") as stream:
            writer = csv.DictWriter(stream,fieldnames=data[0])
            writer.writeheader()
            writer.writerows(data)
    (args.out/"scoreboard.md").write_text(markdown(results))


if __name__ == "__main__":
    main()
