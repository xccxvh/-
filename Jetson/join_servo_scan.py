"""把同次 Nano 扫描事件与俯视相机测量对齐。两端设备时钟始终分列保存。"""
import argparse
from bisect import bisect_right
import csv


def join(scan, track, output):
    # 视觉进程独立记录，每行的时钟为 Nano 读帧时刻。保留无效样本以中断旧位置。
    vision = list(csv.DictReader(track))
    times = [float(row['capture_monotonic_ms']) for row in vision]
    if any(b < a for a, b in zip(times, times[1:])):
        raise ValueError('请使用同一次运行的视觉 CSV')
    events = csv.DictReader(scan)
    extra = ['vision_frame', 'vision_read_ms', 'vision_age_ms', 'ball_x_cm', 'cm_valid']
    fields = list(events.fieldnames)
    writer = csv.DictWriter(output, fields + [key for key in extra if key not in fields])
    writer.writeheader()
    for row in events:
        # 一条遥测可与最近读到的一帧关联。超龄、预测、域外、丢球均留空厘米值。
        stamp = 1000.0 * float(row['nano_timestamp'])
        i = bisect_right(times, stamp) - 1
        row.update(vision_frame='', vision_read_ms='', vision_age_ms='', ball_x_cm='', cm_valid=0)
        if i >= 0:
            measurement = vision[i]
            age = stamp - times[i]
            row.update(vision_frame=measurement['frame'], vision_read_ms=times[i],
                       vision_age_ms='{:.3f}'.format(age))
            if age <= 100.0 and measurement['cm_valid'] == '1' and measurement['predicted'] == '0':
                row.update(ball_x_cm=measurement['rail_x_cm'], cm_valid=1)
        writer.writerow(row)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scan', help='servo_scan 指令/遥测 CSV')
    parser.add_argument('track', help='同一 Nano 同次运行的 main --csv 视觉记录')
    parser.add_argument('output', help='新建联合 CSV')
    args = parser.parse_args()
    with open(args.scan, newline='') as scan, open(args.track, newline='') as track, open(args.output, 'x', newline='') as out:
        join(scan, track, out)
