"""静态闭环 C 路径：沿用冻结实验的候选优先级和跟踪身份规则。"""


def supplement(motion, gray, rail, bounds, timestamp, original):
    # 即使原检测有候选，也要保存本帧轨道窄带，供下一对图像做配准。
    extra = motion.update(gray, rail, bounds, timestamp, propose=not original)
    # 运动分支只有当前帧实测球心。其结果不能积累原 Hough 的边界快速确认计数。
    return (extra, None) if not original and extra else (original, bounds)
