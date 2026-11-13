# -*- coding: utf-8 -*-
"""Sample visualization by source time without replaying missed time slots."""
import math


class RecordingSchedule(object):
    def __init__(self, fps):
        self.fps = float(fps)
        if not math.isfinite(self.fps) or self.fps <= 0:
            raise ValueError('recording fps must be finite and positive')
        self.origin = None
        self.last_slot = -1

    def due(self, timestamp):
        if timestamp is None or not math.isfinite(timestamp):
            return False
        if self.origin is None:
            self.origin = timestamp
        elapsed = timestamp - self.origin
        if not math.isfinite(elapsed) or elapsed < 0:
            return False
        slot = int(math.floor(elapsed * self.fps + 1e-7))
        if slot <= self.last_slot:
            return False
        self.last_slot = slot
        return True
