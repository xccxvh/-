"""断言联合记录在错时、丢球、超龄和预测帧下保留事件并清空厘米值。"""
import csv
import io
import unittest

from join_servo_scan import join


class JoinServoTests(unittest.TestCase):
    def test_join_on_nano_read_time(self):
        track = io.StringIO('frame,capture_monotonic_ms,cm_valid,predicted,rail_x_cm\n'
                            '1,1100,1,0,-0.4\n2,1300,0,0,\n3,1400,1,1,2.5\n')
        scan = io.StringIO('nano_timestamp,board_timestamp\n'
                           '1.0,50\n1.15,200\n1.24,290\n1.35,400\n1.45,500\n')
        result = io.StringIO()
        join(scan, track, result)
        rows = list(csv.DictReader(io.StringIO(result.getvalue())))
        self.assertEqual([r['ball_x_cm'] for r in rows], ['', '-0.4', '', '', ''])
        self.assertEqual([r['cm_valid'] for r in rows], ['0', '1', '0', '0', '0'])
        self.assertEqual(rows[1]['vision_age_ms'], '50.000')
        self.assertEqual(rows[1]['board_timestamp'], '200')
