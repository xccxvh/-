"""只读冻结实拍和配置。独立副本的旋转复核、竞争峰、遮挡及无预测回归。"""
import json
from pathlib import Path
import unittest
import cv2
import numpy as np
from ruler_reference import RulerReference

JET = Path(__file__).resolve().parents[1]
CFG = json.loads((JET/'config_motion_ruler_digits_20261121.json').read_text())['rail']['upper_reference']
REF = cv2.imread(str(JET/CFG['image']),0)

def matcher():
    return RulerReference(REF,CFG['boxes'],96,96,rotation_recheck=True,adaptive_search=True)

def warp(g, dx, dy):
    return cv2.warpAffine(g,np.float32([[1,0,dx],[0,1,dy]]),(g.shape[1],g.shape[0]))

class GuidedTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # 取自 895 帧行驶原片的第 1 帧；使用相机原始 JPEG 和线上 BGR 转灰路径。
        image = cv2.imread(str(JET/'tests/fixtures/moving_free_rotation.jpg'))
        cls.gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

    def warm(self):
        tr=matcher()
        self.assertTrue(tr.update(self.gray)['rotation_rechecked'])
        return tr

    def test_current_local_features_follow_translation(self):
        tr=self.warm()
        first=tr.to_reference(650,645)
        result=tr.update(warp(self.gray,5,-4))
        self.assertEqual(result['registration_mode'],'rotation_track')
        self.assertGreaterEqual(result['min_score'],.85)
        self.assertGreaterEqual(result['min_margin'],.05)
        np.testing.assert_allclose(tr.to_reference(655,641),first,atol=.5)

    def test_each_patch_is_required_in_current_frame(self):
        for x,y,w,h in CFG['boxes']:
            tr=self.warm()
            gray=self.gray.copy()
            gray[y-16:y+h+16,x-18:x+w+18]=127
            self.assertFalse(tr.update(gray)['valid'])
            self.assertIsNone(tr.to_reference(650,645))
            self.assertEqual(tr.update(self.gray)['registration_mode'],'relocalize')

    def test_distant_competitor_for_each_single_patch(self):
        for base in (REF,self.gray):
            for x,y,w,h in CFG['boxes']:
                tr=matcher()
                self.assertTrue(tr.update(base)['valid'])
                gray=base.copy()
                # 复制足够的上下文，86 px 位移处在全窗内、历帧小窗外。
                yy,xx=y-12,x-18
                gray[yy-86:y+h+16-86,xx:x+w+18]=base[yy:y+h+16,xx:x+w+18]
                self.assertFalse(tr.update(gray)['valid'])
                self.assertIsNone(tr.transform)

    def test_invalid_format_and_blank_reset_history_then_relocate(self):
        for bad in (self.gray[:-1],np.zeros_like(REF)):
            tr=self.warm()
            self.assertFalse(tr.update(bad)['valid'])
            self.assertIsNone(tr.transform)
            result=tr.update(warp(self.gray,70,-3))
            self.assertTrue(result['valid'],result)
            self.assertEqual(result['registration_mode'],'relocalize')
