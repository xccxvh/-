# -*- coding: utf-8 -*-
"""真实参考图的软件旋转/合成负例；不代表机械倾角或实机精度认证。"""
import json
from pathlib import Path
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from local_vision import UpperEdgeReference
from ruler_reference import RulerReference


class RotationRecheckTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root=Path(__file__).resolve().parents[1]
        cls.cfg=json.loads((cls.root/'config_geometry_refocused_20261116.json').read_text())['rail']['upper_reference']
        cls.cfg.pop('rotation_recheck',None)  # 父profile以后启用也不改变默认关闭测试。
        cls.ref=cv2.imread(str(cls.root/cls.cfg['image']),0)
        if cls.ref is None:raise RuntimeError('real reference fixture missing')

    def matcher(self, enabled=True, **kwargs):
        options=dict(search_x=45,search_y=45,rotation_recheck=enabled)
        options.update(kwargs)
        return RulerReference(self.ref,self.cfg['boxes'],**options)

    def moved(self, angle, shift=(0.,0.), source=None):
        matrix=cv2.getRotationMatrix2D((546.,315.),angle,1.)
        matrix[:,2]+=shift
        return cv2.warpAffine(self.ref if source is None else source,matrix,(1280,720)),matrix

    def test_real_reference_plus_minus_two_with_translation_reconfirmed(self):
        for angle,shift in [(2.,(0.,0.)),(-2.,(0.,0.)),(2.,(6.,-4.)),(-2.,(-6.,4.))]:
            with self.subTest(angle=angle,shift=shift):
                image,matrix=self.moved(angle,shift)
                legacy=RulerReference(self.ref,self.cfg['boxes'],search_x=45,search_y=45)
                self.assertEqual(legacy.update(image),{'valid':False,'reason':'weak_or_ambiguous_patch'})
                tracker=self.matcher();out=tracker.update(image)
                self.assertTrue(out['valid'])
                self.assertTrue(out['rotation_rechecked'])
                self.assertGreaterEqual(out['min_score'],.85)
                self.assertGreaterEqual(out['min_margin'],.05)
                self.assertLessEqual(out['residual_px'],2.5)
                point=matrix.dot([546.,315.,1.])
                np.testing.assert_allclose(tracker.to_reference(*point),[546.,315.],atol=1.)

    def test_default_disabled_and_fast_success_outputs_unchanged(self):
        image,_=self.moved(0.,(6.,-4.))
        legacy=RulerReference(self.ref,self.cfg['boxes'],search_x=45,search_y=45)
        expected=legacy.update(image)
        self.assertTrue(expected['valid'])
        enabled=self.matcher()
        with patch.object(enabled,'_rotation_recheck',side_effect=AssertionError('fast path changed')):
            self.assertEqual(enabled.update(image),expected)
        np.testing.assert_array_equal(enabled.transform[0],legacy.transform[0])
        np.testing.assert_array_equal(enabled.transform[1],legacy.transform[1])
        disabled=self.matcher(False)
        tilted,_=self.moved(2.)
        with patch.object(disabled,'_rotation_recheck',side_effect=AssertionError('disabled branch entered')):
            self.assertEqual(disabled.update(tilted),{'valid':False,'reason':'weak_or_ambiguous_patch'})
        self.assertIsNone(disabled.transform)

    def test_occlusion_and_uniform_image_clear_previous_transform(self):
        tracker=self.matcher()
        for invalid in [np.zeros_like(self.ref),self.ref.copy()]:
            if invalid.any():
                x,y,w,h=self.cfg['boxes'][1]
                invalid[y-45:y+h+45,x-45:x+w+45]=127
                invalid,_=self.moved(2.,source=invalid)
            self.assertTrue(tracker.update(self.ref)['valid'])
            self.assertFalse(tracker.update(invalid)['valid'])
            self.assertIsNone(tracker.transform)

    def test_repeated_texture_cannot_be_disambiguated_by_narrow_warp(self):
        rng=np.random.RandomState(8)
        image=np.tile(rng.randint(0,256,(240,20)).astype(np.uint8),(1,32))
        boxes=[(90,100,40,24),(290,100,40,24),(490,100,40,24)]
        tracker=RulerReference(image,boxes,search_x=30,search_y=25,rotation_recheck=True)
        result=tracker.update(image)
        self.assertFalse(result['valid'])
        self.assertEqual(result['reason'],'rotation_recheck_weak_or_ambiguous_patch')
        self.assertIsNone(tracker.transform)

    def test_two_copies_at_opposite_search_edges_remain_ambiguous(self):
        # Dalton复现：提议平移-35后，原固定尺寸窗会丢掉+35的等高竞争峰。
        reference=np.full((240,800),128,np.uint8)
        current=reference.copy()
        tile=np.random.RandomState(739).randint(0,256,(24,24)).astype(np.uint8)
        boxes=[(120,100,24,24),(360,100,24,24),(600,100,24,24)]
        for x,y,w,h in boxes:
            reference[y:y+h,x:x+w]=tile
            for dx in (-35,35):current[y:y+h,x+dx:x+dx+w]=tile
        tracker=RulerReference(reference,boxes,search_x=45,search_y=30,rotation_recheck=True)
        result=tracker.update(current)
        self.assertFalse(result['valid'])
        self.assertEqual(result['reason'],'rotation_recheck_weak_or_ambiguous_patch')
        self.assertIsNone(tracker.transform)

    def test_no_trusted_proposal_cannot_reach_rectification(self):
        tracker=self.matcher()
        self.assertTrue(tracker.update(self.ref)['valid'])
        with patch.object(tracker,'_rectified_search',side_effect=AssertionError('untrusted proposal used')):
            result=tracker.update(np.zeros_like(self.ref))
        self.assertEqual(result,{'valid':False,'reason':'rotation_proposal_untrusted'})
        self.assertIsNone(tracker.transform)

    def test_clipped_rectification_clears_prior_valid_transform(self):
        tracker=self.matcher();self.assertTrue(tracker.update(self.ref)['valid'])
        image,_=self.moved(2.)
        with patch.object(tracker,'_rectified_search',return_value=None):
            result=tracker.update(image)
        self.assertEqual(result,{'valid':False,'reason':'rotation_recheck_clipped'})
        self.assertIsNone(tracker.transform)

    def test_single_patch_independent_motion_is_not_rigid_reference(self):
        image=self.ref.copy();x,y,w,h=self.cfg['boxes'][1]
        image[y:y+h,x:x+w]=127
        image[y+12:y+12+h,x:x+w]=self.ref[y:y+h,x:x+w]
        tracker=self.matcher()
        self.assertFalse(tracker.update(image)['valid'])
        self.assertIsNone(tracker.transform)

    def test_no_ball_does_not_invalidate_reference(self):
        without=self.ref.copy()
        without[240:315,310:410]=140  # 改球冠区，不改纸尺纹理模板。
        for angle in [-2.,2.]:
            image,_=self.moved(angle,source=without)
            self.assertTrue(self.matcher().update(image)['valid'])

    def test_full_window_shape_and_no_border_padding(self):
        tracker=self.matcher();box=tracker.patches[0]
        matrix=np.eye(2);shift=np.zeros(2)
        window,origin=tracker._rectified_search(self.ref,box,matrix,shift)
        x,y,w,h,_=box
        self.assertEqual(window.shape,(h+90,w+90))
        np.testing.assert_array_equal(window,self.ref[y-45:y+h+45,x-45:x+w+45])
        self.assertIsNone(tracker._rectified_search(self.ref,box,matrix,[-180.,0.]))
        clipped=self.matcher(search_x=220)
        self.assertIsNone(clipped._rectified_search(self.ref,clipped.patches[0],matrix,shift))

    def test_adaptive_search_and_rotation_recheck_are_independent_booleans(self):
        # 配置文件中的字符串和整数不应隐式切换算法。关闭旋转复核时，
        # 即使已启用自适应跟踪，发生旋转的小窗失败也不能触发旋转复核。
        for key in ('rotation_recheck', 'adaptive_search'):
            for value in (1, 'true', 'adaptive'):
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    UpperEdgeReference(dict(self.cfg, **{key: value}))
        edge = UpperEdgeReference(dict(self.cfg, adaptive_search=True))
        self.assertIsNotNone(edge.locate(self.ref))
        tilted, _ = self.moved(2.)
        self.assertIsNone(edge.locate(tilted))
        self.assertIsNone(edge.matcher.transform)

    def test_adaptive_track_and_same_frame_relocation(self):
        # 通过实际配置入口连续送帧；大跳变必须在该帧内重新匹配原图模板。
        edge = UpperEdgeReference(dict(self.cfg, adaptive_search=True, rotation_recheck=True))
        for dx, mode in [(0, 'relocalize'), (4, 'track'), (36, 'relocalize')]:
            image, _ = self.moved(0., (dx, 0.))
            evidence = edge.locate(image)
            self.assertIsNotNone(evidence)
            self.assertEqual(evidence['registration']['registration_mode'], mode)
            np.testing.assert_allclose(edge.matcher.to_reference(546+dx, 315),
                                       [546., 315.], atol=1.)

        # 遮挡和尺寸变化均中断历史；恢复帧即使在前次小窗外也能独立定位。
        for invalid in (np.zeros_like(self.ref), self.ref[:-1]):
            self.assertIsNone(edge.locate(invalid))
            self.assertIsNone(edge.matcher.to_reference(546, 315))
            image, _ = self.moved(0., (-35., 0.))
            evidence = edge.locate(image)
            self.assertEqual(evidence['registration']['registration_mode'], 'relocalize')
            np.testing.assert_allclose(edge.matcher.to_reference(511, 315),
                                       [546., 315.], atol=1.)

    def test_cold_and_lost_relocate_70px_at_clipped_lower_search_edge(self):
        # 使用真实模板并裁低画面，让扩展搜索窗穿过底边。模板仍完整可见
        # 时必须在当前帧复核成功；向下移动到模板出画时必须清除映射。
        ref = self.ref[:400]
        tracker = RulerReference(ref, self.cfg['boxes'], search_x=96, search_y=96,
                                 rotation_recheck=True, adaptive_search=True)
        for dx, dy in [(70, 0), (-70, -70)]:
            matrix = cv2.getRotationMatrix2D((546., 315.), 2., 1.)
            matrix[:, 2] += [dx, dy]
            image = cv2.warpAffine(ref, matrix, (ref.shape[1], ref.shape[0]))
            result = tracker.update(image)
            self.assertTrue(result['valid'], result)
            self.assertTrue(result['rotation_rechecked'])
            self.assertEqual(result['registration_mode'], 'relocalize')
            point = matrix.dot([546., 315., 1.])
            np.testing.assert_allclose(tracker.to_reference(*point), [546., 315.], atol=1.)
            for _ in range(3):
                self.assertFalse(tracker.update(np.zeros_like(ref))['valid'])
                self.assertIsNone(tracker.transform)
        down = cv2.warpAffine(ref, np.float32([[1, 0, 0], [0, 1, 70]]),
                              (ref.shape[1], ref.shape[0]))
        self.assertFalse(tracker.update(down)['valid'])
        self.assertIsNone(tracker.transform)

    def test_adaptive_warm_tracker_rejects_second_identical_ruler(self):
        # 初次定位只有一套唯一标签。下一帧小窗外出现第二套完整副本，
        # 两者各自都能拟合零残差姿态，必须保留大窗的身份竞争证据。
        ref = np.full((160, 640), 127, np.uint8)
        boxes = [(90, 70, 16, 16), (280, 70, 16, 16), (490, 70, 16, 16)]
        rng = np.random.RandomState(25)
        for x, y, w, h in boxes:
            ref[y:y+h, x:x+w] = rng.randint(0, 256, (h, w))
        tracker = RulerReference(ref, boxes, search_x=45, adaptive_search=True)
        self.assertTrue(tracker.update(ref)['valid'])
        duplicate = ref.copy()
        for x, y, w, h in boxes:
            duplicate[y:y+h, x+36:x+36+w] = ref[y:y+h, x:x+w]
        self.assertFalse(tracker.update(duplicate)['valid'])
        self.assertIsNone(tracker.to_reference(280, 70))

    def test_adaptive_tracking_respects_absolute_search_limit(self):
        tracker = self.matcher(adaptive_search=True)
        # 累积小步越过绝对位移预算时也须失效，防止沿尺子重复纹理持续漂移。
        for dx in range(0, 46, 5):
            image, _ = self.moved(0., (dx, 0.))
            self.assertTrue(tracker.update(image)['valid'])
        image, _ = self.moved(0., (52., 0.))
        self.assertFalse(tracker.update(image)['valid'])
        self.assertIsNone(tracker.transform)

    def test_upper_reference_forwards_explicit_flag_only(self):
        image,_=self.moved(2.)
        default=UpperEdgeReference(self.cfg)
        self.assertIsNone(default.locate(image))
        enabled=UpperEdgeReference(dict(self.cfg,rotation_recheck=True))
        evidence=enabled.locate(image)
        self.assertIsNotNone(evidence)
        self.assertTrue(evidence['registration']['rotation_rechecked'])
        with self.assertRaises(ValueError):self.matcher(enabled='true')


if __name__=='__main__':unittest.main()
