# -*- coding: utf-8 -*-
"""纯synthetic内参数学测试；不代表任何实机标定或镜头精度。"""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from camera_geometry import CameraGeometry


def synthetic_document():
    # 明确为单元测试虚构参数，只写TemporaryDirectory，不作生产内参。
    return dict(schema='camera-intrinsics-v1', model='opencv-pinhole-brown5-v1',
                resolution=[640, 480], camera_id='synthetic-camera', focus_id='synthetic-focus',
                K=[[500., 0., 320.], [0., 510., 240.], [0., 0., 1.]],
                D=[.08, -.02, .001, -.002, .005],
                validation={'passed': True},
                reprojection={'overall_rms_px': .1, 'per_view_rms_px': [.1]*12},
                inputs=[{'sha256': hashlib.sha256(str(i).encode()).hexdigest()} for i in range(12)])


def seal(document):
    result = copy.deepcopy(document)
    result.pop('integrity', None)
    canonical = json.dumps(result, sort_keys=True, separators=(',', ':'),
                           ensure_ascii=False, allow_nan=False).encode('utf-8')
    result['integrity'] = dict(algorithm='sha256', canonicalization='json-sort-keys-compact-utf8-v1',
                               payload_sha256=hashlib.sha256(canonical).hexdigest())
    return result


def independent_brown(document, pixels):
    """测试专用向量公式；不调用生产校验器或OpenCV投影求真值。"""
    K=np.asarray(document['K'],dtype=np.float64)
    x=(pixels[:,0]-K[0,2])/K[0,0];y=(pixels[:,1]-K[1,2])/K[1,1]
    k1,k2,p1,p2,k3=document['D'];r2=x*x+y*y
    radial=1+k1*r2+k2*r2*r2+k3*r2*r2*r2
    return np.column_stack((K[0,0]*(x*radial+2*p1*x*y+p2*(r2+2*x*x))+K[0,2],
                            K[1,1]*(y*radial+p1*(r2+2*y*y)+2*p2*x*y)+K[1,2]))


class CameraGeometryTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.path = self.root / 'synthetic.json'
        self.document = synthetic_document()
        self.write(self.document)

    def write(self, document):
        self.path.write_text(json.dumps(seal(document)))

    def geometry(self, mode='points', **options):
        cfg = dict(mode=mode, undistort_enabled=True, camera_id='synthetic-camera',
                   focus_id='synthetic-focus', intrinsics_file='synthetic.json')
        cfg.update(options)
        return CameraGeometry(cfg, 640, 480, base_dir=self.root)

    def test_raw_default_is_same_frame_and_values_without_intrinsics(self):
        geometry = CameraGeometry({}, 640, 480)
        frame = np.zeros((480, 640, 3), np.uint8)
        points = np.array([[1., 2.], [300., 200.]])
        self.assertIs(geometry.prepare_frame(frame), frame)
        np.testing.assert_array_equal(geometry.correct_points(points), points)
        self.assertEqual(geometry.image_space, 'raw_image')
        self.assertFalse(geometry.intrinsics_valid)
        self.assertEqual({k: geometry.context[k] for k in
                          ('mode', 'resolution', 'camera_id', 'focus_id', 'intrinsics_sha256')},
                         dict(mode='raw', resolution=[640,480], camera_id='', focus_id='', intrinsics_sha256=None))
        json.dumps(geometry.context, allow_nan=False)

    def test_points_are_pixel_coordinates_and_frame_is_untouched(self):
        geometry = self.geometry()
        K, D = np.array(self.document['K']), np.array(self.document['D'])
        # 先用独立正向投影产生有畸变点，校正后应回到理想P=K像素。
        objects = np.array([[-.25,-.2,1.], [.3,.15,1.], [0.,0.,1.]], np.float64)
        distorted = cv2.projectPoints(objects, np.zeros(3), np.zeros(3), K, D)[0].reshape(-1,2)
        ideal = np.column_stack((objects[:,0]*500+320, objects[:,1]*510+240))
        np.testing.assert_allclose(geometry.correct_points(distorted), ideal, atol=1e-5)
        frame = np.zeros((480,640,3),np.uint8)
        self.assertIs(geometry.prepare_frame(frame), frame)
        self.assertEqual(geometry.image_space, 'raw_image')
        self.assertTrue(geometry.intrinsics_valid)
        self.assertEqual(geometry.context['intrinsics_sha256'], hashlib.sha256(self.path.read_bytes()).hexdigest())

    def test_image_maps_precomputed_once_same_size_no_crop(self):
        # 通用正常路径只使用径向畸变；组合参数另有正确或明确拒绝的契约测试。
        self.document['D'][2:4]=[0.,0.];self.write(self.document)
        with patch('camera_geometry.cv2.initUndistortRectifyMap', wraps=cv2.initUndistortRectifyMap) as build:
            geometry = self.geometry('image')
            # synthetic连续二维亮度且边界归零：避免uint8的256回绕阶跃，
            # 也避免右边界非零像素与黑色border混合放大1/32像素量化差。
            yy, xx = np.indices((480,640), dtype=np.float32)
            frame = np.rint(255*np.sin(np.pi*xx/639)*np.sin(np.pi*yy/479)).clip(0,255).astype(np.uint8)
            a, b = geometry.prepare_frame(frame), geometry.prepare_frame(frame)
        self.assertEqual(build.call_count, 1)
        self.assertEqual(a.shape, frame.shape)
        self.assertIsNot(a, frame)
        np.testing.assert_array_equal(a,b)
        expected = cv2.undistort(frame, np.array(self.document['K']), np.array(self.document['D']),
                                 None, np.array(self.document['K']))
        # OpenCV4.1.1的undistort使用定点插值；float map在量化中点可能相差
        # 一档权重。保留原1灰度门限，并转有符号数，不能让-4被显示为252。
        np.testing.assert_allclose(a.astype(np.int16),expected.astype(np.int16),rtol=0,atol=1)
        self.assertEqual(geometry.image_space, 'undistorted_image')
        np.testing.assert_allclose(geometry.correct_points([[10.,20.]]), self.geometry().correct_points([[10.,20.]]))

    def test_image_sampling_matches_independent_brown_geometry(self):
        # 独立验证纯径向和纯切向正常路径，不用插值容差掩盖maps几何误差。
        pixels=np.array([(x,y) for y in (48,120,240,360,432)
                         for x in (64,160,320,480,576)],dtype=np.int32)
        for distortion in ([.08,-.02,0.,0.,.005], [0.,0.,.001,-.002,0.]):
            with self.subTest(distortion=distortion):
                self.document['D']=distortion;self.write(self.document)
                geometry=self.geometry('image')
                expected=independent_brown(self.document,pixels)
                maps=np.column_stack([m[pixels[:,1],pixels[:,0]] for m in geometry._maps])
                np.testing.assert_allclose(maps,expected,rtol=0,
                                           atol=float(np.spacing(np.float32(640.))))

    def test_combined_distortion_maps_are_correct_or_explicitly_rejected(self):
        # 不依据OpenCV版本skip：实际maps错误必须被拒绝，正确才允许启用。
        K=np.asarray(self.document['K']);D=np.asarray(self.document['D'])
        maps=cv2.initUndistortRectifyMap(K,D,None,K,(640,480),cv2.CV_32FC1)
        pixels=np.array([(x,y) for y in np.linspace(0,479,9,dtype=int)
                         for x in np.linspace(0,639,9,dtype=int)])
        expected=independent_brown(self.document,pixels)
        actual=np.column_stack([m[pixels[:,1],pixels[:,0]] for m in maps])
        limit=float(np.spacing(np.float32(max(640.,np.max(np.abs(expected))))))
        if np.max(np.abs(actual-expected))>limit:
            with self.assertRaisesRegex(ValueError,'image map Brown validation failed'):
                self.geometry('image')
        else:
            geometry=self.geometry('image')
            np.testing.assert_allclose(
                np.column_stack([m[pixels[:,1],pixels[:,0]] for m in geometry._maps]),
                expected,rtol=0,atol=limit)
            self.assertTrue(geometry.intrinsics_valid)

    def test_independent_correct_combined_maps_pass_and_bad_maps_fail_closed(self):
        # 直接注入独立公式的完整maps，证明组合参数本身未被禁止；人为几何错位
        # 和非有限值必须在构造时拒绝，而非等首次prepare_frame才暴露。
        yy,xx=np.indices((480,640))
        expected=independent_brown(self.document,np.column_stack((xx.ravel(),yy.ravel())))
        maps=tuple(expected[:,i].reshape(480,640).astype(np.float32) for i in range(2))
        with patch('camera_geometry.cv2.initUndistortRectifyMap',return_value=maps):
            self.assertTrue(self.geometry('image').intrinsics_valid)
        for value in (float(maps[0][0,0])+.01, float('nan')):
            wrong=tuple(m.copy() for m in maps);wrong[0][0,0]=value
            with self.subTest(value=value), patch('camera_geometry.cv2.initUndistortRectifyMap',return_value=wrong), \
                    patch('camera_geometry.cv2.remap') as remap:
                with self.assertRaisesRegex(ValueError,'image map Brown validation failed'):
                    self.geometry('image')
                remap.assert_not_called()
        # raw/points不依赖maps，不能因image兼容防护而改变原有路径。
        with patch('camera_geometry.cv2.initUndistortRectifyMap',side_effect=AssertionError('must not build maps')):
            CameraGeometry({},640,480)
            self.assertTrue(self.geometry('points').intrinsics_valid)

    def test_image_ramp_interpolation_quantizes_actual_maps(self):
        # 单独验证插值，不把实际maps当作Brown几何真值；几何由上一测试约束。
        self.document['D'][2:4]=[0.,0.];self.write(self.document)
        geometry=self.geometry('image')
        yy,xx=np.indices((480,640),dtype=np.float32)
        sampled=geometry.prepare_frame(np.dstack((xx,yy)))
        pixels=np.array([(x,y) for y in (48,120,240,360,432)
                         for x in (64,160,320,480,576)],dtype=np.int32)
        maps=np.column_stack([m[pixels[:,1],pixels[:,0]] for m in geometry._maps])
        self.assertTrue(np.all(maps>0))
        self.assertTrue(np.all(maps<np.array([639,479])))
        # 内部单位斜坡的双线性输出就是量化坐标。1/32表的权重为二进制
        # 精确分数，此坐标范围float32可精确表达；量化对maps误差最多半档。
        quantized=np.rint(maps*32.)/32.
        np.testing.assert_array_equal(sampled[pixels[:,1],pixels[:,0]],quantized)
        np.testing.assert_allclose(quantized,maps,rtol=0,atol=1./64)

    def test_enable_must_be_explicit_and_requires_parameter_file(self):
        for cfg in [{'mode':'points'}, {'mode':'image'}, {'mode':'raw','undistort_enabled':True},
                    {'mode':'unknown'}, {'mode':'points','undistort_enabled':'true'}]:
            with self.subTest(cfg=cfg), self.assertRaises(ValueError):
                CameraGeometry(cfg,640,480)
        with self.assertRaises(ValueError): self.geometry(intrinsics_file='absent.json')
        with self.assertRaises(ValueError): self.geometry(intrinsics_file=None)

    def test_binding_mismatch_is_not_silent_raw_fallback(self):
        for key,value in [('camera_id','other'),('focus_id','other'),('camera_id',''),('focus_id','')]:
            with self.subTest(key=key,value=value), self.assertRaises(ValueError):
                self.geometry(**{key:value})
        with self.assertRaises(ValueError):
            CameraGeometry(dict(mode='image',undistort_enabled=True,intrinsics_file=str(self.path),
                                camera_id='synthetic-camera',focus_id='synthetic-focus'),1280,720)

    def test_checksum_schema_and_bad_parameters_rejected(self):
        damaged = seal(self.document);damaged['K'][0][0]=900.
        self.path.write_text(json.dumps(damaged))
        with self.assertRaisesRegex(ValueError, 'checksum'): self.geometry()
        for key,value in [('schema','old'),('model','fisheye'),('D',[0]*4),
                          ('K',[[0,0,320],[0,510,240],[0,0,1]]),
                          ('validation',{'passed':False}),('resolution',[640.,480.]),
                          ('K',[[500,10,320],[0,510,240],[0,0,1]])]:
            doc=copy.deepcopy(self.document);doc[key]=value;self.write(doc)
            with self.subTest(key=key,value=value),self.assertRaises(ValueError): self.geometry()

    def test_optional_file_sha_binding_and_nonfinite_file_rejected(self):
        with self.assertRaisesRegex(ValueError,'SHA256'):self.geometry(intrinsics_sha256='0'*64)
        damaged=seal(self.document);damaged['D'][0]=float('nan')
        self.path.write_text(json.dumps(damaged))
        with self.assertRaises(ValueError):self.geometry()
        self.path.write_text('{"schema":"camera-intrinsics-v1","schema":"other"}')
        with self.assertRaisesRegex(ValueError,'duplicate'):self.geometry()

    def test_nonfinite_or_wrong_shape_points_rejected(self):
        geometry=self.geometry()
        for value in [[[float('nan'),1]], [1,2], [[1,2,3]], [[1,float('inf')]]]:
            with self.assertRaises(ValueError):geometry.correct_points(value)
        self.assertEqual(geometry.correct_points(np.empty((0,2))).shape,(0,2))
        with self.assertRaises(ValueError):geometry.prepare_frame(np.zeros((240,320,3),np.uint8))


if __name__=='__main__':unittest.main()
