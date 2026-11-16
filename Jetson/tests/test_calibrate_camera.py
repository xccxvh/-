# -*- coding: utf-8 -*-
"""Synthetic棋盘投影及离线CLI测试，不是实机内参或实拍覆盖证明。"""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

import calibrate_camera as tool
from camera_geometry import CameraGeometry


def synthetic_observations(frontoparallel=False):
    objects=np.zeros((7*5,3),np.float32)
    objects[:,:2]=np.mgrid[0:7,0:5].T.reshape(-1,2)*25.
    K=np.array([[950.,0,640],[0,970.,360],[0,0,1.]])
    D=np.array([.03,-.01,.0005,-.0003,.002])
    observations=[]
    for i in range(16):
        rotation=np.array([[-.3,-.1,.15,.35][i%4],[-.3,.2,-.2,.3][i//4],.03*(i%3-1)])
        if frontoparallel:rotation=np.zeros(3)
        matrix=cv2.Rodrigues(rotation)[0]
        center=np.array([[-180,-60,60,180][i%4],[-110,-40,40,110][i//4],550+(i%3)*90.])
        translation=center-matrix.dot([75.,50.,0.])
        corners=cv2.projectPoints(objects,rotation,translation,K,D)[0].reshape(-1,2)
        observations.append(dict(path='synthetic-view-{}'.format(i),
            sha256=hashlib.sha256(str(i).encode()).hexdigest(),corners=corners))
    return observations


class CalibrateCameraTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name)

    def fit(self, observations):
        return tool.calibrate_observations(observations,(1280,720),7,5,25.,'synthetic-camera','synthetic-focus')

    def test_synthetic_multiview_fit_has_residuals_hashes_and_loadable_binding(self):
        doc=self.fit(synthetic_observations())
        self.assertTrue(doc['validation']['passed'])
        self.assertEqual(len(doc['reprojection']['per_view_rms_px']),16)
        self.assertLess(doc['reprojection']['overall_rms_px'],.001)
        self.assertAlmostEqual(doc['K'][0][0],950.,delta=.5)
        self.assertEqual(len(doc['inputs']),16)
        path=self.root/'synthetic.json';path.write_text(json.dumps(doc))
        geometry=CameraGeometry(dict(mode='points',undistort_enabled=True,
            intrinsics_file=str(path),camera_id='synthetic-camera',focus_id='synthetic-focus'),1280,720)
        self.assertTrue(geometry.intrinsics_valid)

    def test_insufficient_duplicate_and_degenerate_views_rejected(self):
        observations=synthetic_observations()
        with self.assertRaises(ValueError):self.fit(observations[:4])
        duplicate=synthetic_observations();duplicate[-1]['sha256']=duplicate[0]['sha256']
        with self.assertRaisesRegex(ValueError,'duplicate'):self.fit(duplicate)
        same_pose=synthetic_observations()
        for view in same_pose:view['corners']=observations[0]['corners'].copy()
        with self.assertRaisesRegex(ValueError,'duplicate|coverage'):self.fit(same_pose)
        tiny=synthetic_observations()
        for view in tiny:view['corners']=np.array([640,360])+(view['corners']-[640,360])*.1
        with self.assertRaisesRegex(ValueError,'coverage'):self.fit(tiny)

    def test_parallel_views_with_translation_and_scale_still_rejected(self):
        # 仅平移/改变距离可满足二维覆盖，却不能可靠辨识焦距和畸变。
        with self.assertRaises(ValueError):self.fit(synthetic_observations(frontoparallel=True))

    def test_collector_rejects_duplicate_bytes_and_mixed_resolution(self):
        image=np.full((480,640),127,np.uint8)
        for row in range(6):
            for col in range(8):
                image[100+row*40:140+row*40,140+col*40:180+col*40]=255 if (row+col)%2 else 0
        first=self.root/'a.png';second=self.root/'b.png'
        cv2.imwrite(str(first),image);second.write_bytes(first.read_bytes())
        with self.assertRaisesRegex(ValueError,'duplicate'):tool.collect_images(str(self.root),7,5)
        cv2.imwrite(str(second),cv2.resize(image,(1280,960)))
        with self.assertRaisesRegex(ValueError,'resolution'):tool.collect_images(str(self.root),7,5)
        first.write_bytes(b'not an image')
        with self.assertRaisesRegex(ValueError,'unreadable'):tool.collect_images(str(self.root),7,5)

    def test_bad_view_is_rejected_not_automatically_dropped(self):
        observations=synthetic_observations()
        observations[-1]['corners']=observations[-1]['corners'].copy()
        observations[-1]['corners'][10]+=[30.,-30.]
        with self.assertRaisesRegex(ValueError,'reprojection'):self.fit(observations)

    def test_collector_rejects_plain_a4_without_chessboard(self):
        for i in range(12):
            cv2.imwrite(str(self.root/('blank{:02d}.png'.format(i))),np.full((480,640,3),180+i,np.uint8))
        with self.assertRaisesRegex(ValueError,'chessboard'):tool.collect_images(str(self.root),7,5)

    def test_collector_runs_real_corner_detection_on_synthetic_board(self):
        image=np.full((480,640),127,np.uint8)
        for row in range(6):
            for col in range(8):
                image[100+row*40:140+row*40,140+col*40:180+col*40]=255 if (row+col)%2 else 0
        path=self.root/'synthetic-board.png';cv2.imwrite(str(path),image)
        observations,resolution=tool.collect_images(str(self.root/'*.png'),7,5)
        self.assertEqual(resolution,(640,480))
        self.assertEqual(observations[0]['corners'].shape,(35,2))
        self.assertEqual(observations[0]['sha256'],hashlib.sha256(path.read_bytes()).hexdigest())

    def test_cli_never_overwrites_or_writes_on_bad_data(self):
        output=self.root/'synthetic-output.json'
        argv=['--images',str(self.root/'*.png'),'--cols','7','--rows','5','--square-mm','25',
              '--camera-id','synthetic-camera','--focus-id','synthetic-focus','--output',str(output)]
        self.assertNotEqual(tool.main(argv),0);self.assertFalse(output.exists())
        output.write_text('preserve')
        self.assertNotEqual(tool.main(argv),0);self.assertEqual(output.read_text(),'preserve')

    def test_cli_writes_only_after_all_synthetic_views_pass(self):
        output=self.root/'synthetic-output.json'
        with patch.object(tool,'collect_images',return_value=(synthetic_observations(),(1280,720))):
            code=tool.main(['--images','synthetic-only','--cols','7','--rows','5','--square-mm','25',
                '--camera-id','synthetic-camera','--focus-id','synthetic-focus','--output',str(output)])
        self.assertEqual(code,0)
        self.assertTrue(json.loads(output.read_text())['validation']['passed'])


if __name__=='__main__':unittest.main()
