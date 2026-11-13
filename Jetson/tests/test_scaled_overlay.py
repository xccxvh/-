import unittest
import numpy as np
from rail_detector import RailLine
from visualization import draw_overlay

class ScaledOverlayTests(unittest.TestCase):
    def test_small_overlay_projects_marks_without_mutating_measurement(self):
        frame=np.zeros((720,1280,3),np.uint8);rail=RailLine(.1,300,True)
        track={'state':'DETECTED','x_px':1000.,'confidence':.8,'method':'geom','predicted':False}
        vis=draw_overlay(frame,rail,[],track,30,10,output_size=(640,360))
        self.assertEqual(vis.shape,(360,640,3))
        self.assertTrue(np.array_equal(vis[200,500],[0,255,0]))
        # Rasterized integer endpoints may move a sloping line by one pixel.
        self.assertTrue(np.any(np.all(vis[209:212,600]==[0,255,0],axis=1)))
        self.assertEqual(track['x_px'],1000.)
        self.assertEqual(rail.intercept,300.)
        self.assertFalse(frame.any())
