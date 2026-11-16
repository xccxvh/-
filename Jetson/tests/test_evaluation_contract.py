import unittest
from evaluate_local_roi import validate_labels, assert_coverage, assess_pixel

class EvaluationContractTests(unittest.TestCase):
    def test_duplicate_and_outside_labels_rejected(self):
        for labels in [[{'frame':0},{'frame':0}],[{'frame':5}]]:
            with self.assertRaises(ValueError):validate_labels(labels,5)
    def test_truncated_or_wrong_resolution_rejected(self):
        for actual,shape in [(4,(720,1280,3)),(5,(360,640,3))]:
            with self.assertRaises(ValueError):assert_coverage(actual,5,shape)
    def test_visible_pixel_truth_retained_if_reference_lost(self):
        self.assertEqual(assess_pixel({'valid':True,'x_px':100},100,6,'reference_unknown'),('correct',0))
        self.assertEqual(assess_pixel({'valid':False},100,6,'reference_unknown'),('miss',None))
