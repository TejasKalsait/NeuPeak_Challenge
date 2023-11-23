from row_nav import *
import numpy
import unittest

rclpy.init()

class TestMyMath(unittest.TestCase):
    
    # Test for 1.npz
    def test_1_npz(self):
        angle_pub = Angle_Publisher("1.npz", False, 4)
        angular_turn , end_of_row= angle_pub.anglePublishOnce()
        self.assertIsInstance(angular_turn, (int, float))
        self.assertEqual(end_of_row, False)
        self.assertTrue(-90.0 < angular_turn < 90.0)

    # Test for 2.npz (End of Row example)
    # I'm assuming even if the end_of_row flag is raised, the bot should not have angular velocity
    def test_2_npz(self):
        angle_pub = Angle_Publisher("2.npz", False, 4)
        angular_turn , end_of_row= angle_pub.anglePublishOnce()
        self.assertIsInstance(angular_turn, (int, float))
        self.assertEqual(end_of_row, False)
        self.assertTrue(-90.0 < angular_turn < 90.0)

    # Test for 3.npz
    def test_3_npz(self):
        angle_pub = Angle_Publisher("3.npz", False, 4)
        angular_turn , end_of_row= angle_pub.anglePublishOnce()
        self.assertIsInstance(angular_turn, (int, float))
        self.assertEqual(end_of_row, False)
        self.assertTrue(-90.0 < angular_turn < 90.0)

    # Test for 4.npz
    def test_4_npz(self):
        angle_pub = Angle_Publisher("4.npz", False, 4)
        angular_turn , end_of_row= angle_pub.anglePublishOnce()
        self.assertIsInstance(angular_turn, (int, float))
        self.assertEqual(end_of_row, False)
        self.assertTrue(-90.0 < angular_turn < 90.0)


if __name__ == '__main__':
    unittest.main()