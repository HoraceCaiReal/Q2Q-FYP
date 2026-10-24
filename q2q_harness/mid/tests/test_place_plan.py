"""ROS message defaults must not turn a valid orientation into (0,1,0,1)."""
import math
import unittest

try:
    from q2q_orchestration.place_plan import goal_constraints
    ROS_AVAILABLE = True
except ModuleNotFoundError:
    ROS_AVAILABLE = False


@unittest.skipUnless(ROS_AVAILABLE, 'Requires the ROS message packages in the VM')
class PlaceOrientationTest(unittest.TestCase):
    def test_orientation_is_a_unit_quaternion(self):
        for goal in goal_constraints():
            for constraint in goal.orientation_constraints:
                q = constraint.orientation
                norm = sum(v*v for v in (q.x, q.y, q.z, q.w))
                self.assertTrue(math.isfinite(norm))
                self.assertAlmostEqual(norm, 1., places=12)


if __name__ == '__main__':
    unittest.main()
