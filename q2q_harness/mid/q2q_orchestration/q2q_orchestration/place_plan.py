"""Floor delivery beside the parked base, outside the Jackal footprint."""
from geometry_msgs.msg import Pose
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive


def goal_constraints():
    position = PositionConstraint()
    position.header.frame_id = 'base_footprint'
    position.link_name, position.weight = 'arm_0_end_effector_link', 1.
    target = Pose()
    target.position.x, target.position.z = .40, .28
    target.orientation.w = 1.
    position.constraint_region.primitives = [SolidPrimitive(
        type=SolidPrimitive.SPHERE, dimensions=[.005])]
    position.constraint_region.primitive_poses = [target]
    orientation = OrientationConstraint()
    orientation.header.frame_id, orientation.link_name = 'base_footprint', position.link_name
    orientation.orientation.y, orientation.weight = 1., 1.
    orientation.orientation.w = 0.  # Quaternion's message default is w=1; (0,1,0,1) is invalid.
    orientation.absolute_x_axis_tolerance = .04
    orientation.absolute_y_axis_tolerance = .04
    orientation.absolute_z_axis_tolerance = .04
    return [Constraints(position_constraints=[position], orientation_constraints=[orientation])]
