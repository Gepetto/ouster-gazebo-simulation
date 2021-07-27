#!/usr/bin/env python

import unittest

import rospy
import rosunit

from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point
from sensor_msgs.point_cloud2 import PointCloud2

boxy = """<robot name="boxy">
  <link name="boxy">
    <inertial>
      <origin xyz="0 0 0" />
      <mass value="1" />
      <inertia ixx="1" ixy="1" ixz="1" iyy="1" iyz="1" izz="1" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" />
      <geometry><box size="1 1 1" /></geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" />
      <geometry><box size="1 1 1" /></geometry>
    </collision>
  </link>
  <gazebo reference="boxy"><material>Gazebo/Blue</material></gazebo>
</robot>"""


class OusterTest(unittest.TestCase):
    def test_insert_cube(self):
        rospy.init_node('ouster_test_node', anonymous=True)

        # Get 2 point clouds
        msg1 = rospy.wait_for_message("/os1_cloud_node/points", PointCloud2, timeout=30)
        msg2 = rospy.wait_for_message("/os1_cloud_node/points", PointCloud2, timeout=1)

        # Ensure they are different
        self.assertNotEqual(msg1.header.stamp, msg2.header.stamp)

        # The scene is the same: they should provide the same data
        self.assertEqual(msg1.data, msg2.data)

        # Spawn a cube
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        spawn_sdf_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        spawn_sdf_model(
            model_name='boxy',
            model_xml=boxy,
            robot_namespace='/boxy',
            initial_pose=Pose(position=Point(1, 1, 0)),
            reference_frame='world',
        )
        msg3 = rospy.wait_for_message("/os1_cloud_node/points", PointCloud2, timeout=1)

        # This time, the scene has changed: the data should be different
        self.assertNotEqual(msg2.data, msg3.data)


if __name__ == '__main__':
    rosunit.unitrun('ouster_os1_64', 'test_ouster', OusterTest)
