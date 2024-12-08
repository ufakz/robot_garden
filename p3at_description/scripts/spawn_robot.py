#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import math

def euler_to_quaternion(roll, pitch, yaw):
    # Convert Euler angles (in radians) to quaternion
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)

    return qx, qy, qz, qw

def wait_for_gazebo():
    rospy.init_node('wait_for_gazebo')
    rospy.wait_for_service('/gazebo/spawn_urdf_model', timeout=60)  # Wait for spawn_urdf_model service
    rospy.loginfo("Gazebo is ready to receive models!")

def get_spawn_params():
    # Retrieve parameters from the ROS parameter server
    spawn_x = rospy.get_param('spawn_x', 0.0)
    spawn_y = rospy.get_param('spawn_y', 0.0)
    spawn_z = rospy.get_param('spawn_z', 0.0)
    spawn_roll = rospy.get_param('spawn_roll', 0.0)
    spawn_pitch = rospy.get_param('spawn_pitch', 0.0)
    spawn_yaw = rospy.get_param('spawn_yaw', 0.0)

    return spawn_x, spawn_y, spawn_z, spawn_roll, spawn_pitch, spawn_yaw

def spawn_model():
    try:
        pose = Pose()

        # Retrieve parameters from the ROS parameter server
        x, y, z, roll, pitch, yaw = get_spawn_params()
        # Convert Euler angles to quaternion
        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)

        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        robot_description = rospy.get_param('/robot_description')
        spawn_model('pioneer3at', robot_description, "", pose, "world")
        rospy.loginfo("Robot spawned successfully!")
    except rospy.ServiceException as e:
        rospy.logerr("Spawn service call failed: %s" % e)

if __name__ == '__main__':
    wait_for_gazebo()  # Wait for Gazebo to be ready
    spawn_model()