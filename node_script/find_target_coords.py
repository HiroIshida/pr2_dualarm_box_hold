#!/usr/bin/env python3
import numpy as np
import pickle
import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, PoseArray
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from utils import PlanerBoundingBox
from jsk_recognition_msgs.msg import BoundingBox
from skrobot.coordinates.math import rpy2quaternion, matrix2quaternion, wxyz2xyzw
from skrobot.coordinates import Coordinates


rospy.init_node('pointcloud_subscriber', anonymous=True)
pub_box = rospy.Publisher("detected_box", BoundingBox, queue_size=100)
pub_reaching_poses = rospy.Publisher("reaching_poses", PoseArray, queue_size=100)


def skcoords_to_rospose(coords: Coordinates) -> Pose:
    quat = wxyz2xyzw(matrix2quaternion(coords.rotation))
    pose = Pose(Point(*coords.translation), Quaternion(*quat))
    return pose


def pointcloud_callback(msg: PointCloud2):
    pts = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)

    z_highest = np.max(pts[:, 2])
    indices_top = np.where(pts[:, 2] > z_highest - 0.05)[0]
    pts_boxtop2d = pts[indices_top, :2]
    box2d = PlanerBoundingBox.from_points(pts_boxtop2d)

    indices_upper = np.where(pts[:, 2] > z_highest - 0.3)[0]
    z_upper_lowest = np.min(pts[indices_upper, 2])

    box = BoundingBox()
    box.header = msg.header
    pose = Pose()
    pose.position.x = box2d.center[0]
    pose.position.y = box2d.center[1]
    pose.position.z = 0.5 * (z_upper_lowest + z_highest)
    quat = rpy2quaternion([box2d.angle, 0, 0])
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z = quat
    box.pose = pose
    box.dimensions.x = np.linalg.norm(box2d.corners[1] - box2d.corners[0])
    box.dimensions.y = np.linalg.norm(box2d.corners[2] - box2d.corners[1])
    box.dimensions.z = z_highest - z_upper_lowest
    pub_box.publish(box)

    print("published")

    table_ex = np.array([1., 0, 0])
    table_ey = np.array([0, 1., 0])

    box_extent = [box.dimensions.x, box.dimensions.y, box.dimensions.z]
    co_box = Coordinates([pose.position.x, pose.position.y, pose.position.z])
    co_box.rotate(box2d.angle, "z")
    box_ex = co_box.rotation[:, 0]
    box_ey = co_box.rotation[:, 1]

    # fetched from rpbench (private)
    margin = 0.05
    if table_ex.dot(box_ex) < 1 / np.sqrt(2):
        x_half_width = 0.5 * box_extent[0]
        pos_target1 = co_box.worldpos() + box_ex * (x_half_width + margin)
        pos_target2 = co_box.worldpos() - box_ex * (x_half_width + margin)
        right_co = Coordinates(pos=pos_target1, rot=co_box.rotation)
        left_co = Coordinates(pos=pos_target2, rot=co_box.rotation)
        if table_ey.dot(box_ex) < 0.0:
            right_co.rotate(+np.pi * 0.5, "z")
            left_co.rotate(+np.pi * 0.5, "z")
        else:
            right_co.rotate(-np.pi * 0.5, "z")
            left_co.rotate(-np.pi * 0.5, "z")
    else:
        y_half_width = 0.5 * box_extent[1]
        pos_target1 = co_box.worldpos() + box_ey * (y_half_width + margin)
        pos_target2 = co_box.worldpos() - box_ey * (y_half_width + margin)
        right_co = Coordinates(pos=pos_target1, rot=co_box.rotation)
        left_co = Coordinates(pos=pos_target2, rot=co_box.rotation)

    pose_array = PoseArray(header = msg.header)
    pose_array.poses.append(skcoords_to_rospose(right_co))
    pose_array.poses.append(skcoords_to_rospose(left_co))
    pub_reaching_poses.publish(pose_array)
    print("published")


rospy.Subscriber("/docker/tf_transform_cloud/output", PointCloud2, pointcloud_callback)
rospy.spin()
