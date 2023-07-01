#!/usr/bin/env python3
import time
import numpy as np
from typing import List
import ros_numpy
import rospy
from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion
from jsk_recognition_msgs.msg import BoundingBox
from sensor_msgs.msg import PointCloud2
from skrobot.coordinates import Coordinates
from skrobot.coordinates.math import matrix2quaternion, rpy2quaternion, wxyz2xyzw
from utils import PlanerBoundingBox
from message_filters import ApproximateTimeSynchronizer, Subscriber

from voxbloxpy import EsdfMap, GridSDF
from voxbloxpy.ros import EsdfNode, EsdfNodeConfig

rospy.init_node("pointcloud_subscriber", anonymous=True)


msg = [None]
def pose_arr_callback(msg_poses):
    msg[0] = msg_poses

grid_sdf_list: List[GridSDF] = []

def hook(esdf_map: EsdfMap):
    # this function might be too heavy.
    # I just do the following to make an animation. but do not do that
    # in the realtime application!
    ts = time.time()
    info = esdf_map.get_voxel_info()
    measure_grid = info.get_boundary_grid(grid_size=0.2)
    grid_sdf = esdf_map.get_grid_sdf(measure_grid, create_itp_lazy=True, fill_value=1.0)
    grid_sdf_list.append(grid_sdf)
    te = time.time()
    rospy.loginfo("elapsed time for getting gridsdf {} sec".format(te - ts))

config = EsdfNodeConfig(point_cloud_topic="/docker/ExtractIndices/output", world_frame="base_link", voxel_size=0.02)
node = EsdfNode(config, hook=hook)
time.sleep(5)  # stop node after a while
node.callback_running = False
print("stop esdf node")
posearr_sub = rospy.Subscriber("/reaching_poses", PoseArray, pose_arr_callback)

while msg[0] is not None:
    time.sleep(1)
print("recv")

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(node._input_pts[:, 0], node._input_pts[:, 1], node._input_pts[:, 2])

gridsdf = node.esdf.get_grid_sdf()
gridsdf.render_volume()
# plt.show()




