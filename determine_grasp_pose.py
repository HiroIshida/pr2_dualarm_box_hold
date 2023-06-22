
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def pointcloud_callback(msg):
    X = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
    print("Received point cloud with shape:", X.shape)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(X[:, 0], X[:, 1], X[:, 2])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()

def pointcloud_subscriber():
    rospy.init_node('pointcloud_subscriber', anonymous=True)
    name = "/docker/detic_cluster_point_indices_decomposer/debug_output"
    rospy.Subscriber(name, PointCloud2, pointcloud_callback)
    rospy.spin()

if __name__ == '__main__':
    pointcloud_subscriber()
