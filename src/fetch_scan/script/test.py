import rospy
import numpy as np
import tf
import tf.transformations as tf_trans
import tf2_ros
import ros_numpy
import tf2_geometry_msgs
from sensor_msgs.msg import PointCloud2
from scipy.spatial.transform import Rotation as R
from sensor_msgs import point_cloud2

def crop_point_cloud(point_cloud, min_bound=np.array([0.3,-0.5,0.2]), max_bound=np.array([1.3,0.5,1.2]), return_mask=False):
    """
    Crops a point cloud to the points that fall within a given 3D bounding box.

    :param point_cloud: numpy array of shape (N, 3) representing the point cloud
    :param min_bound: numpy array of shape (3,) representing the minimum corner of the bounding box
    :param max_bound: numpy array of shape (3,) representing the maximum corner of the bounding box
    :return: numpy array of the cropped point cloud
    """
    # Check if points are within the bounding box
    points = point_cloud[:, 0:3]
    in_bounds = np.all((points >= min_bound) & (points <= max_bound), axis=1)

    # Extract points within the bounding box
    cropped_cloud = point_cloud[in_bounds]

    return cropped_cloud

def cloud_callback(cloud_msg):
    
    # save RGBXYZ: hx
    pc = ros_numpy.numpify(cloud_msg)
    pc = ros_numpy.point_cloud2.split_rgb_field(pc)
    # xyz
    points = np.zeros((pc.shape[0], 3), dtype=np.float32)
    points[:, 0] = pc['x']
    points[:, 1] = pc['y']
    points[:, 2] = pc['z']
    
    # rgb
    rgb = np.zeros((pc.shape[0], 3))
    rgb[:,0] = pc['r']
    rgb[:,1] = pc['g']
    rgb[:,2] = pc['b']
    # print(rgb[0:10])
    
    rgbpoints = np.concatenate((points, rgb), axis=1)
    print("rgbpoints", rgbpoints.shape)
    
    try:
        print(cloud_msg.header.frame_id)
        # Lookup the transformation from camera frame to base frame
        trans = tf_buffer.lookup_transform('base_link', cloud_msg.header.frame_id, rospy.Time(0))
        
        rotation = R.from_quat([
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w
        ]).as_dcm()
        
        translation = np.array([
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z
        ])
        
        points = rgbpoints[:, 0:3].copy()
        # print(points.shape)
        points = np.dot(points, rotation.T) + translation
        
        rgbpoints[:, 0:3] = points
        np.savetxt("/home/fetch/scan/scan_ws/all.txt", rgbpoints)
        
        rgbpoints = crop_point_cloud(rgbpoints)
        print("rgbpoints after crop", rgbpoints.shape)
        
        
        np.savetxt("/home/fetch/scan/scan_ws/crop.txt", rgbpoints)
        print("save ***********8")
        
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr("TF error: %s", e)

rospy.init_node('pointcloud_transform_node')
rospy.Rate(0.1)

tf_buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tf_buffer)

cloud_sub = rospy.Subscriber('/points2', PointCloud2, cloud_callback)

rospy.spin()