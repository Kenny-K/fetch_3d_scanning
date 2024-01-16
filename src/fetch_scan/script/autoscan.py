#!/usr/bin/env python
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import rospy
import tf.transformations as tf_trans
import numpy as np

from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image, PointCloud2

def crop_point_cloud(point_cloud, min_bound, max_bound):
    """
    Crops a point cloud to the points that fall within a given 3D bounding box.

    :param point_cloud: numpy array of shape (N, 3) representing the point cloud
    :param min_bound: numpy array of shape (3,) representing the minimum corner of the bounding box
    :param max_bound: numpy array of shape (3,) representing the maximum corner of the bounding box
    :return: numpy array of the cropped point cloud
    """
    # Check if points are within the bounding box
    in_bounds = np.all((point_cloud >= min_bound) & (point_cloud <= max_bound), axis=1)

    # Extract points within the bounding box
    cropped_cloud = point_cloud[in_bounds]

    return cropped_cloud

def pt_callback(msg):
    global oct_points
    
    gen = point_cloud2.read_points(msg, field_names=('x','y','z'),
                                   skip_nans=True)
    oct_points = np.asarray(list(gen))
    
    
def main(args):
    
    global oct_points
    
    # Initialize the moveit_commander and a rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('fetch_scan')
    rospy.Rate(args['ros_rate'])
    
    # Instantiate moveit_commander object
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("arm_with_torso")
    planning_scene = moveit_commander.PlanningSceneInterface()
    
    # Set motion vel/acc
    group.set_max_acceleration_scaling_factor(0.1)
    group.set_max_velocity_scaling_factor(0.1)
    
    # read the initial
    
    rospy.wait_for_message("/octomap_point_cloud_centers", PointCloud2)
    rospy.Subscriber("/octomap_point_cloud_centers", PointCloud2, pt_callback)
    
    rospy.sleep(1.0)
    print(oct_points.shape)
    crop_bound = np.array(args['roi_bbox'])
    crop_pcd = crop_point_cloud(oct_points, min_bound=crop_bound[:3], max_bound=crop_bound[3:])
    print(crop_pcd.shape)
    np.savetxt('/home/fetch/scan/points.txt', oct_points)
    np.savetxt('/home/fetch/scan/cropped.txt', crop_pcd)   
    
    c_pose = group.get_current_pose().pose
    print(c_pose)

    # # Set the orientation target
    # # Example: Setting the end effector to be level with the ground
    # roll, pitch, yaw = 0, 0, 0  # adjust these values as needed
    # quaternion = tf_trans.quaternion_from_euler(roll, pitch, yaw)
    # group.set_orientation_target(quaternion)

    # # Plan to the new orientation
    # plan = group.plan()

    # # Execute the plan
    # group.execute(plan, wait=True)

    # When finished shut down moveit_commander
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    args_dict = {
        'roi_bbox':[0.5, -1.0, 0.0, 2.5, 1.0, 2.0],
        'vox_res': 0.05,
        'ros_rate': 1.0
    }
    
    main(args_dict)
    
    # position: 
    # x: 0.182646075416
    # y: 0.494772373706
    # z: 1.12583529555
    # orientation: 
    # x: 0.0851490797969
    # y: 0.325462992198
    # z: -0.291387478914
    # w: 0.895498080429
    
    # Set the reference frame for pose targets
    # reference_frame = "/base_link"
    # group.set_pose_reference_frame(reference_frame)