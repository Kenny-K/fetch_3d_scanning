#!/usr/bin/env python
import sys
import moveit_commander
import moveit_msgs.msg
import rospy
import tf2_ros
import tf.transformations as tf_trans
import numpy as np
import ros_numpy


from moveit_python import PlanningSceneInterface
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image, PointCloud2
from time import time
from scipy.spatial import KDTree

from scipy.spatial.transform import Rotation as R

def compute_normals(point_cloud, k=5):
    """
    Computes the normal vector for each point in the point cloud.

    :param point_cloud: numpy array of shape (N, 3) representing the point cloud
    :param k: number of nearest neighbors to consider for each point
    :return: numpy array of shape (N, 3) representing the normal vectors
    """

    tree = KDTree(point_cloud)
    normals = []

    for point in point_cloud:
        # Find k nearest neighbors
        _, indices = tree.query(point, k=k + 1)
        neighbors = point_cloud[indices[1:]]  # Exclude the point itself

        # Compute covariance matrix
        cov_mat = np.cov(neighbors, rowvar=False)

        # Compute eigenvalues and eigenvectors
        eigenvalues, eigenvectors = np.linalg.eigh(cov_mat)

        # Eigenvector with smallest eigenvalue
        normal = eigenvectors[:, np.argmin(eigenvalues)]

        normals.append(normal)

    return np.array(normals)

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

def voxel_grid(point_cloud, voxel_size, min_coords, max_coords, camera_pos=None):
    """
    Converts a point cloud to a voxel grid.

    :param point_cloud: numpy array of shape (Ntuple(voxel_coords)] = True, 3) representing the point cloud
    :param voxel_size: size of each voxel
    :return: voxel grid and the mapping from voxel coordinates to point indices
    """
    # min_coords = np.min(point_cloud, axis=0)
    # max_coords = np.max(point_cloud, axis=0)

    # Compute dimensions of the grid
    dims = np.ceil((max_coords - min_coords) / voxel_size).astype(np.int32)
    voxel_coords = np.floor((point_cloud - min_coords) / voxel_size).astype(np.int32)

    grid_center = np.unique(voxel_coords, axis=0)
    center_position = (grid_center + 0.5) * voxel_size + min_coords
    
    normals = compute_normals(center_position)
    
    voxel_center_pos = np.zeros((dims[0], dims[1], dims[2], 3))
    voxel_center_pos[grid_center[:,0], grid_center[:,1], grid_center[:,2]] = center_position

    voxel_normal = np.zeros((dims[0], dims[1], dims[2], 3))
    voxel_normal[grid_center[:,0], grid_center[:,1], grid_center[:,2]] = normals
    
    voxel_dist = np.ones((dims[0], dims[1], dims[2], 1)) * -1
    if camera_pos is not None:
        dist = np.linalg.norm(center_position - camera_pos, axis=1, keepdims=True)
        voxel_dist[grid_center[:,0], grid_center[:,1], grid_center[:,2]] = dist
        
    voxel_grid = np.concatenate([voxel_center_pos, voxel_normal, voxel_dist], axis=-1)
        
    return voxel_grid

def pt_callback(msg):
    global oct_points
    if oct_points is None:
        gen = point_cloud2.read_points(msg, field_names=('x','y','z'),
                                    skip_nans=True)
        oct_points = np.asarray(list(gen))
    

def rgbd_callback(point_cloud):
    global save_scan, tf_buffer
    
    if save_scan:
        # Convert PointCloud2 to Numpy array
        pc = ros_numpy.numpify(point_cloud)
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
        
        rgbd_point_cloud = np.concatenate((points, rgb), axis=1)
        print("rgbd_point_cloud", rgbd_point_cloud.shape)
        
        points = rgbd_point_cloud[:, 0:3].copy()
        
        
        # calibration to base_link
        print(point_cloud.header.frame_id)
        # Lookup the transformation from camera frame to base frame
        trans = tf_buffer.lookup_transform('base_link', point_cloud.header.frame_id, rospy.Time(0))
        
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
        
        points = np.dot(points, rotation.T) + translation
        rgbd_point_cloud[:, 0:3] = points
        
        rgbd_point_cloud = crop_point_cloud(rgbd_point_cloud, min_bound=np.array([0.3,-0.5,0.2]), max_bound=np.array([1.3,0.5,1.2]), return_mask=False)
        print("rgbd_point_cloud after crop", rgbd_point_cloud.shape)
        timestamp = str(time()).split('.')[0]
        scan_name = "/home/fetch/scan/scan_ws/scan_{}.txt".format(timestamp)
        np.savetxt(scan_name, rgbd_point_cloud)
        rospy.loginfo("Saving the scan at {}".format(scan_name))
        save_scan = False

def direction_vector_to_euler_angles(direction_vector):
    """
    Convert a direction vector to Euler angles (roll, pitch, yaw).

    :param direction_vector: A 3D unit direction vector.
    :return: A tuple of (roll, pitch, yaw) in radians.
    """
    # Normalize the direction vector
    direction_vector = direction_vector / np.linalg.norm(direction_vector)

    # Calculate yaw and pitch
    yaw = np.arctan2(direction_vector[1], direction_vector[0])
    pitch = np.arcsin(-direction_vector[2])

    # Assuming roll = 0 as we don't have information about it in a single direction vector
    roll = 0

    return roll, pitch, yaw
    
def main(args):
    
    global oct_points, save_scan, tf_buffer
    save_scan = False
    
    # Initialize the moveit_commander and a rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('fetch_scan')
    rospy.Rate(args['ros_rate'])
    
    # Instantiate moveit_commander object
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("arm")
    planning_scene = PlanningSceneInterface("base_link")
    
    # Set motion vel/acc
    group.set_max_acceleration_scaling_factor(0.1)
    group.set_max_velocity_scaling_factor(0.1)
    
    group.set_end_effector_link("gripper_link")
    
    # move to start pose
    init_pose = Pose()
    init_pose.position.x = args['init_pose']['x']
    init_pose.position.y = args['init_pose']['y']
    init_pose.position.z = args['init_pose']['z']
    init_pose.orientation.x = args['init_pose']['wx']
    init_pose.orientation.y = args['init_pose']['wy']
    init_pose.orientation.z = args['init_pose']['wz']
    init_pose.orientation.w = args['init_pose']['w']
    group.set_pose_target(init_pose)
    print(type(init_pose.orientation))
    
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    
    # read the initial
    oct_points = None
    rospy.wait_for_message("/octomap_point_cloud_centers", PointCloud2)
    rospy.Subscriber("/octomap_point_cloud_centers", PointCloud2, pt_callback)
    rospy.wait_for_message("/points2", PointCloud2)
    rospy.Subscriber("/points2", PointCloud2, rgbd_callback)
    rospy.sleep(2.0)
    
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    
    # Get reset octomap service
    # rospy.wait_for_service("/octomap_server/reset", Empty)
    # octo_reset = rospy.ServiceProxy("/octomap_server/reset", Empty)
    # octo_reset()
    
    # crop the Region of Interest
    crop_bound = np.array(args['roi_bbox'])
    rospy.loginfo("Get complete scene point cloud of shape {}".format(oct_points.shape))
    crop_pcd = crop_point_cloud(oct_points, min_bound=crop_bound[:3], max_bound=crop_bound[3:])
    
    rospy.loginfo("Get initial scene point cloud of shape {}".format(crop_pcd.shape))
    
    # add the table into planning scene
    table_cube_center = crop_pcd[crop_pcd[:,2] > 0.3]
    for name in planning_scene.getKnownCollisionObjects():
        planning_scene.removeCollisionObject(name, True)
    rospy.loginfo("Waiting for Planning Scene to clean up ...")
    rospy.sleep(8)
    assert len(planning_scene.getKnownCollisionObjects()) == 0
    for i, cube_center in enumerate(table_cube_center):
        planning_scene.addCube("table_cube_{}".format(i), args['vox_res'], cube_center[0], cube_center[1], cube_center[2])
    rospy.loginfo("Waiting for Planning Scene to add collision ...")
    rospy.sleep(8)
    planning_scene.waitForSync(8.0) # wait for adding collision
    rospy.loginfo("Adding {}/{} cube blocks into the scene".format(
        len(planning_scene.getKnownCollisionObjects()), table_cube_center.shape[0]))
    
    
    # build init voxel
    cur_pose = group.get_current_pose().pose.position
    cur_xyz = np.array([cur_pose.x, cur_pose.y, cur_pose.z])
    voxels = voxel_grid(crop_pcd, voxel_size=args['vox_res'], 
               min_coords=crop_bound[:3], max_coords=crop_bound[3:], camera_pos=cur_xyz)
    rospy.loginfo("Build the initial voxel cells")
    
    # save init scan
    save_scan = True
    rospy.wait_for_message("/points2", PointCloud2)
    
    # save init octomap
    timestamp = str(time()).split('.')[0]
    scan_name = "/home/fetch/scan/scan_ws/octo_{}.txt".format(timestamp)
    np.savetxt(scan_name, crop_pcd)
    rospy.loginfo("Saving the scan at {}".format(scan_name))
    
    # raise ValueError("stop")
    
    print(voxels[:,:,:,6][voxels[:,:,:,6]>0].min(), voxels[:,:,:,6].max())
    voxels[:,:,:,6][voxels[:,:,:,6] > 1.0] = -1.
    cnt = 0
    complete_flag = False
    success_cnt = 0
    while True:
        if complete_flag:
            print("Success!")
            break
        if success_cnt > 10:
            break
        while True:
            # select the farthest point (occupied)
            voxel_not_none_index = np.where(voxels[:, :, :, 6]!=-1.)
            
            if len(voxel_not_none_index[0]) == 0:
                complete_flag = True
                print("Success!")
                break
            
            voxel_not_none_index_array = np.concatenate((np.expand_dims(voxel_not_none_index[0], axis=1), np.expand_dims(voxel_not_none_index[1], axis=1), np.expand_dims(voxel_not_none_index[2], axis=1)), 1)
            # print("voxel_not_none_index_array", voxel_not_none_index_array.shape) # [n, 3] occpied points voxel index
            
            voxel_not_none = voxels[voxel_not_none_index]
            print("voxel_not_none", voxel_not_none.shape)
            max_id = np.argmax(voxel_not_none[:,6])
            
            max_id_voxel = voxel_not_none_index_array[max_id] # occupied voxels with max distance [3]
            # print("max_id_voxel", max_id_voxel)
            
            max_feat = voxel_not_none[max_id]
            max_center, max_normal = max_feat[:3], max_feat[3:6]
            roll, pitch, yaw = direction_vector_to_euler_angles(-max_normal)     #TODO
            max_position = max_center + 0.2 * max_normal / np.linalg.norm(max_normal)
            
            print("max_center, max_position, max_normal", max_center, max_position, max_normal)
            
            
            # select the farthest point
            # max_id = np.unravel_index(np.argmax(voxels[:,:,:,6]), voxels.shape[:-1])
            # max_feat = voxels[max_id[0], max_id[1], max_id[2], :]
            # max_center, max_normal = max_feat[:3], max_feat[3:6]
            
            
            # roll, pitch, yaw = direction_vector_to_euler_angles(-max_normal)
            # max_position = max_center + 0.2 * max_normal / np.linalg.norm(max_normal)
            
            # print("max_center, max_position", max_center, max_position)
            
            max_q = tf_trans.quaternion_from_euler(roll, pitch, yaw)
            
            target_pose = Pose()
            target_pose.position.x = max_position[0]
            target_pose.position.y = max_position[1]
            target_pose.position.z = max_position[2]
            target_pose.orientation.x = max_q[0]
            target_pose.orientation.y = max_q[1]
            target_pose.orientation.z = max_q[2]
            target_pose.orientation.w = max_q[3]
            group.set_pose_target(target_pose)
            plan = group.plan()
            
            if len(plan.joint_trajectory.points) > 0: 
                rospy.loginfo("Find next scan.")
                print(target_pose)
                group.execute(plan, wait=True)
                group.stop()
                group.clear_pose_targets()
                
                # save init scan
                save_scan = True
                rospy.wait_for_message("/points2", PointCloud2)
                success_cnt += 1
                
                # mark completed voxel as -1
                # voxels[max_id_voxel[0], max_id_voxel[1], max_id_voxel[2]] = 0
                # voxels[max_id_voxel[0], max_id_voxel[1], max_id_voxel[2], 6] = -1.

                max_x_bound = np.minimum(max_id_voxel[0] + 2, 199)
                max_y_bound = np.minimum(max_id_voxel[1] + 2, 199)
                max_z_bound = np.minimum(max_id_voxel[2] + 2, 199)

                min_x_bound = np.minimum(max_id_voxel[0] - 2, 0)
                min_y_bound = np.minimum(max_id_voxel[1] - 2, 0)
                min_z_bound = np.minimum(max_id_voxel[2] - 2, 0)

                voxels[min_x_bound:max_x_bound+1, min_y_bound:max_y_bound+1, min_z_bound:max_z_bound+1] = 0
                voxels[min_x_bound:max_x_bound+1, min_y_bound:max_y_bound+1, min_z_bound:max_z_bound+1, 6] = -1.


                # mask the NNs also as -1
                # range x, y, z translation
                # resolution: [200, 200, 200]
                # resolution = 199
                # trans = [
                #     [-2, 2], 
                #     [-2, 2],
                #     [-2, 2]
                # ]

                # for x_i in range(trans[0][0], trans[0][1]):
                #     for y_i in range(trans[1][0], trans[1][1]):
                #         for z_i in range(trans[2][0], trans[2][1]):
                #             trans_index = [x_i, y_i, z_i]
                #             max_id_voxel += trans_index
                #             if not ((max_id_voxel > resolution).any() or (max_id_voxel < 0).any()):
                #                 voxels[max_id_voxel[0], max_id_voxel[1], max_id_voxel[2]] = 0
                #                 voxels[max_id_voxel[0], max_id_voxel[1], max_id_voxel[2], 6] = -1.
                                
                break
                
            else:
                rospy.loginfo("Find voxel {} unreachable. Mark its distance as -1.".format(max_center.tolist()))
                # voxels[max_id[0], max_id[1], max_id[2], 6] = -1
                
                # revise for not none voxel
                # new the occupied voxels with max distance
                # print("max_id", max_id)
                # print(voxel_not_none[max_id])
                # print(voxels[max_id_voxel[0], max_id_voxel[1], max_id_voxel[2]])
                voxels[max_id_voxel[0], max_id_voxel[1], max_id_voxel[2]] = 0
                voxels[max_id_voxel[0], max_id_voxel[1], max_id_voxel[2], 6] = -1.
                # print(voxels[max_id_voxel[0], max_id_voxel[1], max_id_voxel[2]])
                # print("reset unreachable voxel", voxels[max_id_voxel[0], max_id_voxel[1], max_id_voxel[2]])
            
            if cnt > 75: break
            cnt += 1
    
    # save the scan
    save_scan = True
    rospy.sleep(1.0)    
    
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
        # 'roi_bbox':[0.3, -1.0, 0.0, 2.3, 1.0, 2.0],
        'roi_bbox':[0.25, -0.5, 0.1, 1.25, 0.5, 1.1],
        'vox_res': 0.05,
        'ros_rate': 2.0,
        'cam_dist': 0.2,
        'init_pose': {'x': 0.0851,
                      'y': 0.4947,
                      'z': 1.125,
                      'wx': 0.0851,
                      'wy': 0.3254,
                      'wz': -0.291,
                      'w': 0.895}
    }
    
    main(args_dict)
    
    #high
    # position: 
    # x: 0.182646075416
    # y: 0.494772373706
    # z: 1.12583529555
    # orientation: 
    # x: 0.0851490797969
    # y: 0.325462992198
    # z: -0.291387478914
    # w: 0.895498080429
    
    #low
    # position: 
    # x: 0.261772613861
    # y: -0.0449500810755
    # z: 0.536347437875
    # orientation: 
    # x: -0.0165934091551
    # y: -0.0581931582812
    # z: 0.112410471168
    # w: 0.991817574493
        
    # Set the reference frame for pose targets
    # reference_frame = "/base_link"
    # group.set_pose_reference_frame(reference_frame)