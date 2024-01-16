#!/usr/bin/env python

import rospy
import copy
import sys
import yaml
import pickle
import pcl
import math 
import numpy as np

from tf.transformations import quaternion_from_euler, quaternion_matrix
from tf import TransformListener

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3
from visualization_msgs.msg import Marker
from laser_geometry import LaserProjection
from std_msgs.msg import ColorRGBA

from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import AttachedCollisionObject
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from moveit_commander import RobotCommander, MoveGroupCommander

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal






# slowdown the callback frequency
def callback_throttle(msg):
    global last_callback_time
    if rospy.Time.now() - last_callback_time > rospy.Duration(1.0):
        last_callback_time = rospy.Time.now()
        check_left_line(msg)


# check left line with laser scan
def check_left_line(msg):
    
    global fit_y
    global if_test
    global hand_quat 
    global last_callback_time 
    global whiteboard_pose 
    global whiteboard_shape 
    global line_coefficients 
    global line_detected
    
    
    ranges = msg.ranges
    
    min_left_angle = -3.14
    max_left_angle = 0.0
    
    first_range = True
    
    filtered_ranges = []
    left_scan = LaserScan()
    left_scan.header = msg.header
    left_scan.angle_min = min_left_angle
    left_scan.angle_max = max_left_angle
    left_scan.angle_increment = msg.angle_increment
    left_scan.time_increment = msg.time_increment
    left_scan.scan_time = msg.scan_time
    left_scan.range_min = msgetect line.range_min
    left_scan.range_max = msg.range_max
    
    for i in range(len(ranges)):
        angle = msg.angle_min + i * msg.angle_increment
        
        if angle > min_left_angle and angle < max_left_angle:
            if float('-inf') < ranges[i] < float('inf'):
                if first_range:
                    first_range = False
                    left_scan.angle_min = angle
                # show with marker
                filtered_ranges.append(ranges[i])
                left_scan.angle_max = angle
   
    left_scan.ranges = filtered_ranges
    leftLaserPublisher.publish(left_scan)
  
    # convert laser scan to point cloud
    laser_projector = LaserProjection()
    cloud = laser_projector.projectLaser(left_scan)
  
    
    # use pcl to detect line
    pcl_cloud = pcl.PointCloud()
    points = []
    for p in point_cloud2.read_points(cloud, skip_nans=True):
        points.append([p[0], p[1], p[2]])
    pcl_cloud.from_list(points)
    
    seg = pcl_cloud.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_LINE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(0.01)
    indices, line_coefficients = seg.segment()
    
    # extract line endpoints
    if len(indices) > 0 :
        p1 = pcl_cloud[indices[0]]
        p2 = pcl_cloud[indices[-5]]    


    # Create a Marker message
    marker = Marker()
    marker.header.frame_id = "laser_link"
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.01
    marker.color.r = 1.0
    marker.color.a = 1.0

    point1 = Point()
    point1.x = p1[0] 
    point1.y = p1[1]
    point1.z = p1[2]
    point2 = Point()
    point2.x = p2[0]
    point2.y = p2[1]
    point2.z = p2[2]
    

    marker.points.append(point1)
    marker.points.append(point2)

    # Publish the Marker message
    marker_pub.publish(marker)
    
    
    # add cube based on line
    line_cube = CollisionObject()
    line_cube.header.frame_id = "laser_link"
    line_cube.id = "line_cube"
    line_cube.operation = line_cube.ADD
    
    whiteboard_shape.type = whiteboard_shape.BOX
    
    # if the distance between the line and robot is too far, just create a fake wall for testing
    if abs(p1[1]) > fit_y and abs(p2[1]) > fit_y:
        rospy.loginfo("p1[1]:%f, p2[1]:%f,  line is too far, fake wall created", p1[1], p2[1])
        if_test = True
        whiteboard_pose.position.x = 0 
        whiteboard_pose.position.y = -fit_y
        whiteboard_pose.position.z = -1
        
        
        # quat = quaternion_about_axis(angle_value, line_direction)
        quat = quaternion_from_euler(0, 0, math.pi/6)
        whiteboard_pose.orientation.x = quat[0]
        whiteboard_pose.orientation.y = quat[1]
        whiteboard_pose.orientation.z = quat[2]
        whiteboard_pose.orientation.w = quat[3]
        
        whiteboard_shape.dimensions = [1.3, 0.1, 2]
    else:
        if_test = False
        whiteboard_pose.position.x = (p1[0] + p2[0])/2 
        whiteboard_pose.position.y = (p1[1] + p2[1])/2
        whiteboard_pose.position.z = -1
        
        
        # quat = quaternion_about_axis(angle_value, line_direction)
        quat = quaternion_from_euler(0, 0, math.atan2(line_coefficients[4], line_coefficients[3]))
        whiteboard_pose.orientation.x = quat[0]
        whiteboard_pose.orientation.y = quat[1]
        whiteboard_pose.orientation.z = quat[2]
        whiteboard_pose.orientation.w = quat[3]
  
   
    # p1 and p2 is not the exactly endpoints of whiteboard, so we need to add a padding value
        whiteboard_shape.dimensions = [abs(p1[0] - p2[0])+0.4, 0.1 , 2]
    
    line_cube.primitives.append(whiteboard_shape)
    line_cube.primitive_poses.append(whiteboard_pose)
    planning_scene.removeCollisionObject("line_cube")
    collision_pub.publish(line_cube)
    planning_scene.waitForSync(5)
    line_detected = True
        

# write a move function to transfer 
def move(move_distance, laser_pose):
    assert len(move_distance) == 3
    global whiteboard_pose

    laser_matrix = quaternion_matrix([whiteboard_pose.orientation.x, whiteboard_pose.orientation.y, whiteboard_pose.orientation.z, whiteboard_pose.orientation.w])
    laser_position_offset=laser_matrix[:3, :3].dot(move_distance)
    laser_pose.pose.position.x+=laser_position_offset[0]
    laser_pose.pose.position.y+=laser_position_offset[1]
    laser_pose.pose.position.z+= laser_position_offset[2]
    
    base_pose = listener.transformPose("base_link",laser_pose)
    return laser_pose, base_pose


if __name__ == '__main__':
    
    rospy.init_node("wipe")   
    # rospy.sleep(4)
    # client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # client.wait_for_server()
    # rospy.loginfo("lalalalallalalallalalallalalal3")

    # # Create a MoveBaseGoal with the desired pose (1cm forward)
    # goal = MoveBaseGoal()
    # goal.target_pose.header.frame_id = 'base_link'
    # goal.target_pose.pose = Pose(position=Point(0.1, 0, 0))

    # # Send the goal to the move_base action server
    # client.send_goal(goal)
    # client.wait_for_result()
    
    
    white_board_max_height = 1.7
    white_board_height = 0.75
    white_board_min_height = white_board_max_height - white_board_height
    wipe_range = 0.08
    fit_y = 0.7
    constraint_position_offset_value = 0.2
    
    
    
    # define the global variable
    if_test = False
    hand_quat = quaternion_from_euler(math.pi/2, 0, math.pi/2)
    last_callback_time = rospy.Time.now()
    whiteboard_pose = Pose()
    whiteboard_shape = SolidPrimitive()
    line_coefficients = []
    
   
    # Create move group interface for a fetch robot
    move_group = MoveGroupCommander("arm_with_torso")
    robot = RobotCommander()
    move_group.set_end_effector_link("gripper_link")
    move_group.set_max_velocity_scaling_factor(0.1)
    move_group.set_planning_time(10);
    

    # Define ground plane
    # This creates objects in the planning scene that mimic the ground
    # If these were not in place gripper could hit the ground
    planning_scene = PlanningSceneInterface("base_link")
    planning_scene.removeCollisionObject("my_front_ground")
    planning_scene.removeCollisionObject("my_back_ground")
    planning_scene.removeCollisionObject("my_right_ground")
    planning_scene.removeCollisionObject("my_left_ground")
    planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
    planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
    planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
    planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)
    
    planning_scene.addCube("back_wall", 2, -1.3, 0, 1)
    
    # attach the eraser     
    
    planning_scene.attachBox("eraser", size_x=0.06, size_y=0.08, size_z=0.28, x=0.07, y=0, z=0, link_name='gripper_link', touch_links=['l_gripper_finger_link', 'r_gripper_finger_link'])    
    # planning_scene.attachBox("eraser", size_x=0.06, size_y=0.08, size_z=0.28, x=0.12, y=0, z=0, link_name='gripper_link', touch_links=['l_gripper_finger_link', 'r_gripper_finger_link'])    

    planning_scene.waitForSync(5)
    
    
    # subscriber and publisher 
    rospy.Subscriber("/base_scan", LaserScan, callback_throttle)
    leftLaserPublisher = rospy.Publisher('/left_laser', LaserScan, queue_size=10)
    marker_pub = rospy.Publisher('/line_marker', Marker, queue_size=10)
    constraint_mark_pub = rospy.Publisher('/constraint_maker', Marker, queue_size=10)
    collision_pub = rospy.Publisher('/collision_object',CollisionObject, queue_size=10)
    listener = TransformListener()
    listener.waitForTransform("laser_link", "base_link",  rospy.Time(), rospy.Duration(4.0))
    
    
    line_detected = False
    while not line_detected:
        rospy.sleep(1)
        
        
    # define the constraints area by the whiteboard area(0.2m closer to the robot)
    constraint_matrix = quaternion_matrix([whiteboard_pose.orientation.x, whiteboard_pose.orientation.y, whiteboard_pose.orientation.z, whiteboard_pose.orientation.w])
    constraint_pose = copy.deepcopy(whiteboard_pose)
    constraint_position_offset=constraint_matrix[:3, :3].dot([0, constraint_position_offset_value, 0])
    constraint_pose.position.x+=constraint_position_offset[0]
    constraint_pose.position.y+=constraint_position_offset[1]
    constraint_pose.position.z+=constraint_position_offset[2]
    hand_quat = quaternion_from_euler(-math.pi/2, 0, -math.atan2(line_coefficients[3], line_coefficients[4]))
  
    constraint = Constraints()
    position_constraint = PositionConstraint()
    position_constraint.header.frame_id = "laser_link"
    position_constraint.link_name = "gripper_link"
    constraint_shape = copy.deepcopy(whiteboard_shape)
    constraint_shape.dimensions[1] = 0.02
    position_constraint.constraint_region.primitives.append(whiteboard_shape)
    position_constraint.constraint_region.primitive_poses.append(constraint_pose)
    position_constraint.weight = 1
    constraint.position_constraints.append(position_constraint) 
    orientation_constraint = OrientationConstraint()
    orientation_constraint.header.frame_id = "laser_link"
    orientation_constraint.link_name = 'gripper_link'
    orientation_constraint.orientation.w = hand_quat[3]
    orientation_constraint.orientation.x = hand_quat[0]
    orientation_constraint.orientation.y = hand_quat[1]
    orientation_constraint.orientation.z = hand_quat[2]
    orientation_constraint.absolute_x_axis_tolerance = 0.02
    orientation_constraint.absolute_y_axis_tolerance = 0.02
    orientation_constraint.absolute_z_axis_tolerance = 0.02
    orientation_constraint.weight = 1.0
    constraint.orientation_constraints.append(orientation_constraint)

   
    move_group.set_path_constraints(constraint)
    
    # rospy.spin()
    # exit()
    
    start_pose = PoseStamped()
    start_pose.header.frame_id = "laser_link"
    start_pose.pose.position.x = constraint_pose.position.x 
    
    start_pose.pose.position.y = constraint_pose.position.y
    start_pose.pose.position.z = 0
    start_pose.pose.orientation.w = hand_quat[3]
    start_pose.pose.orientation.x = hand_quat[0]
    start_pose.pose.orientation.y = hand_quat[1]
    start_pose.pose.orientation.z = hand_quat[2]
    base_pose = listener.transformPose("base_link",start_pose)
    # set the start pose
    base_pose.pose.position.x += 0.2-whiteboard_shape.dimensions[0]/2 
    base_pose.pose.position.z = white_board_min_height
    move_group.set_pose_target(base_pose)
    print("base pose for initialization:", base_pose)
    plan = move_group.plan()
    
    if len(plan.joint_trajectory.points) <= 0: 
        rospy.loginfo("initialization failure!!!!!")
        exit(0)
    move_group.execute(plan, wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    rospy.sleep(2)
 
   
   
    laser_pose = listener.transformPose("laser_link", base_pose)
    start_pose = copy.deepcopy(laser_pose)
        
    # loop to wipe
    index = 1
    while not rospy.is_shutdown():
        
        rospy.loginfo("**********Start wiping %d**********", index)
        
        # try Cartesian path       cannot use!!!!!!!!!!!!! 
        # waypoints = []
        
        # wpose = move_group.get_current_pose().pose
        # wpose.position.z += 0.1   # First move up (z)
        # waypoints.append(copy.deepcopy(wpose))
        # wpose.position.x += 0.1  # Second move forward/backwards in (x)
        # waypoints.append(copy.deepcopy(wpose))
        # # wpose.position.y -= 0.1 # Third move sideways (y)
        # # waypoints.append(copy.deepcopy(wpose))
        
        # # wpose.position.x += 0.1 # Second move forward/backwards in (x)
        # # waypoints.append(copy.deepcopy(wpose))
        
        # (plan, fraction) = move_group.compute_cartesian_path(
        #                                 waypoints,   # waypoints to follow
        #                                 0.01,        # eef_step
        #                                 0.0) 
        # rospy.loginfo("**********fraction: %f**********", fraction)
        # if(fraction < 0.7):
        #     rospy.loginfo("wipe up failure!!!!!")
        #     break
        
        # move_group.execute(plan, wait=True)
        
        rospy.loginfo("%s" , move_group.get_path_constraints())
        
        # set goals directly
        laser_pose, base_pose = move([0, 0, -white_board_height], laser_pose)
        
        # directly set the pose
        move_group.set_pose_target(base_pose)
        print("base pose for wiping up:", base_pose)
        plan = move_group.plan()
        if len(plan.joint_trajectory.points) <= 0: 
            rospy.loginfo("wipe up failure!!!!!")
            break        
        move_group.execute(plan, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        rospy.sleep(1)
    
        break
        
        # target_pose.pose.position.x += wipe_range
        # move_group.set_pose_target(target_pose)
        # plan = move_group.plan()
        # if len(plan.joint_trajectory.points) <= 0: 
        #     rospy.loginfo("wipe sideway failure!!!!!")
        #     break    
        # # else:
        # #     save_plan.joint_trajectory.points.append(plan.joint_trajectory.points)
            
        # move_group.execute(plan, wait=True)
        # move_group.stop()
        # move_group.clear_pose_targets()
        # rospy.sleep(1)
        
        target_pose.pose.position.z -= white_board_height
        move_group.set_pose_target(target_pose)
        plan = move_group.plan()
        if len(plan.joint_trajectory.points) <= 0: 
            rospy.loginfo("wipe down failure!!!!!")
            break
        # else:
        #     save_plan.joint_trajectory.points.append(plan.joint_trajectory.points)
        move_group.execute(plan, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        rospy.sleep(1)
        
        target_pose.pose.position.x += wipe_range
        move_group.set_pose_target(target_pose)
        plan = move_group.plan()
        if len(plan.joint_trajectory.points) <= 0: 
            rospy.loginfo("wipe sideway failure!!!!!")
            break
        # else:
        #     save_plan.joint_trajectory.points.append(plan.joint_trajectory.points)
        move_group.execute(plan, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        rospy.sleep(1)
        
        index = index + 1
        
    
    
    move_group.set_pose_target(start_pose)
    plan = move_group.plan()
    
    if len(plan.joint_trajectory.points) <= 0: 
        rospy.loginfo("initialization failure!!!!!")
        exit(0)
    move_group.execute(plan, wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    rospy.sleep(2)
    
    rospy.loginfo("**********Wiping finished**********")
    

    rospy.spin()
