import rospy
import numpy as np
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image, PointCloud2
from octomap_msgs.msg import Octomap
from octomap_msgs.srv import GetOctomap, GetOctomapRequest, GetOctomapResponse
import numpy
from cv_bridge import CvBridge
import cv2


def callback(data):
    global temp_time
    if rospy.Time.now() - temp_time > rospy.Duration(3.0):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        print(type(cv_image))
        cv2.imshow("test", cv_image)
        cv2.waitKey(3)
        temp_time = rospy.Time.now()
        
def pt_callback(msg):
    
    gen = point_cloud2.read_points(msg, field_names=('x','y','z'),
                                   skip_nans=True)
    points = np.asarray(list(gen))
    print(points.shape)
    print(points.mean(axis=0), points.min(axis=0), points.max(axis=0))
        
def octo_callback(msg):

    map = msg.deserialize_numpy(msg.data[0], np)
    print(map)
    

     
def listener():
    global temp_time, oct_cloud
    oct_cloud = None
    temp_time = rospy.Time.now()
    
    rospy.init_node('node_name')
    rospy.Rate(1.0)
    
    # rospy.Subscriber("/rgb/image_raw", Image, callback)
    rospy.wait_for_message("/octomap_point_cloud_centers", PointCloud2)
    print("Get msg")
    rospy.Subscriber("/octomap_point_cloud_centers", PointCloud2, pt_callback)
    
    
    print("After subscribe")

    # rospy.Subscriber("/octomap_binary", Octomap, octo_callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
if __name__ == "__main__":

    listener()
    
    # rospy.init_node('node_name')
    
    # rospy.wait_for_service('/octomap_binary')
    # map_fetcher = rospy.ServiceProxy('/octomap_binary', GetOctomap)
    
    # map = map_fetcher()
    # print(map)