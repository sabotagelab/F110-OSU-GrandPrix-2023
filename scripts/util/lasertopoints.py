import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
import math

rospy.init_node("laserscan_to_pointcloud")

lp = lg.LaserProjection()

pc_pub = rospy.Publisher("converted_pc", PointCloud2, queue_size=1)

def scan_cb(msg):
    # convert the message of type LaserScan to a PointCloud2
    pc2_msg = lp.projectLaser(msg)

    # now we can do something with the PointCloud2 for example:
    # publish it
    pc_pub.publish(pc2_msg)
    
    # convert it to a generator of the individual points
    point_generator = pc2.read_points(pc2_msg)
    
    points = []
    i = 0
    j = 0
    # we can access a generator in a loop
    for point in point_generator:
        i += 1
        if i > 240 and i < 840:
            points.append((point[0], point[1]))
        #i +=1
    print(len(points))


rospy.Subscriber("/scan", LaserScan, scan_cb, queue_size=1)
rospy.spin()