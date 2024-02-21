#!/usr/bin/env python

import rospy, math
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import PolygonStamped, Point32
import math
import csv
from os.path import expanduser

# def filter_duplicates(file):
#     with open(file) as f:
#         data = list(csv.reader(f))
#     new_data = [a for i, a in enumerate(data) if a not in data[:i]]
#     with open(file, 'w') as t:
#         write = csv.writer(t)
#         write.writerows(new_data)
#     t.close()


def getpoints():
    home = expanduser("~")
    fl = home + "/f1ten-scripts/csv/leftwp.csv"
    fr = home + "/f1ten-scripts/csv/rightwp.csv"
    # filter_duplicates(fl)
    # filter_duplicates(fr)
    with open(fl, "r") as p:
        point_data_left = list(csv.reader(p))
        p.close()
    with open(fr, "r") as p:
        point_data_right = list(csv.reader(p))
        p.close()
    rights = []
    lefts = []
    for i in range(len(point_data_left)):
        lefts.append((float(point_data_left[i][0]), float(point_data_left[i][1])))
    for i in range(len(point_data_right)):
        rights.append((float(point_data_right[i][0]), float(point_data_right[i][1])))
    return lefts, rights


def publish_obstacle_msg():
    lefts, rights = getpoints()
    obstacle_msg = ObstacleArrayMsg()
    obstacle_msg.header.stamp = rospy.Time.now()
    obstacle_msg.header.frame_id = "/map"
    # Add line obstacle

    for i in range(len(lefts) - 1):
        obstacle_msg.obstacles.append(ObstacleMsg())
        obstacle_msg.obstacles[i].id = i + 1
        line_start = Point32()
        line_start.x = lefts[i][0]
        line_start.y = lefts[i][1]
        # line_start.y = -3
        line_end = Point32()
        line_end.x = lefts[i + 1][0]
        line_end.y = lefts[i + 1][1]
        # line_end.y = -4
        obstacle_msg.obstacles[i].polygon.points = [line_start, line_end]

    # Add polygon obstacle
    ob = ObstacleMsg()
    ob.id = 0
    for i in rights:
        v1 = Point32()
        v1.x = i[0]
        v1.y = i[1]
        ob.polygon.points.append(v1)
    obstacle_msg.obstacles.append(ob)
    pub.publish(obstacle_msg)


if __name__ == "__main__":
    try:
        rospy.init_node("obstacle_msg")
        pub = rospy.Publisher("/obstacles", ObstacleArrayMsg, queue_size=1, latch=True)
        publish_obstacle_msg()
        rospy.sleep(5)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
