#!/usr/bin/env python
import rospy
from os.path import expanduser
from nav_msgs.msg import Path

home = expanduser('~')
# file = open(home +'/f1ten-scripts/waypoints.csv', 'w')
file = open(home +'/f1ten-scripts/sides.csv', 'w')
# file = open(home +'/f1ten-scripts/rightwp.csv', 'w')


def save_waypoint(traj_msg):
    for i in traj_msg.poses:
        file.write('%f, %f\n' % (i.pose.position.x, i.pose.position.y))
    file.close()
    print('Goodbye')


if __name__ == '__main__':
    rospy.init_node('waypoints_logger', anonymous=True)
    # traj_msg = rospy.wait_for_message( "/via_points", Path)
    traj_msg = rospy.wait_for_message( "/side", Path)
    # traj_msg = rospy.wait_for_message( "/right", Path)
    print('Saving waypoints...')
    save_waypoint(traj_msg)
