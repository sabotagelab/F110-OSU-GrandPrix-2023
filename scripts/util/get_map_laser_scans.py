import csv
from datetime import datetime
import matplotlib.pyplot as plt
import numpy as np
import rospy
import rosbag

##### SPECIFY WHERE TO SAVE MAP OF POSITIONS TO LASER SCANS
save_file_name = 'pos_to_scan_map.csv'


###### LOAD THE BAG FILE FROM THE INITIAL MAPPING OF THE TRACK
bagfile = 'bags/no_obstacles.bag'
bag = rosbag.Bag(bagfile)


###### SAVE THE POSITIONS AND TIMESTAMPS
positions_raw = {}

timestamps = []
for topic, msg, t in bag.read_messages(topics=['/pf/pose/odom']):
    tsec = t.to_sec()
    positions_raw[tsec] = [msg.pose.pose.position.x, msg.pose.pose.position.y]
    timestamps.append(tsec)

timestamps = list(set(timestamps))

min_timestamp = min(timestamps)

######## RESET POSITIONS KEYS TO FORMATTED TIMESTAMPS
positions = {}
for t, p in positions_raw.items():
    positions[t-min_timestamp] = p

######## SAVE THE SCANS AT THE TIMESTAMPS
position_timestamps = np.array(list(positions.keys()))
scans = {}
scan_diff = {}

count_scans = 0
redundant_scans = 0
for topic, msg, t in bag.read_messages(topics=['/scan']):
    tsec = t.to_sec() - min_timestamp
    diff = np.abs(position_timestamps - tsec)
    min_diff = np.min(diff)
    min_diff_id = np.argmin(diff)
    position_time = position_timestamps[min_diff_id]

    if position_time in scans.keys():
        if min_diff < scan_diff[position_time]:
            scans[position_time] = list(msg.ranges)
            scan_diff[position_time] = min_diff
            redundant_scans += 1
    else:
        scans[position_time] = list(msg.ranges)
        scan_diff[position_time] = min_diff
    count_scans += 1

#print('Num scans {}'.format(count_scans))
#print('Replaced Scans {}'.format(redundant_scans))
#print(len(scans))
#print(len(positions))
#print(scan_diff.values())

######## SAVE POSITION TO SCAN FILE
save_file = open(save_file_name, 'w')
csv_writer = csv.writer(save_file)
for t, p in positions.items():
    if t not in scans.keys():
        continue
    x = p[0]
    y = p[1]
    ranges = scans[t]
    row = [x, y] + ranges
    csv_writer.writerow(row)
save_file.close()


