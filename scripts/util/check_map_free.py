import csv
from datetime import datetime
import matplotlib.pyplot as plt
import numpy as np
import rospy
import rosbag


def check_map_free(recorded_scans, current_pos, current_scan):
    # FIND CLOSEST POSITION
    positions = recorded_scans[:,0:2]
    diff = np.linalg.norm(positions - current_pos, axis=1)
    min_diff = np.argmin(diff)
    closest_position = positions[min_diff]
    #print(closest_position)

    # GET SCAN OF CLOSEST POSITION
    ref_scan = recorded_scans[min_diff, 2:]
    
    # TODO: DO SCAN MATCHING TO ALIGN RECORDED SCAN AND CURRENT SCAN

    # GET DIFFERENCE BETWEEN (ALIGNED) SCAN AND RECORDED SCAN
    scan_diff = ref_scan - current_scan
    print(scan_diff)
    free = all(scan_diff > -1) 

    return free, ref_scan, scan_diff



################## FOR TESTING

###### LOAD THE BAG FILE FROM RECORDING WITH OBSTACLES
bagfile = 'bags/dynamic_obst.bag'
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


######## SAVE A DICTIONARY OF POSITIONS TO SCANS FOR TESTING
positions_to_scans = {}
for t, p in positions.items():
    if t not in scans.keys():
        continue
    x = p[0]
    y = p[1]
    positions_to_scans[(x, y)] = scans[t]

test_scan_ts = position_timestamps[0]
test_position = positions[test_scan_ts]
test_scan = positions_to_scans[(test_position[0], test_position[1])]

print(test_position)
plt.scatter(list(range(0, len(test_scan))), test_scan, color='blue')

recorded_scans = np.loadtxt('pos_to_scan_map.csv', delimiter=',')
free, ref_scan, scan_diff = check_map_free(recorded_scans, test_position, test_scan)

plt.scatter(list(range(0, len(ref_scan))), ref_scan, color='red')

plt.scatter(list(range(0, len(scan_diff))), scan_diff, color='green', alpha=0.5)
plt.show()

print('Is map free? {}'.format(free))
