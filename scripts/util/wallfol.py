import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import math
integral = 0.0
prev_error = 0.0

def Getvelocity(angle):
    angle = abs(math.degrees(angle))
    if angle > 20: return 0.3
    if angle > 10: return 0.5
    if angle <= 10: return 0.8

def PID(error):
    kp = 6.5
    kd = -1.5
    ki = 0
    global integral, prev_error
    inp = kp*error + kd*(error - prev_error)
    prev_error = error
    angle = -math.atan2(inp, 4)
    velocity = Getvelocity(angle)
    drive_publisher(velocity, angle)
    print("taking angle ", math.degrees(angle), " and velo ", velocity)

def lidar_callback(data):
    col = False
    counter = 0
    icount = 0
    print("going..................")
    for i in data.ranges:
        counter += 1
        if i < 0.16:
            icount += 1
            if icount > 7:
                drive_publisher(0, 0)
                col = True
                print("####################stop at ", counter, icount)
                break
    if not col:
        Desired_gap = 0.35
        angle = 58
        index_b = 810
        index_a = index_b - angle*3
        a = data.ranges[index_a]
        b = data.ranges[index_b]
        c, d = data.ranges[0], data.ranges[540]
        angle = math.radians(angle)
        alpha = math.atan2((a*math.cos(angle) - b), (a*math.sin(angle)))
        Dt = b*math.cos(alpha) + 1*math.sin(alpha)
        error = Desired_gap - Dt
        print("dt is ", Dt, " error is ", error)
        PID(error)
        #Dt should be 0.80

def drive_publisher(v, theta):
    drive = AckermannDriveStamped()
    drive.header.stamp = rospy.Time.now()
    drive.header.frame_id = ""
    drive.drive.steering_angle = theta
    drive.drive.speed = v
    drive_pub.publish(drive)
    

rospy.init_node("walfol", anonymous = True)
lidar_sub = rospy.Subscriber("/scan", LaserScan, lidar_callback)
drive_pub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/teleop", AckermannDriveStamped, queue_size=1)
rospy.sleep(0.5)
rospy.spin()

