#!/usr/bin/env python
import roslib; roslib.load_manifest('jaco_demo')
import rospy
from jaco_msgs.msg import JointCurrents
from std_msgs.msg import Float32

estimate = 0.0
window = [0.0] * 40

def callback(data):
    total = sum([abs(reading) for reading in data.currents])
    global window
    global estimate
    new_window = window[1:]
    new_window.append(total)
    estimate = 24 * sum(new_window) / len(new_window)
    window = new_window

def main():
    rospy.init_node('power_meter', anonymous=True)
    rospy.Subscriber("mico_arm_driver/out/joint_currents", JointCurrents, callback)
    pub = rospy.Publisher('power_estimate', Float32, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.loginfo(estimate)
        pub.publish(estimate)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
