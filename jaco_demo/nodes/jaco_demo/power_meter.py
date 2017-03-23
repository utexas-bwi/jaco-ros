#!/usr/bin/env python
import roslib; roslib.load_manifest('jaco_demo')
import rospy
from jaco_msgs.msg import JointCurrents
from std_msgs.msg import Float32
import time

power_estimate = 0.0
cumulative_power_estimate = 0.0
window = [0.0] * 40
last_timestamp = None
arm_voltage = 24

def callback(data):
    new_timestamp = time.time()
    total = sum([abs(reading) for reading in data.currents])
    global window
    global power_estimate
    global last_timestamp
    global cumulative_power_estimate
    # Slide the window, add our new measurement
    new_window = window[1:]
    new_window.append(total)

    # P = IV
    # We'll use the arm's nominal voltage (as labeled on the base of the arm) and
    # a moving average over the last second of the total joint current consumption
    power_estimate = arm_voltage * sum(new_window) / len(new_window)
    window = new_window
    if last_timestamp:
	diff = new_timestamp - last_timestamp 
	# We'll integrate the power estimate as we go. This measurement has a unit of watt/seconds
	cumulative_power_estimate += diff * power_estimate
    last_timestamp = time.time()

def main():
    rospy.init_node('power_meter', anonymous=True)
    rospy.Subscriber("mico_arm_driver/out/joint_currents", JointCurrents, callback)
    power_publisher = rospy.Publisher('power_estimate', Float32, queue_size=10)
    cumulative_power_publisher = rospy.Publisher('cumulative_power_estimate', Float32, queue_size=10)
    rate = rospy.Rate(10)
    t = 0
    while not rospy.is_shutdown():
        
        power_publisher.publish(power_estimate)
	# Only publish the cumulative power estimate once a second
	if t % 10 == 0:
		# We'll convert watt seconds to watt hours to make this number more useful
		rospy.loginfo(cumulative_power_estimate / 3600)
		cumulative_power_publisher.publish(cumulative_power_estimate)
	
	t += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
