#!/usr/bin/env python
import roslib; roslib.load_manifest('jaco_demo')
import rospy
from jaco_msgs.msg import JointCurrents
from visualization_msgs.msg import MarkerArray, Marker
import time

marker_array_publisher = None
max_observed = [0.001] * 6

def callback(data):
    new_timestamp = time.time()
    currents = data.currents
    marker_array = MarkerArray()
    global max_observed
    for i in range(len(currents)):
	current = currents[i]
	max_observed[i] = max(max_observed[i], abs(currents[i]))
	scale = min(1.0, abs(current) / max_observed[i])
        marker = Marker()
	if i == 0:
	    marker.header.frame_id = "mico_link_base"
	else:
	    marker.header.frame_id = "mico_link_" + str(i)   
	marker.header.stamp = rospy.get_rostime()
	marker.ns = "mico_arm_sensor_visualization";
	marker.id = i;
	marker.type = Marker.SPHERE
	marker.action = Marker.ADD
	marker.pose.position.x = 0
	marker.pose.position.y = 0
	marker.pose.position.z = 0
	marker.pose.orientation.x = 0.0
	marker.pose.orientation.y = 0.0
	marker.pose.orientation.z = 0.0
	marker.pose.orientation.w = 1.0
	marker.scale.x = scale * 0.05
	marker.scale.y = scale * 0.05
	marker.scale.z = scale * 0.05
	marker.color.a = 1.0
	marker.color.r = 0.0
	marker.color.g = 1.0
	marker.color.b = 0.0
	marker_array.markers.append(marker)
	
    global marker_array_publisher
    marker_array_publisher.publish(marker_array)
   

def main():
    rospy.init_node('joint_sensor_markers', anonymous=True)
    rospy.Subscriber("mico_arm_driver/out/joint_currents", JointCurrents, callback)
    global marker_array_publisher
    marker_array_publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)

    rospy.spin()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
