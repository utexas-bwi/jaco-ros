#!/usr/bin/env python
import roslib; roslib.load_manifest('jaco_demo')
import rospy
from jaco_msgs.msg import JointCurrents
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray, Marker
import time

marker_array_publisher = None
max_observed_current_magnitude = [0.001] * 6
max_observed_effort_magnitude = [0.001] * 6

def current_callback(data):
    global marker_array_publisher
    currents = data.currents
    marker_array = MarkerArray()
    make_current_markers(currents, marker_array)
    marker_array_publisher.publish(marker_array)

def effort_callback(data):
	global marker_effort_publisher
	efforts = data.effort
	marker_array = MarkerArray()
	make_effort_markers(efforts, marker_array)
	marker_array_publisher.publish(marker_array)

def make_effort_markers(efforts, marker_array):
    new_timestamp = time.time() 
    global max_observed_effort_magnitude
    max_observed_effort_magnitude = update_max_magnitude(efforts, max_observed_effort_magnitude)
    scales = scale_for_observations(efforts, max_observed_effort_magnitude)
    for i in range(len(efforts)):
		effort = efforts[i]
		color = color_for_observation(effort)
		frame = frame_for_joint(i)
		scale = scales[i]
		marker = Marker()
		marker.header.frame_id = frame
		marker.header.stamp = rospy.get_rostime()
		marker.ns = "mico_arm_effort_visualization"
		marker.id = i
		marker.type = Marker.CUBE
		marker.action = Marker.ADD
		marker.pose.position.x = 0.00 # 0.07
		marker.pose.position.y = 0
		marker.pose.position.z = 0
		marker.pose.orientation.x = 0.0
		marker.pose.orientation.y = 0.0
		marker.pose.orientation.z = 0.0
		marker.pose.orientation.w = 1.0
		marker.scale.x = scale * 0.025 + 0.02
		marker.scale.y = scale * 0.025 + 0.02
		marker.scale.z = scale * 0.025 + 0.02
		marker.color.a = 0.8
		marker.color.r = color[0]
		marker.color.g = color[1]
		marker.color.b = color[2]
		marker_array.markers.append(marker)
    return marker_array

def make_current_markers(currents, marker_array):
    new_timestamp = time.time()
    global max_observed_current_magnitude
    max_observed_current_magnitude = update_max_magnitude(currents, max_observed_current_magnitude)
    scales = scale_for_observations(currents, max_observed_current_magnitude)
    for i in range(len(currents)):
		current = currents[i]
		color = color_for_observation(current)
		frame = frame_for_joint(i)
		scale = scales[i]
		marker = Marker()
		marker.header.frame_id = frame
		marker.header.stamp = rospy.get_rostime()
		marker.ns = "mico_arm_current_visualization"
		marker.id = i + 6
		marker.type = Marker.SPHERE
		marker.action = Marker.ADD
		marker.pose.position.x = 0
		marker.pose.position.y = 0
		marker.pose.position.z = 0
		marker.pose.orientation.x = 0.0
		marker.pose.orientation.y = 0.0
		marker.pose.orientation.z = 0.0
		marker.pose.orientation.w = 1.0
		marker.scale.x = scale * 0.05 + 0.02
		marker.scale.y = scale * 0.05 + 0.02
		marker.scale.z = scale * 0.05 + 0.02
		marker.color.a = 0.8
		marker.color.r = color[0]
		marker.color.g = color[1]
		marker.color.b = color[2]
		marker_array.markers.append(marker)
    return marker_array

def frame_for_joint(joint_index):
	if joint_index == 5:
	    return "mico_link_hand"
	else:
	   return "mico_link_" + str(joint_index + 1)   
	
def scale_for_observations(currents, max_observed):
	scales = [0.0] * len(currents)
	for i in range(len(currents)):
		current = abs(currents[i])
		scales[i] = min(1.0, current / max_observed[i])
		scales[i] = max(0.0, scales[i])
	return scales
	
def color_for_observation(observation):
	if observation == 0:
		return 0.5, 0.5, 0.5
	elif observation < 0:
		return 1.0, 0.0, 0.0
	else:
		return 0.0, 1.0, 0.0

def update_max_magnitude(observations, max_seen):
	for i in range(len(observations)):
		max_seen[i] = max(max_seen[i], abs(observations[i]))
	return max_seen

def main():
    rospy.init_node('joint_sensor_markers', anonymous=True)
    rospy.Subscriber("mico_arm_driver/out/joint_currents", JointCurrents, current_callback)
    rospy.Subscriber("mico_arm_driver/out/joint_efforts", JointState, effort_callback)
    global marker_array_publisher
    marker_array_publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)

    rospy.spin()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
