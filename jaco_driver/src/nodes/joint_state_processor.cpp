//============================================================================
// Name        : jaco_arm_driver.cpp
// Author      : WPI, Clearpath Robotics
// Version     : 0.5
// Copyright   : BSD
// Description : A ROS node that subscribes the original mico joint state messages and re-publishes them so that 
//				they are compatible with the old URDF model for the robot
//============================================================================


#include <signal.h> 
#include <vector>
#include <string.h>
   
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <jaco_msgs/FingerPosition.h>

#include <tf2_msgs/TFMessage.h>

#define PI 3.14159265359

ros::Publisher joint_state_pub;

/*
 * name: ['Shoulder_Joint', 'Arm_Joint', 'Forearm_Joint', 'Wrist_1_Joint', 'Wrist_2_Joint', 'Hand_Joint', 'Finger_1_Proximal_Joint', 'Finger_1_Distal_Joint', 'Finger_2_Proximal_Joint', 'Finger_2_Distal_Joint']

 */
 
float finger_positions[2];

void finger_state_cb (const jaco_msgs::FingerPositionConstPtr& input)
{
	finger_positions[0]=input->finger1;
	finger_positions[1]=input->finger2;
}

void joint_state_cb (const sensor_msgs::JointStateConstPtr& input)
{
	//ROS_INFO("Heard js...");
	
	sensor_msgs::JointState remapped_msg;
	
	remapped_msg.header = input->header;
	
	remapped_msg.header.stamp = ros::Time::now();
	
	remapped_msg.name.push_back("Shoulder_Joint");
	remapped_msg.name.push_back("Arm_Joint");
	remapped_msg.name.push_back("Forearm_Joint");
	remapped_msg.name.push_back("Wrist_1_Joint");
	remapped_msg.name.push_back("Wrist_2_Joint");
	remapped_msg.name.push_back("Hand_Joint");
	remapped_msg.name.push_back("Finger_1_Proximal_Joint");
	remapped_msg.name.push_back("Finger_1_Distal_Joint");
	remapped_msg.name.push_back("Finger_2_Proximal_Joint");
	remapped_msg.name.push_back("Finger_2_Distal_Joint");


	/*
	 * Now we remap these to the old URDF model....
	 */ 
	for (int i = 0; i < 6; i ++){
		float v_i = input->position[i];
		
		if (i == 0)
			v_i = -1*v_i;
		else if (i == 1 || i == 2)
			v_i += PI/2;
		else if (i == 3)
			v_i += PI;
		else if (i == 4)
			v_i += PI;
		else if (i == 5){
			v_i = -1*v_i;
			v_i += -PI/16+PI/2;
		}
		
		remapped_msg.position.push_back(v_i);
	}
	
	float f1_pos = (finger_positions[0] / 7500)*0.78;
	float f2_pos = (finger_positions[1] / 7500)*0.78;
	
	for (int i = 0; i < 2; i ++){
		remapped_msg.position.push_back(f1_pos);
	}
	for (int i = 0; i < 2; i ++){
		remapped_msg.position.push_back(f2_pos);
	}
	
	joint_state_pub.publish(remapped_msg);
}


void tf_cb(const tf2_msgs::TFMessageConstPtr& input){
	//ROS_INFO("%i",input->transforms.size());
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mico_joint_processor");
    ros::NodeHandle nh("~");
   
	//create subscriber to joint angles
	ros::Subscriber sub = nh.subscribe ("/mico_arm_driver/out/joint_state", 1, joint_state_cb);
  
	//create subscriber to joint angles
	ros::Subscriber sub_finger = nh.subscribe ("/mico_arm_driver/out/finger_position", 1, finger_state_cb);
  
	//subscribe to tf
	ros::Subscriber sub_tf = nh.subscribe ("/tf", 1, tf_cb);
  //
  
  
	//create publisher
	joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1000);
  
    while (ros::ok())
    {
        ros::spin();
    }
    
    return 0;
}
