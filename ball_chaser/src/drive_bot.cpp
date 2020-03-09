#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"
//TODO: Include the ball_chaser "DriveToTarget" header file

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// TODO: Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities
bool handle_drive_request_callback(ball_chaser::DriveToTarget::Request &req,
                                ball_chaser::DriveToTarget::Response &res)
{
    // Display Service request
    ROS_INFO("[+] Received service request: linear_x=%f, angular_z:%f\n", req.linear_x, 
            req.angular_z);

    // Create a geometry_msgs::Twist for DriveToTarget request
    geometry_msgs::Twist twist_cmd;

    twist_cmd.linear.x = req.linear_x;
    twist_cmd.angular.z = req.angular_z;
    
    // Publish twist commands to mkotor
    motor_command_publisher.publish(twist_cmd);

    // Append the feedback message to service response
    res.msg_feedback = "Received twist command: linear_x=" + std::to_string(req.linear_x) + " angular_z="
                    + std::to_string(req.angular_z); 
                    

    return true;
}


int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // TODO: Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request_callback);

    // TODO: Delete the loop, move the code to the inside of the callback function and make the necessary changes to publish the requested velocities instead of constant values
    // while (ros::ok()) {
    //     // Create a motor_command object of type geometry_msgs::Twist
    //     geometry_msgs::Twist motor_command;
    //     // Set wheel velocities, forward [0.5, 0.0]
    //     motor_command.linear.x = 0.5;
    //     motor_command.angular.z = 0.0;
    //     // Publish angles to drive the robot
    //     motor_command_publisher.publish(motor_command);
    // }

    // TODO: Handle ROS communication events
    ROS_INFO("[+] Drive Bot service ready...");
    ros::spin();

    return 0;
}