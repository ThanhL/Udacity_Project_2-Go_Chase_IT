#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include "pid_controller.h"
#include <sensor_msgs/Image.h>
#include <assert.h>

// Define a global client that can request services
ros::ServiceClient client;

// Define global PID controller
#define KP_GAIN 1.0
#define KI_GAIN 0.0
#define KD_GAIN 0.0
#define MAX_PID_OUTPUT 2.0
#define MIN_PID_OUTPUT -2.0

PID pid_controller(KP_GAIN, KI_GAIN, KD_GAIN, MAX_PID_OUTPUT, MIN_PID_OUTPUT);


// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget drive_to_tgt_srv;
    drive_to_tgt_srv.request.linear_x = lin_x;
    drive_to_tgt_srv.request.angular_z = ang_z;

    if (client.call(drive_to_tgt_srv))
    {
        ROS_INFO("%s", drive_to_tgt_srv.response.msg_feedback.c_str());
    } 
    else
    {
        ROS_ERROR("Failed to call Drive Bot service!");
    }
}


// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    // ROS_INFO("%d", img.data.size());
    // ROS_INFO("Step size: %d", img.step);
    // ROS_INFO("encoding: %s", img.encoding);

    // Note that there are 3 rgb channels, so when iterating through image array, we have to go through
    // 3 data elements to process one pixel
    int num_rgb_channels = 3;   
    int red_channel_value, green_channel_value, blue_channel_value;

    // mean location of the white pixel in the x coordinate 
    // NOTE: only consider x because we've partitioned to either left, forward, right action space
    float white_pixel_x_mean = 0.0; 
    float white_pixel_x_sum = 0.0;
    int num_white_pixels = 0;
    

    for (int i=0; i < img.height * img.step; i += num_rgb_channels)
    {
        // Decompose and cache the image channels
        red_channel_value = img.data[i];
        green_channel_value = img.data[i+1];
        blue_channel_value = img.data[i+2];

        // Check for white pixel
        if (red_channel_value == white_pixel && 
            green_channel_value == white_pixel && 
            blue_channel_value == white_pixel)
        {
            // Determine the x_position of the white pixel
            int x_position = i % img.step;

            // Aggregate the x position white pixel sum and the number of white pixels
            white_pixel_x_sum += x_position;
            num_white_pixels += 1;
        }
    }

    // Check if we've detected any white pixels, if so process the centroid of the white pixel
    assert(!(num_white_pixels < 0));    // There cannot be negative white pixels
    if (num_white_pixels > 0)
    {
        // Determine the mean x position of the white pixels
        white_pixel_x_mean = (float) white_pixel_x_sum / num_white_pixels;

        ROS_INFO("White Pixel x position Mean: %f", white_pixel_x_mean);


        // Want to maintain the white pixel x position mean in the center of the camera image
        double desired_refrence = std::floor(img.width / 2.0);
        ROS_INFO("Desired white pixel mean position: %f", desired_refrence);

        // Calculate the angular velocity pid output
        ROS_INFO("Previous ROS time (secs): %f", previous_time);
        double dt = ros::Time::now().toSec() - previous_time;
        previous_time = ros::Time::now().toSec();		// Update previous time
        ROS_INFO("DT: %f", dt);


        /*
        // Determine where the mean position lies in what partition (left, mid, right) and process motor control
        if (white_pixel_x_mean < img.step/3)
        {
            // Left Quadrant
            drive_robot(0.5, 1);
        }
        else if (white_pixel_x_mean > (img.step/3 * 2))
        {
            // Mid Quadrant
            drive_robot(0.5, 0);
        } 
        else
        {
            // Right Quadrant
            drive_robot(0.5, -1);
        }
        */
    } 
    else
    {
        // Do da nothing
        drive_robot(0.0, 0.0);
    }




    // drive_robot(0.1, 0.0);

}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}