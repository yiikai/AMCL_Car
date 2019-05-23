#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    client.call(srv);
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    sensor_msgs::Image::_data_type imgdata = img.data;
    sensor_msgs::Image::_width_type step = img.width;
    uint32_t range = step / 3;
    uint32_t left = 0 + step / 3;
    uint32_t mid = left + step / 3; 
    uint32_t right = step;
    int i = 0;
    uint32_t imgsize = imgdata.size();
    while(imgdata[i] != white_pixel || 
	imgdata[i+1] != white_pixel ||
	imgdata[i+2] != white_pixel)
    {
	i += 3;
	if( i == imgsize)
	{
	   ROS_INFO("No ball in view. car should stop");
	   drive_robot(0.0,0.0);
	   return;
	}

    }
    uint32_t pos = i / 3 % img.height;
    ROS_INFO("Data pixel pos is : %d",pos);
    if(pos >= 0 && pos < left)
    {
	//call left
	ROS_INFO("Left!!!!!!!!!!!");
	drive_robot(0.0,0.5);
    }
    else if(pos >= left && pos < mid)
    {
	//call forward
	ROS_INFO("Forward!!!!!!!!!!!");
	drive_robot(0.5,0.0);
    }
    else
    {
	//call right
	
	ROS_INFO("Right!!!!!!!!!!!");
	drive_robot(0.0,-0.5);
    }

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
