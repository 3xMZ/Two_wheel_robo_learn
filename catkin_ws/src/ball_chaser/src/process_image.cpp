#include <ros/ros.h>
#include "ball_chaser/DriveToTarget.h"

#include <string>
#include <iostream>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("Driving Robot");

    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the safe_move service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service DriveToTarget");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    cv::Mat cv_image = cv_ptr -> image;
    ROS_INFO_STREAM(std::to_string(cv_image.at<cv::Vec3b>(0,0)[0]));
    ROS_INFO_STREAM(std::to_string(cv_image.at<cv::Vec3b>(0,0)[1]));
    ROS_INFO_STREAM(std::to_string(cv_image.at<cv::Vec3b>(0,0)[2]));

    int white_pixel = 255;
    int width_mid = img.width / 2;
    int height_mid = img.height / 2;
    int center_color = img.data[height_mid*img.width + width_mid];

    ROS_INFO_STREAM(std::to_string(center_color));
    ROS_INFO_STREAM(std::to_string(img.width));
    ROS_INFO_STREAM(std::to_string(img.height));
    ROS_INFO_STREAM(img.encoding);
    ROS_INFO_STREAM(std::to_string(img.data[0]));
    ros::Duration(5.0).sleep();
    if (img.data[height_mid*img.width + width_mid] != white_pixel){
        for (int i = 0; i < img.width; i++) {
            for (int j = 0; j < img.height; j++) {
                int curr_ind = i + j*img.width;
                /*if (img.data[curr_ind] > 178){
                    ROS_INFO_STREAM(std::to_string(curr_ind));
                    ROS_INFO_STREAM(std::to_string(img.data[curr_ind]));
                    ros::Duration(0.1).sleep();
                }*/
                if (img.data[curr_ind] + 30 >= white_pixel) {
                    ROS_INFO_STREAM(std::to_string(img.data[curr_ind]));
                    ros::Duration(2.0).sleep();

                    int diff = width_mid - i;
                    float turn = (diff*1.0)/800.0;

                    drive_robot(0.10, turn);
                    ros::Duration(2.0).sleep();
                    goto end_loop;
                }
            }
        }
        drive_robot(0.05, .125);
        ros::Duration(1.0).sleep();
        end_loop:
        drive_robot(0, 0);
    }
    ros::Duration(3.0).sleep();
    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
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