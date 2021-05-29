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
    cv::Mat cv_image = cv_ptr -> image;  //cv_image is BGR

    int width_mid = img.width / 2;
    int height_mid = img.height / 2;
    int center_color = img.data[height_mid*img.width + width_mid];
    ROS_INFO_STREAM(img.encoding); //RGB8
    /*
    ROS_INFO_STREAM(std::to_string(center_color));
    ROS_INFO_STREAM(std::to_string(img.width));
    ROS_INFO_STREAM(std::to_string(img.height));
    ROS_INFO_STREAM(img.encoding);
    ROS_INFO_STREAM(std::to_string(img.data[0]));
    ros::Duration(5.0).sleep();
    */
    if (cv_image.at<cv::Vec3b>(height_mid, width_mid)[2] != 255){
        for (int i = 0; i < img.width; i++) {
            for (int j = 0; j < img.height; j++) {
                int B = cv_image.at<cv::Vec3b>(j, i)[0];
                int G = cv_image.at<cv::Vec3b>(j, i)[1];
                int R = cv_image.at<cv::Vec3b>(j, i)[2];

                if (B <= 5 && R >= 250 && G <= 5){
                    ROS_INFO_STREAM(std::to_string(i));
                    int diff = width_mid - i;
                    float turn = (diff*1.0)/img.width;

                    drive_robot(0.10, turn);
                    ros::Duration(1.0).sleep();
                    goto end_loop;
                }
            }
        }
        drive_robot(0, .125);
        ros::Duration(1.0).sleep();
        end_loop:
        drive_robot(0, 0);
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