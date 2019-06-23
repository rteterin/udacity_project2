#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    ball_chaser::DriveToTarget srv;

    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv))
    {
        ROS_ERROR("Failed to call service command_robot");
    }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    constexpr double linear_velocity = 0.5;
    constexpr double angular_velocity = 0.5;

    constexpr int white_pixel = 255;

    if (img.encoding != "rgb8")
    {
        ROS_ERROR("Invalid image encoding: %s, expected rgb8", img.encoding.c_str());
        return;
    }

    int side_width = img.width / 3;

    int center_left_x = side_width;
    int center_right_x = img.width - side_width;

    for (uint32_t row = 0; row < img.height; ++row)
    {
        for (uint32_t column = 0; column < img.width; ++column)
        {
            const unsigned char* pixel_address = img.data.data() + img.step * row + 3 * column;
            unsigned char red = pixel_address[0];
            unsigned char green = pixel_address[1];
            unsigned char blue = pixel_address[2];

            if (red == white_pixel && green == white_pixel && blue == white_pixel)
            {
                if (column < center_left_x)
                {
                    drive_robot(linear_velocity, angular_velocity);
                }
                else if (column > center_right_x)
                {
                    drive_robot(linear_velocity, -angular_velocity);
                }
                else
                {
                    drive_robot(linear_velocity, 0);
                }

                return;
            }
        }
    }

    drive_robot(0, 0);
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
