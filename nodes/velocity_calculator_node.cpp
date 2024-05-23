#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class VelocityCalculator
{
public:
    VelocityCalculator(const ros::NodeHandle& nh) : nh_(nh),
        private_nh_("~"),
        tfListener_(tfBuffer_),
        last_transform_valid_(false)
    {
        // Load parameters
        private_nh_.param("world_frame", world_frame_, std::string("world"));
        private_nh_.param("robot_frame", robot_frame_, std::string("robot_base"));
        private_nh_.param("velocity_topic", velocity_topic_, std::string("robot_velocity"));
        private_nh_.param("rate", rate_hz_, 10);

        // Setup publisher
        vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(velocity_topic_, 10);
    }

    void run()
    {
        ros::Rate rate(rate_hz_);
        while (ros::ok())
        {
            calculateAndPublishVelocity();
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher vel_pub_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;

    std::string world_frame_;
    std::string robot_frame_;
    std::string velocity_topic_;
    int rate_hz_;

    geometry_msgs::TransformStamped last_transform_;
    bool last_transform_valid_;

    void calculateAndPublishVelocity()
    {
        try
        {
            auto transform = tfBuffer_.lookupTransform(world_frame_, robot_frame_, ros::Time(0));
            if (last_transform_valid_)
            {
                ros::Duration time_diff = transform.header.stamp - last_transform_.header.stamp;
                if (time_diff.toSec() > 0)
                {
                    geometry_msgs::TwistStamped velocity_msg;
                    velocity_msg.header.stamp = ros::Time::now();
                    velocity_msg.header.frame_id = robot_frame_;

                    velocity_msg.twist.linear.x = (transform.transform.translation.x - last_transform_.transform.translation.x) / time_diff.toSec();
                    velocity_msg.twist.linear.y = (transform.transform.translation.y - last_transform_.transform.translation.y) / time_diff.toSec();
                    velocity_msg.twist.linear.z = (transform.transform.translation.z - last_transform_.transform.translation.z) / time_diff.toSec();

                    tf2::Quaternion q_curr, q_last;
                    tf2::convert(transform.transform.rotation, q_curr);
                    tf2::convert(last_transform_.transform.rotation, q_last);
                    double roll, pitch, yaw, last_roll, last_pitch, last_yaw;
                    tf2::Matrix3x3(q_curr).getRPY(roll, pitch, yaw);
                    tf2::Matrix3x3(q_last).getRPY(last_roll, last_pitch, last_yaw);

                    velocity_msg.twist.angular.x = (roll - last_roll) / time_diff.toSec();
                    velocity_msg.twist.angular.y = (pitch - last_pitch) / time_diff.toSec();
                    velocity_msg.twist.angular.z = (yaw - last_yaw) / time_diff.toSec();

                    vel_pub_.publish(velocity_msg);
                }
            }
            last_transform_ = transform;
            last_transform_valid_ = true;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Failed to calculate velocity: %s", ex.what());
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velocity_calculator");
    ros::NodeHandle nh;
    VelocityCalculator calculator(nh);
    calculator.run();
    return 0;
}
