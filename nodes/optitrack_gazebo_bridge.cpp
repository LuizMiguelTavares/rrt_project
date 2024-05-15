#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelState.h>

class OptiTrackGazeboBridge
{
public:
    OptiTrackGazeboBridge()
    {
        // Initialize ROS node handle
        ros::NodeHandle nh;

        // Service client for setting Gazebo model state
        set_model_state_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
        

        // Subscriber to PoseStamped messages from OptiTrack
        optitrack_sub = nh.subscribe("/optitrack/pose_stamped", 1, &OptiTrackGazeboBridge::poseCallback, this);
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        // Prepare service request to update model state in Gazebo
        gazebo_msgs::SetModelState set_model_state;
        set_model_state.request.model_state.model_name = "your_robot_model_name";  // Replace with your model's name
        set_model_state.request.model_state.pose = msg->pose;
        set_model_state.request.model_state.reference_frame = "world";  // Ensure the frame matches your Gazebo configuration

        // Call the service
        if (!set_model_state_client.call(set_model_state))
        {
            ROS_ERROR("Failed to call service set_model_state");
        }
    }

private:
    ros::Subscriber optitrack_sub;
    ros::ServiceClient set_model_state_client;
    std::string pose_topic, model_name, reference_frame;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "optitrack_to_gazebo_bridge");
    OptiTrackGazeboBridge bridge;
    ros::spin();
    return 0;
}