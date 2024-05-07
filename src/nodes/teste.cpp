void generateLocalRRT() {
    updatePositionFromTF(global_map_.header.frame_id, map_frame_id_);
    std::vector<double> aux_position = {robot_position[0] - global_map_.info.origin.position.x, robot_position[1] - global_map_.info.origin.position.y};

    motion_planning::Node start(aux_position, nullptr);
    motion_planning::Node* closest_path_point = quad_tree_->nearest_neighbor(&start);

    geometry_msgs::PointStamped last_point_inside_map;
    bool last_point_valid = false;

    cv::Mat local_map = occupancyGridToCvMat(map_);
    bool obstacle_encountered = false;
    geometry_msgs::PoseStamped last_local_pose;
    bool last_local_pose_valid = false;

    for (int i = closest_path_point->index; i < path_.poses.size(); i++) {
        geometry_msgs::PoseStamped pose;
        pose.header = path_.header;
        pose.pose = path_.poses[i].pose;

        geometry_msgs::PoseStamped pose_map_frame;
        if (transformPoseToMapFrame(pose, pose_map_frame)) {
            if (isInMap(pose_map_frame.pose.position)) {
                if (!last_local_pose_valid) {
                    last_local_pose = pose_map_frame;
                    last_local_pose_valid = true;
                } else {
                    if (!obstacle_encountered) {
                        int x_old = static_cast<int>((last_local_pose.pose.position.x - map_.info.origin.position.x) / map_.info.resolution);
                        int y_old = static_cast<int>((last_local_pose.pose.position.y - map_.info.origin.position.y) / map_.info.resolution);
                        int x_new = static_cast<int>((pose_map_frame.pose.position.x - map_.info.origin.position.x) / map_.info.resolution);
                        int y_new = static_cast<int>((pose_map_frame.pose.position.y - map_.info.origin.position.y) / map_.info.resolution);

                        y_old = map_.info.height - y_old;
                        y_new = map_.info.height - y_new;

                        obstacle_encountered = motion_planning::check_obstacle_intersection(local_map, x_old, y_old, x_new, y_new, radius_);
                        last_local_pose = pose_map_frame;
                    }
                }
                last_point_inside_map.point = pose.pose.position;
                last_point_valid = true;
            } else {
                break;
            }
        }
    }

    if (!last_point_valid) {
        ROS_ERROR("%s: No global path point inside the local map.", ros::this_node::getName().c_str());
        return;
    }

    if (obstacle_encountered) {
        ROS_ERROR("%s: Obstacle encountered in the local map.", ros::this_node::getName().c_str());
        return;
    }

    // Convert last known valid pose to node
    x_start = static_cast<int>(map_.info.width/2)
    y_start = static_cast<int>(map_.info.height/2)

    x_goal = static_cast<int>((last_point_inside_map.point.x - map_.info.origin.position.x) / map_.info.resolution);
    y_goal = static_cast<int>((last_point_inside_map.point.y - map_.info.origin.position.y) / map_.info.resolution);
    y_goal = map_.info.height - y_goal;

    std::vector<double> start_pos = {x_start, y_start};
    std::vector<double> goal_pos = {x_goal, y_goal};

    motion_planning::Node start_node(start_pos, nullptr);
    motion_planning::Node goal_node(goal_pos, nullptr);

    std::vector<motion_planning::Node*> nodes = motion_planning::rrt(local_map, &start_node, &goal_node, num_nodes_, step_size_, goal_threshold_, bias_probability_, radius_);

    if (nodes.empty()) {
        ROS_ERROR("No path found from RRT within the local map.");
        return;
    }

    last_point_inside_map.header.stamp = ros::Time::now();
    last_point_inside_map.header.frame_id = path_.header.frame_id;
    last_point_pub_.publish(last_point_inside_map);

    // Convert the path from nodes to a ROS path message
    nav_msgs::Path ros_path = convertNodesToPath(nodes, map_.header.frame_id);
    path_pub_.publish(ros_path);
}

nav_msgs::Path convertNodesToPath(const std::vector<motion_planning::Node*>& nodes, const std::string& frame_id) {
    nav_msgs::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = ros::Time::now();

    for (auto node : nodes) {
        if (node == nullptr) continue; // Skip null nodes

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = frame_id;
        pose_stamped.header.stamp = ros::Time::now();
        
        // Assuming your node stores positions in map coordinates
        pose_stamped.pose.position.x = node->position[0]; // Node position x
        pose_stamped.pose.position.y = node->position[1]; // Node position y
        pose_stamped.pose.position.z = 0; // Assume z is 0 for 2D navigation

        // Default orientation (no rotation)
        pose_stamped.pose.orientation.x = 0.0;
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = 0.0;
        pose_stamped.pose.orientation.w = 1.0;

        path.poses.push_back(pose_stamped);
    }

    return path;
}
