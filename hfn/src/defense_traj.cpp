#include<hfn/defense_traj.h>

DefenseTraj::DefenseTraj(const double length, const double width)
{
    ROS_INFO("Initilizing defenseTraj");
    length_ = length;
    width_ = width;
}

void DefenseTraj::InitSquareGoalPoints(const Eigen::Vector3d &left_corner)
{
    ROS_INFO("Initilizing square goal points");
    goal_points_.clear();
    goal_points_.push_back(left_corner + Eigen::Vector3d(0, width_, 0));
    goal_points_.push_back(left_corner + Eigen::Vector3d(length_, width_, 0));
    goal_points_.push_back(left_corner + Eigen::Vector3d(length_, 0, 0));
    goal_points_.push_back(left_corner);

    for (auto &point : goal_points_)
    {
        std::cout << point.transpose() << std::endl;
    }

    is_initialized_ = true;
    is_goal_reached_ = true; // reach(init) the first lower left corner
    cur_goal_index_ = -1;
}


Eigen::Vector3d DefenseTraj::GetNextGoalPoint()
{
    if (!is_initialized_)
    {
        ROS_ERROR("DefenseTraj is not initialized");
        return Eigen::Vector3d::Zero();
    }
    cur_goal_index_ = cur_goal_index_ % num_goals_;
    if(!is_goal_reached_)
    {
        ROS_ERROR("Current goal is not reached yet, current goal index: %d", cur_goal_index_);
        return goal_points_[cur_goal_index_];
    }

    is_goal_reached_ = false;
    cur_goal_index_ = (cur_goal_index_ + 1) % num_goals_;
    return goal_points_[cur_goal_index_];
}