#ifndef SCARAB_HFN_INCLUDE_HFN_DEFENSE_TRAJ_H
#define SCARAB_HFN_INCLUDE_HFN_DEFENSE_TRAJ_H

/****
 * This file is used as a ROS NODE to publish the goal points in
 * square shape, need vicon as feedback sensor
 ****/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Core>
#include <vector>

class DefenseTraj
{
private:
    const int num_goals_ = 4;
    double length_ = 0.0;
    double width_ = 0.0;
    std::vector<Eigen::Vector3d> goal_points_;

    bool is_initialized_ = false;
    bool is_goal_reached_ = false;
    int cur_goal_index_ = 0;

    ros::NodeHandle nh_;

public:
    explicit DefenseTraj( double length,  double width);
    ~DefenseTraj()= default;

    void SetReachGoalFlag(const bool flag) { is_goal_reached_ = flag; }
    void InitSquareGoalPoints(const Eigen::Vector3d & left_corner);
    Eigen::Vector3d GetNextGoalPoint();
    bool isSquareGoalInitialized() { return is_initialized_; }

};


#endif
