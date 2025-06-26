/*
 * Noetic-Multi-Robot-Sandbox for multi-robot research using ROS 2
 * Copyright (C) 2020 Alysson Ribeiro da Silva
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

/*
 * Ros and system
 */
#include "rclcpp/rclcpp.hpp"
#include "frontier_msgs/msg/frontiers.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "std_srvs/srv/trigger.hpp"
#include "frontier_msgs/srv/frontiers.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

/*
 * Frontier discovery node class
 */
class FrontierExplorationNode : public rclcpp::Node{
    friend class DiscoverFrontiers;
    friend class ExploreFrontierAction;
    friend class GoBaseStation;

public:
    FrontierExplorationNode();
    ~FrontierExplorationNode();
    void StartMissionCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void ComputeFrontiersClustersService(std::shared_ptr<frontier_msgs::srv::Frontiers::Response> response);
    void InitializeBehaviorTree();
    void Update();
    void SendGoal();

private:
    BT::Tree aExplorationBehaviorTree;;    

    // Control variables
    int aQueueSize;
    int aId;
    int aUpdateFrequencyHz;
    int aComputeFrontierTimeoutSeconds;
    bool aMissionStarted;
    std::string aNamespace;

    // Timers
    std::vector<rclcpp::TimerBase::SharedPtr> aTimers;
    frontier_msgs::msg::Frontiers aLastReceivedFrontiersList;
    geometry_msgs::msg::Pose aCurrentFrontierPose;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr aStartMissionService;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr aNavigateToPoseActionClient;
};
