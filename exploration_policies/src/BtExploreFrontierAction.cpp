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

#include "FrontierDiscoveryNode.h"
#include "std_srvs/srv/trigger.hpp"
#include "BtExploreFrontierAction.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <algorithm>

ExploreFrontierAction::ExploreFrontierAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {
    aNav2ActionFinished = false;
}

ExploreFrontierAction::~ExploreFrontierAction() {

}

void ExploreFrontierAction::Initialize(FrontierExplorationNode* parent_node) {
    aPParentNode = parent_node;
}

BT::NodeStatus ExploreFrontierAction::onStart() {
    RCLCPP_INFO(aPParentNode->get_logger(), "Starting the ExploreFrontier action node.");

    std::shared_ptr<frontier_msgs::srv::Frontiers::Response> response =
        std::make_shared<frontier_msgs::srv::Frontiers::Response>();
    FrontierExplorationNode* parent_node_cast = (FrontierExplorationNode*)aPParentNode;
    parent_node_cast->ComputeFrontiersClustersService(response);

    RCLCPP_INFO(aPParentNode->get_logger(), "Received %lu frontiers.", response->centroids.poses.size());

    // send goal to nav2, I'm using the topic because the commander API is running only in python (-_-)"
    std::string namespace_tf = aPParentNode->aNamespace;
    if (namespace_tf.front() == '/') {
        namespace_tf.erase(namespace_tf.begin()); // Remove trailing slash if present
    }
    auto goal = nav2_msgs::action::NavigateToPose_Goal();
    goal.pose.header.frame_id = namespace_tf + "/map";
    if (!response->centroids.poses.empty()) {
        // Choose the first frontier as the goal
        goal.pose.pose = response->centroids.poses.front();
        RCLCPP_INFO(aPParentNode->get_logger(), "Setting goal to first frontier at position: (%f, %f, %f)",
                    goal.pose.pose.position.x, goal.pose.pose.position.y, goal.pose.pose.position.z);
    } else {
        RCLCPP_ERROR(aPParentNode->get_logger(), "No frontiers found to explore.");
        return BT::NodeStatus::FAILURE;
    }

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&ExploreFrontierAction::goalResponseCallback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = 
        std::bind(&ExploreFrontierAction::feedbackCallback, this, std::placeholders::_1);
    send_goal_options.result_callback =
        std::bind(&ExploreFrontierAction::result_callback, this, std::placeholders::_1);

    aPParentNode->aNavigateToPoseActionClient->async_send_goal(goal, send_goal_options);
    aNav2ActionFinished = false;

    return BT::NodeStatus::RUNNING;
}

// If onStart() returned RUNNING, we will keep calling
// this method until it return something different from RUNNING
BT::NodeStatus ExploreFrontierAction::onRunning() {
    if(aNav2ActionFinished) return BT::NodeStatus::SUCCESS;
    else return BT::NodeStatus::RUNNING;
}

// callback to execute if the action was aborted by another node
void ExploreFrontierAction::onHalted() {
    RCLCPP_INFO(aPParentNode->get_logger(), "Exploration aborted.");
}

void ExploreFrontierAction::goalResponseCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle) {
    if (!goal_handle) {
        RCLCPP_ERROR(aPParentNode->get_logger(), "Goal was rejected by the action server.");
        return;
    }
    RCLCPP_INFO(aPParentNode->get_logger(), "Goal accepted by the action server.");
}

void ExploreFrontierAction::feedbackCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle) {
    if (!goal_handle) {
        RCLCPP_ERROR(aPParentNode->get_logger(), "Received feedback for an invalid goal handle.");
        return;
    }
}

void ExploreFrontierAction::result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(aPParentNode->get_logger(), "Goal succeeded.");
    } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
        RCLCPP_ERROR(aPParentNode->get_logger(), "Goal was aborted.");
    } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
        RCLCPP_WARN(aPParentNode->get_logger(), "Goal was canceled.");
    } else {
        RCLCPP_ERROR(aPParentNode->get_logger(), "Unknown result code: %d", static_cast<int>(result.code));
    }
    aNav2ActionFinished = true;
}