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
#include <vector>
#include <memory>
#include <string>
#include <list>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "frontier_msgs/msg/frontiers.hpp"
#include "frontier_msgs/srv/frontiers.hpp"
#include "nav_msgs/msg/occupancy_grid.h"

/*
 * Helpers
 */
#include "Common.h"
#include "SearchAlgorithms.h"

/*
 * Frontier discovery node class
 */
class FrontierDiscoveryNode : public rclcpp::Node {
public:
    FrontierDiscoveryNode();
    ~FrontierDiscoveryNode();

private:
    void update();
    void cspace_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void estimate_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void create_marker(visualization_msgs::msg::Marker& input, const std::string& ns, int id, int seq);
    void set_pose_arr(geometry_msgs::msg::PoseArray& arr, int seq);
    void reset_frontier_msg(frontier_msgs::msg::Frontiers& msg);
    double compute_centroid_value(nav_msgs::msg::OccupancyGrid& occ, Vec2i& centroid, const double& lidarRange);
    void compute_service_clusters_callback(const std::shared_ptr<frontier_msgs::srv::Frontiers::Request> request,
                                 std::shared_ptr<frontier_msgs::srv::Frontiers::Response> response);
    void compute_service_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                           std::shared_ptr<std_srvs::srv::Trigger::Response> response);                                 

    // Control variables
    int aQueueSize;
    int aId;
    int aSeq;
    int aClusterDetectionMin;
    bool aReceivedCSpace;
    bool aHasPose;
    double aRate;
    double aMaxLidarRange;
    Vec2i aPos;
    std::string aNamespace;

    // Timers
    std::vector<rclcpp::TimerBase::SharedPtr> aTimers;

    // Subscribers
    std::vector<rclcpp::SubscriptionBase::SharedPtr> aSubscribers;
    
    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr aClusterMarkerPub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr aFrontiersMapPub;
    rclcpp::Publisher<frontier_msgs::msg::Frontiers>::SharedPtr aFrontiersClustersPub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr aReachabilityMapPub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr aCostMapPub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr aValueMapPub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr aUtilityMapPub;

    // Service for compute trigger
    rclcpp::Service<frontier_msgs::srv::Frontiers>::SharedPtr aComputeServiceClusters;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr aComputeServiceUtilityMap;

    // Messages
    geometry_msgs::msg::PoseArray aPoseArrMsg;
    geometry_msgs::msg::Pose aWorldPos;
    nav_msgs::msg::OccupancyGrid aOcc;
    nav_msgs::msg::OccupancyGrid aFrontiersMap;
    frontier_msgs::msg::Frontiers aFrontiersMsg;
    visualization_msgs::msg::Marker aClusterMarkerMsg;

    // Helpers
    std::list<Vec2i> aPath;
    std::vector<Vec2i> aFrontiers;
    std::vector<Vec2i> aCentroids;
    std::vector<Vec2i> aFilteredCentroids;
    std::vector<std::vector<Vec2i>> aClusters;
    std::vector<std::vector<Vec2i>> aFilteredClusters;
};
