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
#include <algorithm>

FrontierDiscoveryNode::FrontierDiscoveryNode() : rclcpp::Node("frontier_discovery_node") {
    // Declare and get parameters
    this->declare_parameter<int>("id", 1);
    this->declare_parameter<double>("max_lidar_range", 10.0);
    this->declare_parameter<double>("rate", 2.0);
    this->declare_parameter<int>("queue_size", 2);
    this->get_parameter("id", aId);
    this->get_parameter("max_lidar_range", aMaxLidarRange);
    this->get_parameter("rate", aRate);
    this->get_parameter("queue_size", aQueueSize);
    aNamespace = this->get_namespace();

    // Publishers
    aClusterMarkerPub = this->create_publisher<visualization_msgs::msg::Marker>(aNamespace + "/frontier_discovery/frontiers_clusters_markers", aQueueSize);
    aFrontiersMapPub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(aNamespace + "/frontier_discovery/frontiers", aQueueSize);
    aFrontiersClustersPub = this->create_publisher<frontier_msgs::msg::Frontiers>(aNamespace + "/frontier_discovery/frontiers_clusters", aQueueSize);

    // Subscriptions
    aSubscribers.push_back(
        this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            aNamespace + "/c_space",
            aQueueSize,
            std::bind(&FrontierDiscoveryNode::cspace_callback, this, std::placeholders::_1)));

    aSubscribers.push_back(
        this->create_subscription<geometry_msgs::msg::PoseStamped>(
            aNamespace + "/pose",
            aQueueSize,
            std::bind(&FrontierDiscoveryNode::estimate_pose_callback, this, std::placeholders::_1)));
    
    // Service
    aComputeService = this->create_service<std_srvs::srv::Trigger>(
        aNamespace + "/frontier_discovery/compute",
        std::bind(&FrontierDiscoveryNode::compute_service_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Timers
    double update_period = PeriodToFreqAndFreqToPeriod(aRate);
    aTimers.push_back(
        this->create_wall_timer(
            std::chrono::duration<double>(update_period),
            std::bind(&FrontierDiscoveryNode::update, this)));
}

FrontierDiscoveryNode::~FrontierDiscoveryNode() {}

void FrontierDiscoveryNode::cspace_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    aOcc.data = msg->data;
    aOcc.info = msg->info;
    aOcc.header = msg->header;
    aReceivedCSpace = true;
}

void FrontierDiscoveryNode::estimate_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    aWorldPos.position = msg->pose.position;
    aWorldPos.orientation = msg->pose.orientation;
    aHasPose = true;
}

void FrontierDiscoveryNode::compute_service_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;
    RCLCPP_INFO(this->get_logger(), "Service request received");
    
    // Directly compute frontiers here (was previously in update/PROCESSING)
    if (!aReceivedCSpace || !aHasPose) {
        response->success = false;
        response->message = "CSpace or pose not received yet.";
        return;
    }

    WorldToMap(aOcc, aWorldPos, aPos);
    sa::ComputeFrontiers(aOcc, aFrontiersMap, aFrontiers);
    sa::ComputeClusters(aFrontiersMap, aFrontiers, aClusters);

    aFilteredClusters.clear();
    for (auto& cluster : aClusters) {
        if (cluster.size() > aClusterDetectionMin) aFilteredClusters.push_back(cluster);
    }

    sa::ComputeClusterCenterOfMass(aPos, aFilteredClusters, aCentroids);
    reset_frontier_msg(aFrontiersMsg);

    if (aCentroids.size() > 0) {
        set_pose_arr(aPoseArrMsg, aSeq);
        create_marker(aClusterMarkerMsg, aNamespace, aId, aSeq);
        RCLCPP_INFO(this->get_logger(), "%ld available frontier.", aCentroids.size());
        for (size_t i = 0; i < aCentroids.size(); ++i) {
            tf2::Vector3 temp_world;
            MapToWorld(aOcc, aCentroids[i], temp_world);
            geometry_msgs::msg::Point p;
            geometry_msgs::msg::Pose po;
            p.z = 0.25;
            p.x = temp_world.getX();
            p.y = temp_world.getY();
            po.position.x = temp_world.getX();
            po.position.y = temp_world.getY();
            aClusterMarkerMsg.points.push_back(p);
            aPoseArrMsg.poses.push_back(po);
            RCLCPP_INFO(this->get_logger(), "\t[%.2f %.2f]", temp_world.getX(), temp_world.getY());
        }
        aFrontiersMsg.centroids = aPoseArrMsg;
        aClusterMarkerPub->publish(aClusterMarkerMsg);
        aFrontiersMapPub->publish(aFrontiersMap);
        aSeq += 1;
    }

    aFrontiersClustersPub->publish(aFrontiersMsg);
    response->success = true;
    response->message = "Frontier discovery completed.";
}

void FrontierDiscoveryNode::create_marker(visualization_msgs::msg::Marker& input, const std::string& ns, int id, int seq) {
    input.id = id;
    input.header.frame_id = ns + "/map";
    input.header.stamp = this->now();
    input.ns = ns;
    input.points.clear();
    input.type = visualization_msgs::msg::Marker::CUBE_LIST;
    input.action = visualization_msgs::msg::Marker::MODIFY;
    input.pose.orientation.x = 0.0;
    input.pose.orientation.y = 0.0;
    input.pose.orientation.z = 0.0;
    input.pose.orientation.w = 1.0;
    input.scale.x = 0.25;
    input.scale.y = 0.25;
    input.scale.z = 0.5;
    input.color.a = 1.0;
    input.color.r = 0.0;
    input.color.g = 0.3;
    input.color.b = 1.0;
    input.lifetime = rclcpp::Duration::from_seconds(60);
}

void FrontierDiscoveryNode::set_pose_arr(geometry_msgs::msg::PoseArray& arr, int seq) {
    arr.poses.clear();
    arr.header.frame_id = "robot_" + std::to_string(aId) + "/map";
    arr.header.stamp = this->now();
    // arr.header.seq = seq; // seq is not used in ROS 2 std_msgs/Header
}

// TODO: Replace with your actual custom message type
void FrontierDiscoveryNode::reset_frontier_msg(frontier_msgs::msg::Frontiers& msg) {
    msg.centroids.poses.clear();
    msg.costs.data.clear();
    msg.values.data.clear();
    msg.utilities.data.clear();

    // Use these control variables to get max and min values during single loop
    // avoid calling search everytime until having something better
    msg.lowest_utility_index = -1;
    msg.lowest_cost_index = -1;
    msg.lowest_value_index = -1;

    msg.highest_utility_index = -1;
    msg.highest_cost_index = -1;
    msg.highest_value_index = -1;

    msg.lowest_cost = std::numeric_limits<float>::max();
    msg.lowest_value = std::numeric_limits<float>::max();
    msg.lowest_utility = std::numeric_limits<float>::max();
    
    msg.highest_cost = -1.0;
    msg.highest_value = -1.0;
    msg.highest_utility = -1.0;
}

double FrontierDiscoveryNode::compute_centroid_value(nav_msgs::msg::OccupancyGrid& occ, Vec2i& centroid, const double& lidarRange) {
    double range_squared = lidarRange * lidarRange;
    int range_in_cells = static_cast<int>(lidarRange / occ.info.resolution);
    Vec2i min = Vec2i::Create(centroid.x - range_in_cells, centroid.y - range_in_cells);
    Vec2i max = Vec2i::Create(centroid.x + range_in_cells, centroid.y + range_in_cells);
    min.x = std::max(0, min.x);
    min.y = std::max(0, min.y);
    max.x = std::min(max.x, static_cast<int>(occ.info.width));
    max.y = std::min(max.y, static_cast<int>(occ.info.height));
    int count = 0;
    for (int x = min.x; x < max.x; ++x) {
        for (int y = min.y; y < max.y; ++y) {
            double dx = (centroid.x - x) * occ.info.resolution;
            double dy = (centroid.y - y) * occ.info.resolution;
            double circle_test = dx * dx + dy * dy;
            if (circle_test <= range_squared) {
                int index = y * occ.info.width + x;
                if (occ.data[index] == -1) {
                    count++;
                }
            }
        }
    }
    double cell_area = occ.info.resolution * occ.info.resolution;
    double total_area_value = static_cast<double>(count) * cell_area;
    RCLCPP_INFO(this->get_logger(), "Cells: %d area to uncover in meters squared: %f", count, total_area_value);
    return total_area_value;
}

void FrontierDiscoveryNode::update() {
    // Only used for periodic tasks if needed, otherwise can be left empty or removed
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrontierDiscoveryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}