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

#include "FrontierExplorationNode.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "ExploreFrontierAction.h"

FrontierExplorationNode::FrontierExplorationNode(): Node("frontier_exploration_node") {
    // Declare and get parameters
    declare_parameter<int>("id", 1);
    declare_parameter<int>("queue_size", 2);
    declare_parameter<int>("update_frequency_hz", 1);
    declare_parameter<int>("compute_frontier_timeout_seconds", 60);
    get_parameter("id", aId);
    get_parameter("queue_size", aQueueSize);
    get_parameter("update_frequency_hz", aUpdateFrequencyHz);
    get_parameter("compute_frontier_timeout_seconds", aComputeFrontierTimeoutSeconds);
    aNamespace = get_namespace();
    aMissionStarted = false;

    if(aNamespace.size() <= 1) throw std::runtime_error("Namespace must be set for the Frontier Exploration Node.");

    double update_period = 1.0 / static_cast<double>(aUpdateFrequencyHz);
    aTimers.push_back(
        create_wall_timer(
            std::chrono::duration<double>(update_period),
            std::bind(&FrontierExplorationNode::Update, this)));

    aStartMissionService = create_service<std_srvs::srv::Trigger>(
        aNamespace + "/frontier_exploration/start_mission", 
        std::bind(&FrontierExplorationNode::StartMissionCallback, this, std::placeholders::_1, std::placeholders::_2));

    aNavigateToPoseActionClient = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, aNamespace + "/navigate_to_pose");

    if (!aNavigateToPoseActionClient->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(get_logger(), "NavigateToPose action server not available.");
        throw std::runtime_error("NavigateToPose action server not available.");
    }
    RCLCPP_INFO(get_logger(), "NavigateToPose action server is available.");

    InitializeBehaviorTree();
}

void FrontierExplorationNode::ComputeFrontiersClustersService(std::shared_ptr<frontier_msgs::srv::Frontiers::Response> response) {
    RCLCPP_INFO(get_logger(), "Computing frontiers...");

    auto request = std::make_shared<frontier_msgs::srv::Frontiers::Request>();
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("frontier_service_spinning_node", aNamespace);
    rclcpp::Client<frontier_msgs::srv::Frontiers>::SharedPtr compute_service = 
        node->create_client<frontier_msgs::srv::Frontiers>(aNamespace + "/frontier_clusters_discovery/compute");

    while(!compute_service->wait_for_service(std::chrono::seconds(aComputeFrontierTimeoutSeconds))) {
        if(!rclcpp::ok()) {
            RCLCPP_ERROR(get_logger(), "Node is shutting down, exiting service wait loop.");
            return;
        }
        RCLCPP_INFO(get_logger(), "Waiting for the compute frontiers service to become available...");
    }
    
    auto result = compute_service->async_send_request(request);
    if(rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(get_logger(), "Successfully called the compute frontiers service.");
        auto service_response = result.get();
        response->centroids = service_response->centroids;
        response->costs = service_response->costs;
        response->values = service_response->values;
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to call the compute frontiers service.");
    }
}

void FrontierExplorationNode::StartMissionCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    RCLCPP_INFO(get_logger(), "Received start mission request.");
    aMissionStarted = true;
    response->success = true;
    response->message = "Mission started successfully.";
}

void FrontierExplorationNode::InitializeBehaviorTree() {
    // This function can be used to initialize the behavior tree if needed
    RCLCPP_INFO(get_logger(), "Initializing Behavior Tree for Frontier Exploration Node...");
    
    // testing with behavior trees in an explicit manner...
    BT::BehaviorTreeFactory factory;
    BT::Blackboard::Ptr shared_board = BT::Blackboard::create();

    // load all custom tree nodes
    factory.registerNodeType<ExploreFrontierAction>("ExploreFrontierAction");

    std::string tree_definition =  "<root BTCPP_format=\"4\" >\
                                        <BehaviorTree ID=\"MainTree\">\
                                            <Sequence name=\"root_sequence\">\
                                                <ExploreFrontierAction   name=\"explore_frontier\"/>\
                                            </Sequence>\
                                        </BehaviorTree>\
                                    </root>";

    aExplorationBehaviorTree = factory.createTreeFromText(tree_definition, shared_board);

    aExplorationBehaviorTree.applyVisitor([this](BT::TreeNode* node) {
        RCLCPP_INFO(get_logger(), "Node: %s", node->name().c_str());
        if(auto explore_node = dynamic_cast<ExploreFrontierAction*>(node)) {
            explore_node->Initialize(this);
        }
        RCLCPP_INFO(get_logger(), "Initialized ExploreFrontier node.");
    });
}

void FrontierExplorationNode::Update() {
    if(!aMissionStarted) return;

    // Call the expploration behavior tree
    aExplorationBehaviorTree.tickOnce();
}

FrontierExplorationNode::~FrontierExplorationNode() {

}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr node = std::make_shared<FrontierExplorationNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    RCLCPP_INFO(node->get_logger(), "Frontier Exploration Node started.");
    executor.spin();

    rclcpp::shutdown();
    return 0;
}