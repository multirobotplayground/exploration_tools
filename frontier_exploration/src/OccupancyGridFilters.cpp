/*
 * Jazzy-Multi-Robot-Sandbox for multi-robot research using ROS 2
 * Copyright (C) 2025 Alysson Ribeiro da Silva
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

#include "OccupancyGridFilters.h"

OccupancyGridFilter::OccupancyGridFilter() : Node("occupancy_grid_filter_node") {
    // Constructor implementation
    RCLCPP_INFO(get_logger(), "OccupancyGridFilter node initialized.");

    declare_parameter<double>("obstacle_inflation_radius_meters", 0.6);
    declare_parameter<double>("free_inflation_radius_meters", 0.5);
    declare_parameter<int>("update_frequency_hz", 10);
    declare_parameter<int>("queue_size", 10);
    declare_parameter<bool>("inflate_obstacles", false);
    declare_parameter<bool>("inflate_free_space", true);
    get_parameter("update_frequency_hz", aUpdateFrequencyHz);
    get_parameter("queue_size", aQueueSize);
    get_parameter("obstacle_inflation_radius_meters", aObstacleInflationRadiusMeters);
    get_parameter("free_inflation_radius_meters", aFreeInflationRadiusMeters);
    aReceivedGrid = false;
    aNamespace = get_namespace();

    aGridSub = create_subscription<nav_msgs::msg::OccupancyGrid>(
        aNamespace + "/input_occupancy_grid", 
        aQueueSize,
        std::bind(&OccupancyGridFilter::occupancyGridCallback, this, std::placeholders::_1));

    aFilteredGridPub = create_publisher<nav_msgs::msg::OccupancyGrid>(
        aNamespace + "/filtered_occupancy_grid",
        aQueueSize);

    double update_period = 1.0 / static_cast<double>(aUpdateFrequencyHz);
    aFilterTimer = create_wall_timer(
        std::chrono::duration<double>(update_period),
        std::bind(&OccupancyGridFilter::update, this));
}

OccupancyGridFilter::~OccupancyGridFilter() {
    // Destructor implementation
}

void OccupancyGridFilter::occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    aGrid.header = msg->header;
    aGrid.info = msg->info;

    // the std::vector has an implementation that allows deep copying
    // so we can directly assign the data from the received message
    if (msg->data.empty()) {
        RCLCPP_WARN(get_logger(), "Received empty occupancy grid data.");
        return;
    }
    if (msg->info.width == 0 || msg->info.height == 0) {
        RCLCPP_WARN(get_logger(), "Received occupancy grid with zero dimensions.");
        return;
    }
    if (msg->info.resolution <= 0) {
        RCLCPP_WARN(get_logger(), "Received occupancy grid with non-positive resolution.");
        return;
    }
    if (msg->data.size() != msg->info.width * msg->info.height) {
        RCLCPP_WARN(get_logger(), "Received occupancy grid data size does not match dimensions.");
        return;
    }
    
    aGrid.data = msg->data;
    
    if(aReceivedGrid == false) {
        aObstacleInflationRadiusCells = 
            static_cast<int>(aObstacleInflationRadiusMeters / aGrid.info.resolution);
        aFreeInflationRadiusCells = 
            static_cast<int>(aFreeInflationRadiusMeters / aGrid.info.resolution);
        aReceivedGrid = true;
        RCLCPP_INFO(get_logger(), 
            "Inflation radii set to %d cells for obstacles and %d cells for free space.", aObstacleInflationRadiusCells, aFreeInflationRadiusCells);
    }

    RCLCPP_INFO(get_logger(), "Received occupancy grid with size: %d x %d", aGrid.info.width, aGrid.info.height);
}

void OccupancyGridFilter::inflate(nav_msgs::msg::OccupancyGrid& input_grid, 
            nav_msgs::msg::OccupancyGrid& output_grid, 
            int radius_cells, 
            int value_to_inflate, 
            const bool& copy_data = true) {
    if(copy_data) {
        output_grid.data = input_grid.data;
        output_grid.info = input_grid.info;
        output_grid.header = input_grid.header;
    }

    for (size_t i = 0; i < input_grid.data.size(); ++i) {
        if (input_grid.data[i] >= value_to_inflate) {  // If the cell is occupied
            int x = i % input_grid.info.width;
            int y = i / input_grid.info.width;
            for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
                    if (dx * dx + dy * dy <= radius_cells * radius_cells) {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx >= 0 && nx < output_grid.info.width && ny >= 0 && ny < output_grid.info.height) {
                            output_grid.data[ny * output_grid.info.width + nx] = value_to_inflate;
                        }
                    }
                }
            }
        }
    }
}

void OccupancyGridFilter::update() {
    if(!aReceivedGrid) {
        RCLCPP_WARN(get_logger(), "No occupancy grid received yet. Skipping update.");
        return;
    }

    // Apply the filter to the occupancy grid
    inflate(aGrid, aFilteredGrid, aFreeInflationRadiusCells, 0);
    inflate(aGrid, aFilteredGrid, aObstacleInflationRadiusCells, 100, false);
    aFilteredGridPub->publish(aFilteredGrid);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyGridFilter>());
    rclcpp::shutdown();
    return 0;
}