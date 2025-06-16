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

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class OccupancyGridFilter : public rclcpp::Node {
public:
    OccupancyGridFilter();
    ~OccupancyGridFilter();
    void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void inflate(nav_msgs::msg::OccupancyGrid& input_grid, 
                nav_msgs::msg::OccupancyGrid& output_grid, 
                int radius_cells, 
                int value_to_inflate, 
                const bool& copy_data = true);
    void update();

    private:
        std::string aNamespace;
        int aObstacleInflationRadiusCells;
        int aFreeInflationRadiusCells;
        int aQueueSize;
        int aUpdateFrequencyHz;
        double aObstacleInflationRadiusMeters;
        double aFreeInflationRadiusMeters;
        bool aReceivedGrid;

        nav_msgs::msg::OccupancyGrid aGrid;
        nav_msgs::msg::OccupancyGrid aFilteredGrid;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr aFilteredGridPub;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr aGridSub;
        rclcpp::TimerBase::SharedPtr aFilterTimer;
};
