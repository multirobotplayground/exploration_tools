/*
 * Noetic-Multi-Robot-Sandbox for multi-robot research using ROS Noetic
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

#ifndef SEARCH_ALGORITHMS_H
#define SEARCH_ALGORITHMS_H

#include <stdio.h>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <exception>
#include <list>
#include <vector>
#include <limits>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "Common.h"

#define REACHED 2
#define VISITED 1
#define UNVISITED -1

namespace sa {
    void InitOccFrom(nav_msgs::msg::OccupancyGrid& rInput, nav_msgs::msg::OccupancyGrid& rOutput);
    bool IsInBounds(nav_msgs::msg::OccupancyGrid& rInput, Vec2i& rPos);
    bool IsInBounds(const int& width, const int& height, Vec2i& rPos);
    bool CheckAny(nav_msgs::msg::OccupancyGrid& rInput, const Vec2i& rStart, const Vec2i& rEnd, const int& rVal);
    void ComputePath(nav_msgs::msg::OccupancyGrid& rOcc, 
                     const Vec2i& rStart, 
                     const Vec2i& rEnd, 
                     std::list<Vec2i>& rOutPath);
    void ComputeValueMap(nav_msgs::msg::OccupancyGrid& rInput,
                            nav_msgs::msg::OccupancyGrid& rOutput,
                              std::vector<Vec2i>& rReached,
                            const double& rMaxLidarRange);
    double WavefrontPropagationValue(nav_msgs::msg::OccupancyGrid& rInput,
                                const Vec2i& rStart,
                            const double& rMaxLidarRange);
    void WavefrontPropagation(
        nav_msgs::msg::OccupancyGrid& rInput,
        nav_msgs::msg::OccupancyGrid& rControl,
        const Vec2i& rStart,
        std::vector<Vec2i>& rReached);
    double ComputeCellValue(nav_msgs::msg::OccupancyGrid& occ, Vec2i& centroid, const double& lidarRange);
    void WavefrontFilter(nav_msgs::msg::OccupancyGrid& rFrontiersMap, 
                         std::vector<Vec2i>& rFilteredFrontiers);
    void ComputeFrontiers(nav_msgs::msg::OccupancyGrid& rInput,  
                          nav_msgs::msg::OccupancyGrid& rOutput,
                          std::vector<Vec2i>& rReached,
                          std::vector<Vec2i>& rFrontiers);
    void ComputeClusters(nav_msgs::msg::OccupancyGrid& rFrontiersMap, 
                         std::vector<Vec2i>& rFrontiers, 
                         std::vector<std::vector<Vec2i>>& rOutClusters,
                         const int& rMinClusterSize);
    Vec2i ClosestFrontierCluster(const Vec2i& rPos, std::vector<Vec2i>& rCluster);
    Vec2i MedianFrontierCluster(const Vec2i& rPos, std::vector<Vec2i>& rCluster);
    void ComputeCentroids(const Vec2i& rPos, 
                          std::vector<std::vector<Vec2i>>& rClusters, 
                          std::vector<Vec2i>& rOutCentroids);
    void ComputeClusterCenterOfMass(const Vec2i& rPos, 
                        std::vector<std::vector<Vec2i>>& rClusters, 
                        std::vector<Vec2i>& rOutCentroids);
};
#endif