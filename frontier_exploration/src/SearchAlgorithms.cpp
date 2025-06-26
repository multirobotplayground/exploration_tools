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

#include "SearchAlgorithms.h"
// #include "ros/ros.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <queue>
#include <set>
#include "rclcpp/rclcpp.hpp"

std::mt19937* randomglobal() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    return &gen;
}

namespace sa {

    struct el{
        Vec2i pos;
        double dist;
        el(){
            pos = Vec2i::Create(0,0);
            dist = 0.0;
        }
        el(const Vec2i p, const double d) {
            pos = p;
            dist = d;
        }
        bool operator<(const el& a) const {
            return dist > a.dist;
        }
    };

    void InitOccFrom(nav_msgs::msg::OccupancyGrid& rInput, nav_msgs::msg::OccupancyGrid& rOutput) {
        rOutput.data.assign(rInput.data.size(), -1);
        rOutput.info = rInput.info;
        rOutput.header = rInput.header;
    }

    bool IsInBounds(nav_msgs::msg::OccupancyGrid& rInput, Vec2i& rPos) {
        return rPos.x >= 0 &&
            rPos.y >= 0 &&
            rPos.x < rInput.info.width &&
            rPos.y < rInput.info.height;
    }

    bool IsInBounds(const int& width, const int& height, Vec2i& rPos) {
        RCLCPP_INFO(rclcpp::get_logger("SearchAlgorithms"), "Visiting child (%d,%d), dim: %d %d", rPos.x, rPos.y, width, height);

        return rPos.x >= 0 &&
            rPos.y >= 0 &&
            rPos.x < width &&
            rPos.y < height;
    }

    bool CheckAny(nav_msgs::msg::OccupancyGrid& rInput, const Vec2i& rStart, const Vec2i& rEnd, const int& rVal) {
        Vec2i start = rStart;
        Vec2i end = rEnd;
        int width = rInput.info.width;
        int height = rInput.info.height;
        if(start.x < 0) start.x = 0;
        if(start.x >= width) start.x = width - 1;
        if(start.y < 0) start.y = 0;
        if(start.y >= height) start.y = height - 1;
        if(end.x < 0) end.x = 0;
        if(end.x > width) end.x = width;
        if(end.y < 0) end.y = 0;
        if(end.y > height) end.y = height;

        // this for should be end inclusive
        // thus is the end.[xy] + 1
        for(int y = start.y; y < end.y + 1; ++y) {
            for(int x = start.x; x < end.x + 1; ++x) {
                if(rInput.data[y*width+x] == rVal) return true;
            }
        }
        return false;
    }

    /*
    * ComputePath is an implementation of the A* algorithm on top of a cell decomposed map.
    * where black pixels are free space, white pixels are obstacles, and
    * blue pixels are frontiers.
    */
    struct MatrixEl{
        Vec2i pred;
        double dist;
        bool visi;
        MatrixEl() {
            pred = Vec2i::Create(-1,-1);
            dist = std::numeric_limits<double>::max();
            visi = false;
        }
    };

    void ComputeValueMap(nav_msgs::msg::OccupancyGrid& rInput,
                         nav_msgs::msg::OccupancyGrid& rOutput,
                              std::vector<Vec2i>& rReached,
                            const double& rMaxLidarRange) {
        rOutput.data.assign(rInput.data.size(), -1);
        rOutput.info = rInput.info;
        rOutput.header = rInput.header;
        double max_value = M_PI * rMaxLidarRange * rMaxLidarRange;
        double max = 0.0;
        for(size_t i = 0; i < rReached.size(); ++i) {
            int index = rReached[i].y * rInput.info.width + rReached[i].x;
            if(index < 0 || index >= rInput.data.size()) {
                RCLCPP_ERROR(rclcpp::get_logger("SearchAlgorithms"), "Index out of bounds in ComputeValueMap: %d", index);
                continue;
            }
            double value = WavefrontPropagationValue(rInput, rReached[i], rMaxLidarRange);
            rOutput.data[index] = static_cast<int8_t>(value);
            if(value > max) max = value;
        }
        for(size_t i = 0; i < rReached.size(); ++i) {
            int index = rReached[i].y * rInput.info.width + rReached[i].x;
            double val_to_norm = (double)(rOutput.data[index]);
            double norm = val_to_norm / max;
            double scaled = norm * 99.0 + 1.0; // scale to [54, 254] range
            rOutput.data[index] = static_cast<int8_t>(scaled);
        }
    }

    double WavefrontPropagationValue(nav_msgs::msg::OccupancyGrid& rInput,
                                    const Vec2i& rStart,
                                    const double& rMaxLidarRange) {
        // initialize distances and predecessors
        // using struct with all elements to optimize
        // cache hits
        int width = rInput.info.width;
        int height = rInput.info.height;

        nav_msgs::msg::OccupancyGrid control;
        control.info = rInput.info;
        control.header = rInput.header;
        control.data.assign(width * height, UNVISITED);
        
        std::queue<Vec2i> to_visit;
        to_visit.push(Vec2i::Create(rStart.x, rStart.y));
        control.data[rStart.y * width + rStart.x] = VISITED;

        Vec2i current;
        Vec2i children;
        bool found = false;
        int index;
        int val;
        double value = 0.0;

        while(to_visit.size() > 0) {
            current = to_visit.front();
            to_visit.pop();

            if(rMaxLidarRange > 0.0) {
                double dx = (rStart.x - current.x) * rInput.info.resolution;
                double dy = (rStart.y - current.y) * rInput.info.resolution;
                double circle_test = dx * dx + dy * dy;
                if(circle_test > rMaxLidarRange * rMaxLidarRange) continue; // skip if outside lidar range
            }
            
            // iterate over children
            for(int x = 0; x < 3; ++x) {
                for(int y = 0; y < 3; ++y) {
                    if(x == y) continue;
                    children = Vec2i::Create(current.x - 1 + x, current.y - 1 + y);
                    index = children.y*control.info.width+children.x;
                    if(children.x < 0 || children.y < 0 || children.x >= width || children.y >= height) continue;
                    if(control.data[index] == VISITED) continue;
                    val = rInput.data[index];
                    // check if it is free space or unknown space, avoid obstacles
                    if(val < 90) {
                        to_visit.push(Vec2i::Create(children.x, children.y));
                        control.data[index] = VISITED;
                        // count for unknown cells from this point of view
                        if(val == -1) value += 1.0;
                    }
                }
            }
        }
        return value * (rInput.info.resolution * rInput.info.resolution); // return area in meters squared
    }

    void WavefrontPropagation(
        nav_msgs::msg::OccupancyGrid& rInput,
        nav_msgs::msg::OccupancyGrid& rControl,
        const Vec2i& rStart,
        std::vector<Vec2i>& rReached) {

        rReached.clear();

        // initialize distances and predecessors
        // using struct with all elements to optimize
        // cache hits
        int width = rInput.info.width;
        int height = rInput.info.height;

        rControl.info = rInput.info;
        rControl.header = rInput.header;
        rControl.data.assign(width * height, UNVISITED);

        std::queue<Vec2i> to_visit;
        to_visit.push(Vec2i::Create(rStart.x, rStart.y));
        rControl.data[rStart.y * width + rStart.x] = VISITED;

        Vec2i current;
        Vec2i children;
        bool found = false;
        int index;
        int val;

        while(to_visit.size() > 0) {
            current = to_visit.front();
            to_visit.pop();
            
            // iterate over children
            for(int x = 0; x < 3; ++x) {
                for(int y = 0; y < 3; ++y) {
                    if(x == y) continue;
                    children = Vec2i::Create(current.x - 1 + x, current.y - 1 + y);
                    index = children.y*rControl.info.width+children.x;
                    if(children.x < 0 || children.y < 0 || children.x >= width || children.y >= height) continue;
                    if(rControl.data[index] == VISITED || rControl.data[index] == REACHED) continue;
                    rControl.data[index] = VISITED;
                    val = rInput.data[index];
                    if(val >= 0 && val < 90) {
                        rControl.data[index] = REACHED;
                        rReached.push_back(children);
                        to_visit.push(Vec2i::Create(children.x, children.y));
                    }
                }
            }
        }
    }

    double ComputeCellValue(nav_msgs::msg::OccupancyGrid& occ, Vec2i& centroid, const double& lidarRange) {
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
        // RCLCPP_INFO(this->get_logger(), "Cells: %d area to uncover in meters squared: %f", count, total_area_value);
        return total_area_value;
    }

    void ComputePath(
        nav_msgs::msg::OccupancyGrid& rInput, 
        const Vec2i& rStart, 
        const Vec2i& rEnd, 
        std::list<Vec2i>& rOutPath) {

        rOutPath.clear();

        // Bounds check for start and end
        if (!IsInBounds(rInput, const_cast<Vec2i&>(rStart)) || !IsInBounds(rInput, const_cast<Vec2i&>(rEnd))) {
            return;
        }

        // initialize distances and predecessors
        // using struct with all elements to optimize
        // cache hits
        Matrix<MatrixEl> control(rInput.info.height, rInput.info.width);
        control.clear(MatrixEl());

        // control variables
        bool found = false;
        double dist = 0.0;
        double heuristic = 0.0;
        double distance_metric = 0.0;
        Vec2i current;
        Vec2i children;

        // initialize search
        std::priority_queue<el> q;
        q.push(el(Vec2i::Create(rStart.x, rStart.y), 0.0));
        control[rStart.y][rStart.x].dist = 0.0;

        // do search
        while(q.size() > 0) {
            // TODO:swap the extract min function to 
            // be more optimized
            current = q.top().pos;
            q.pop();

            // Defensive bounds check for current
            if (!IsInBounds(rInput, current)) continue;
            if(control[current.y][current.x].visi == true) continue;
            control[current.y][current.x].visi = true;

            // stop condition
            if(current == rEnd) {
                found = true;
                break;
            }

            // iterate over children
            for(int x = 0; x < 3; ++x) {
                for(int y = 0; y < 3; ++y) {
                    if(x == 1 && y == 1) continue;

                    children = Vec2i::Create(current.x - 1 + x, current.y - 1 + y);
                    if(IsInBounds(rInput, children)
                       && control[children.y][children.x].visi == false 
                       && rInput.data[children.y*rInput.info.width+children.x] >= 0
                       && rInput.data[children.y*rInput.info.width+children.x] < 50) {
                        // compute distance and heuristic to parent
                        // and end points
                        dist = control[current.y][current.x].dist + Distance(current, children);
                        heuristic = Distance(rEnd, children);
                        distance_metric = dist + heuristic;
                        
                        // relax edge
                        if(control[children.y][children.x].dist > distance_metric) {
                            control[children.y][children.x].dist = distance_metric;
                            control[children.y][children.x].pred = Vec2i::Create(current.x, current.y);
                        } 

                        // if not on the heap, add to the heap
                        el element(children, control[children.y][children.x].dist);
                        q.push(element);
                    }
                }
            }
        }

        // compute output path from search
        if(found) {
            current = rEnd;
            while(current != rStart) {
                if (!IsInBounds(rInput, current)) break; // Defensive check
                rOutPath.push_front(current);
                current = control[current.y][current.x].pred;
            }
        }
    }

    void ComputeFrontiers(nav_msgs::msg::OccupancyGrid& rInput, 
                          nav_msgs::msg::OccupancyGrid& rOutput, 
                          std::vector<Vec2i>& rReached,
                          std::vector<Vec2i>& rFrontiers) {
        rFrontiers.clear();                            
        InitOccFrom(rInput, rOutput);
        Vec2i start;
        Vec2i end;
        int index;
        for(size_t i = 0; i < rReached.size(); ++i) {
            start.x = rReached[i].x - 1; start.y = rReached[i].y - 1;
            end.x   = rReached[i].x + 1; end.y   = rReached[i].y + 1;
            index = rReached[i].y * rInput.info.width + rReached[i].x;
            if(rInput.data[index] >= 0 
                   && rInput.data[index] < 50 
                   && CheckAny(rInput, start, end, -1)) {
               rOutput.data[index] = 100;
               rFrontiers.push_back(Vec2i::Create(rReached[i].x, rReached[i].y));
           }
        }
    }

    void ComputeClusters(nav_msgs::msg::OccupancyGrid& rFrontiersMap, 
                        std::vector<Vec2i>& rFrontiers, 
                        std::vector<std::vector<Vec2i>>& rOutClusters,
                        const int& rMinClusterSize) {
        Matrix<int> visited(rFrontiersMap.info.width, rFrontiersMap.info.height);
        visited.clear(0);
        rOutClusters.clear();

        std::list<Vec2i> q;
        std::vector<Vec2i> cluster;
        Vec2i current;
        Vec2i children;  
        for(size_t f = 0; f < rFrontiers.size(); ++f) {
            current = rFrontiers[f];
            if(visited[current.y][current.x] == 0) {
                q.clear();
                cluster.clear();
                q.push_back(current);
                while(q.size() > 0) {
                    current = q.back();
                    q.pop_back();
                    cluster.push_back(current);
                    visited[current.y][current.x] = 2;
                    for(int y = 0; y < 3; ++y) {
                        for(int x = 0; x < 3; ++x){
                            children = Vec2i::Create(current.x - 1 + x, current.y - 1 + y);
                            if(IsInBounds(rFrontiersMap, children)
                               && visited[children.y][children.x] == 0
                               && rFrontiersMap.data[children.y*rFrontiersMap.info.width+children.x] >= 90) {
                                visited[children.y][children.x] = 1;
                                q.push_back(children);
                            }
                        }
                    }
                }

                // after the flooding search
                // append clusters to clusters list
                if(cluster.size() > rMinClusterSize) {
                    rOutClusters.push_back(cluster);
                }
            }
        }
    }

    Vec2i ClosestFrontierCluster(const Vec2i& rPos, std::vector<Vec2i>& rCluster) {
        int closest = 0;
        double dist = std::numeric_limits<double>::max();
        double temp_dist = 0.0;
        for(size_t i = 0; i < rCluster.size(); ++i) {
            temp_dist = Distance(rCluster[i], rPos);
            if(temp_dist < dist) {
                dist = temp_dist;
                closest = i;
            }
        }
        return Vec2i::Create(rCluster[closest].x, rCluster[closest].y);
    }

    Vec2i MedianFrontierCluster(const Vec2i& rPos, std::vector<Vec2i>& rCluster) {
        std::vector<std::pair<double, Vec2i>> to_sort;
        std::pair<double, Vec2i> min;
        std::pair<double, Vec2i> max;
        min.first = std::numeric_limits<double>::max();
        max.first = std::numeric_limits<double>::min();
        double distance = -1.0;
        double average = 0.0;
        for(size_t i = 0; i < rCluster.size(); ++i) {
            distance = Distance(rCluster[i], rPos);
            to_sort.push_back(std::pair<double, Vec2i>(distance, rCluster[i]));
            if(distance > max.first) {
                max.first = distance;
                max.second = rCluster[i];
            }
            if(distance < min.first) {
                min.first = distance;
                min.second = rCluster[i];
            }
        }
        average = (min.first + max.first)/2.0;
        // seek for near to average
        double dist = std::numeric_limits<double>::max();
        Vec2i near;
        for(size_t i = 0; i < to_sort.size(); ++i) {
            distance = abs(average - to_sort[i].first);
            if(distance < dist) {
                dist = distance;
                near = to_sort[i].second;
            }
        }
        return near;
    }

    void ComputeCentroids(const Vec2i& rPos, 
                        std::vector<std::vector<Vec2i>>& rClusters, 
                        std::vector<Vec2i>& rOutCentroids) {
        rOutCentroids.clear();
        for(size_t i = 0; i < rClusters.size(); ++i) {
            rOutCentroids.push_back(ClosestFrontierCluster(rPos, rClusters[i]));
        }
    }

    void ComputeClusterCenterOfMass(const Vec2i& rPos, 
                        std::vector<std::vector<Vec2i>>& rClusters, 
                        std::vector<Vec2i>& rOutCentroids) {
        rOutCentroids.clear();
        for(size_t i = 0; i < rClusters.size(); ++i) {
            rOutCentroids.push_back(MedianFrontierCluster(rPos, rClusters[i]));
        }
    }
};