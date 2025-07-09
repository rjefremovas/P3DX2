#ifndef PERCEPTION_H
#define PERCEPTION_H

#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <deque>


// Struct to hold processed sonar data (ranges and angles)
struct Sonars {
    std::vector<float> range;
    std::vector<float> angle;
};

// ===================================================================================
// CRITICAL FIX: The 'Cell' struct for Dijkstra's algorithm.
// The 'dist' member MUST be a float to match the potential field costs.
// ===================================================================================
struct Cell {
    int row, col;
};


class Perception {
public:
    // Constructor
    Perception();

    // ROS message callback functions
    void receiveLaser(const sensor_msgs::msg::LaserScan::ConstSharedPtr &value);
    void receivePose(const nav_msgs::msg::Odometry::ConstSharedPtr &value);
    void receiveSonar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &value);

    // Data processing and retrieval functions
    std::vector<float> getLatestLaserRanges();
    std::vector<float> getLatestPose();
    Sonars getLatestSonarRanges();

    // Core logic for mapping and navigation
    std::vector<float> updateMap(Sonars sonars);

    // Main display loop for visualization
    static void displayLoop();

    // Static functions for GLUT display callbacks
    static void drawMap();
    static void drawMap2();

private:
    // Member variables to store the latest ROS message data
    sensor_msgs::msg::LaserScan laserROS;
    nav_msgs::msg::Odometry poseROS;
    sensor_msgs::msg::PointCloud2 sonarROS;
    std::vector<float> latestPose;
    std::deque<std::pair<int, int>> dijkstraPath;
};

#endif // PERCEPTION_H
