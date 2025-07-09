#ifndef PERCEPTION_H
#define PERCEPTION_H

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

struct Sonars{
    std::vector<float> range;
    std::vector<float> angle;
};

struct Cell {
    int r, c;
    int dist;

    // Custom comparator for priority queue
    bool operator>(const Cell& other) const {
        return dist > other.dist;
    }
};

class Perception
{
public:
    Perception();
     
    std::vector<float> getLatestLaserRanges();
    //std::vector<float> getLatestSonarRanges();
    Sonars getLatestSonarRanges();
    std::vector<float> getLatestPose();
    
    void receiveLaser(const sensor_msgs::msg::LaserScan::ConstSharedPtr &value);
    void receiveSonar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &value);
    void receivePose(const nav_msgs::msg::Odometry::ConstSharedPtr &value);
    std::vector<float> updateMap(Sonars sonars);
    static void drawMap();
    static void drawMap2(); 
    static void displayLoop();


private:
    sensor_msgs::msg::LaserScan laserROS;
    sensor_msgs::msg::PointCloud2 sonarROS;
    //geometry_msgs::msg::SharedPtr poseROS;
    nav_msgs::msg::Odometry poseROS;
    std::vector<float> latestLaser;
    std::vector<float> latestPose;
};

#endif // PERCEPTION_H
