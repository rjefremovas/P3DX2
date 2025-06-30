// main.cpp
#include <iostream>
#include <pthread.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <GL/glut.h>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 

#include "Action.h"
#include "Perception.h"
#include "Utils.h"

using std::placeholders::_1;
using namespace std::chrono_literals;


char pressedKey;

sensor_msgs::msg::LaserScan laserROS;
    //geometry_msgs::msg::SharedPtr poseROS;
nav_msgs::msg::Odometry poseROS;

class NavigationNode : public rclcpp::Node
{
public:
  NavigationNode(Action &action, Perception &perception)
      : Node("navigation_p3dx"), action_(action), perception_(perception)
  {
    //pub_twist = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    //sub_laserscan = this->create_subscription<sensor_msgs::msg::LaserScan>("/laser_scan", 100,
    //                                                                       std::bind(&NavigationNode::receiveLaser, this, _1));
    //sub_sonar = this->create_subscription<sensor_msgs::msg::PointCloud2>("/sonar", 100,
    //                                                                     std::bind(&Perception::receiveSonar, &perception, _1));                                                                           
    //sub_pose = this->create_subscription<nav_msgs::msg::Odometry>("/absTruePose", 100,
    //timer_ = this->create_wall_timer(100ms, std::bind(&NavigationNode::timer_callback, this));                                                                                                                              std::bind(&NavigationNode::receivePose, this, _1));
    sub_pose = this->create_subscription<nav_msgs::msg::Odometry>("/absTruePose", 1, std::bind(&Perception::receivePose, &perception, _1));
    pub_twist = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    sub_laserscan = this->create_subscription<sensor_msgs::msg::LaserScan>("/laser_scan", 100,
                                                                           std::bind(&Perception::receiveLaser, &perception, _1));
    sub_sonar = this->create_subscription<sensor_msgs::msg::PointCloud2>("/sonar", 100,
                                                                         std::bind(&Perception::receiveSonar, &perception, _1));
    timer_ = this->create_wall_timer(100ms, std::bind(&NavigationNode::timer_callback, this));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laserscan;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_sonar;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_pose;
  Action &action_;
  Perception &perception_;

  
  
  //std::vector<float> latestSonars = perception_.getLatestSonarRanges();

  /*
  void receiveLaser(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
  {
    laserROS = *msg;
    latestLaser = msg->ranges;
  }

  void receivePose(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    float x = msg->pose.pose.position.x;
    float y = msg->pose.pose.position.y;
    auto q = msg->pose.pose.orientation;
    tf2::Quaternion tf_quat(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    latestPose = {x, y, static_cast<float>(yaw)};
  }
  */

  

  void timer_callback()
  {
    
    std::vector<float> lasers = perception_.getLatestLaserRanges();
    Sonars sonars = perception_.getLatestSonarRanges();
    std::vector<float> pose = perception_.getLatestPose();

    //std::vector<float> latestLaser = perception_.getLatestLaserRanges();
    //std::vector<float> latestPose  = perception_.getLatestPose();
    /*
    std::cout << "Read " << lasers.size() << " laser measurements" << std::endl;
    std::cout << "Read " << sonars.size() << " sonar measurements" << std::endl;
    std::cout << "Read " << pose.size() << " pose measurements" << std::endl;
    */

    // Get keyboard input
    //char ch = pressedKey; 
    //MotionControl mc = action_.handlePressedKey(ch);
    //std::cout << mc.mode << ' ' << mc.direction << std::endl;

    // Compute next action
    /*
    if (mc.mode == MANUAL)
    {
      action_.manualRobotMotion(mc.direction);
    }
    else if (mc.mode == WANDER)
    {
      action_.avoidObstacles(lasers, sonars);
    }
    else if (mc.mode == FARFROMWALLS)
    {
      action_.keepAsFarthestAsPossibleFromWalls(lasers, sonars);
    }
    */
    //std::cout << "antes dos obstaculos \n" ;

    std::vector<float> speed = perception_.updateMap(sonars);
    action_.avoidObstacles(speed);
    //action_.correctVelocitiesIfInvalid();
    //std::cout << "depois dos obstaculos \n" ;-
    geometry_msgs::msg::Twist twistROS;
    twistROS.linear.x = action_.getLinearVelocity();
    twistROS.angular.z = action_.getAngularVelocity();
    pub_twist->publish(twistROS);
    //std::cout << "antes do mapa \n" ;
    
    //std::cout << "depois do mapa \n" ;
    //std::cout << "Published linVel " << twistROS.linear.x << " angVel " << twistROS.angular.z << std::endl;
    //geometry_msgs::msg::Twist twist;
    //twist.linear.x = 0.1;
    //twist.angular.z = 0.1;
    //pub_twist->publish(twist);
  }
};



void *keyboardThreadFunction(void *)
{
  while (pressedKey != 27)
  {
    pressedKey = getCharWithoutWaitingENTER();
  }
  return NULL;
}

void *mainThreadFunction(void *)
{
  Action action;
  Perception perception;
  auto node = std::make_shared<NavigationNode>(action, perception);
  rclcpp::spin(node);
  return NULL;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  pthread_t mainThread, keyboardThread, oglThread;
  pthread_create(&mainThread, NULL, mainThreadFunction, NULL);
  pthread_create(&keyboardThread, NULL, keyboardThreadFunction, NULL);
  pthread_create(&oglThread, NULL, (void *(*)(void *))Perception::displayLoop, NULL);
  //pthread_create(&ogl2Thread2, NULL, (void *(*)(void *))Perception::displayLoop2, NULL);
 
 
  pthread_join(mainThread, NULL);
  pthread_join(keyboardThread, NULL);
  pthread_join(oglThread, NULL);
  //pthread_join(ogl2Thread2, NULL);

  rclcpp::shutdown();
  return 0;
}
