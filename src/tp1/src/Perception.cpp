#include "Perception.h"
#include "Utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cstring>
#include <GL/glut.h>
#include <math.h>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <vector>
#include <iostream>
#include <queue>
#include <limits>
#include <rclcpp/rclcpp.hpp>





// Dijkstra movimento
int dx[] = {-1, 1, 0, 0};
int dy[] = {0, 0, -1, 1};
bool dijkstra = false;
const int MAP_SIZE = 500;
const float RESOLUTION = 0.1f;
bool localGradientFound = false;
bool inDijkstra = false;
std::vector<Cell> path;
int dijkstraCounter = 0;
int localFailureCount = 0;
const int LOCAL_FAILURE_THRESHOLD = 100;
// Bayesian Map
//float map[MAP_SIZE][MAP_SIZE] = {{0.5}};

// ITERATIONS
int MAX_ITERATIONS = 10;

// Velocidades
float linSpeed = 0.0;
float angSpeed = 0.0;
// =========
// HIMM mapa
// =========
// Mapa - desenhado
int map[MAP_SIZE][MAP_SIZE] = {{0}};

// Mapa - harmonico
float map2[MAP_SIZE][MAP_SIZE] = {{0}};

int window1, window2;
int last_px = 9999;
int last_py = 9999; 
float kx = 0;
float ky = 0;
const float PI = 3.14159265359f;
float nanoaux = 0.0;

// Bayesian variables and constants
const float MAX_RANGE = 2.0f;
const float SONAR_WIDTH_RADIANS = 10.0f*PI/180.0f;
const float MAX_OCCUPIED = 0.98f;
const float BETA = 10 * PI/180;

double robot_x = 0.0; 
double robot_y = 0.0; 
double roll = 0.0, pitch = 0.0, robot_yaw = 0.0;
//float robot_yaw = 0.00000;
double angle = 0.0;
long long last_nanoseconds = 0;
int map_x = 0;
int map_y = 0;

std::vector<double> predicted_x;
std::vector<double> predicted_y;
std::vector<double> predicted_yaw;
std::vector<double> ground_truth_x;
std::vector<double> ground_truth_y;
std::vector<double> ground_truth_yaw;

Perception::Perception()
{
}

void Perception::receiveLaser(const sensor_msgs::msg::LaserScan::ConstSharedPtr &value)
{
    //  STRUCTURE OF sensor_msgs::msg::LaserScan

    // Single scan from a planar laser range-finder
    // 
    // If you have another ranging device with different behavior (e.g. a sonar
    // array), please find or create a different message, since applications
    // will make fairly laser-specific assumptions about this data

    // Header header
    //     # Standard metadata for higher-level stamped data types.
    //     # This is generally used to communicate timestamped data
    //     # in a particular coordinate frame.
    //     #
    //     # sequence ID: consecutively increasing ID
    //     uint32 seq
    //     #Two-integer timestamp that is expressed as:
    //     # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    //     # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    //     # time-handling sugar is provided by the client library
    //     time stamp
    //     #Frame this data is associated with
    //     # 0: no frame
    //     # 1: global frame
    //     string frame_id
    //              # timestamp in the header is the acquisition time of
    //              # the first ray in the scan.
    //              #
    //              # in frame frame_id, angles are measured around
    //              # the positive Z axis (counterclockwise, if Z is up)
    //              # with zero angle being forward along the x axis
    laserROS.header = value->header;
    

    // float32 angle_min        # start angle of the scan [rad]
    // float32 angle_max        # end angle of the scan [rad]
    // float32 angle_increment  # angular distance between measurements [rad]
    laserROS.angle_min = value->angle_min;
    laserROS.angle_max = value->angle_max;
    laserROS.angle_increment = value->angle_increment;

    // float32 time_increment   # time between measurements [seconds] - if your scanner
    //                          # is moving, this will be used in interpolating position
    //                          # of 3d points
    // float32 scan_time        # time between scans [seconds]
    laserROS.time_increment = value->time_increment;
    laserROS.scan_time = value->scan_time;

    // float32 range_min        # minimum range value [m]
    // float32 range_max        # maximum range value [m]
    laserROS.range_min = value->range_min;
    laserROS.range_max = value->range_max;

    // float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
    // float32[] intensities    # intensity data [device-specific units].  If your
    //                          # device does not provide intensities, please leave
    //                          # the array empty.
    laserROS.ranges = value->ranges;
    laserROS.intensities = value->intensities;
}



void Perception::receivePose(const nav_msgs::msg::Odometry::ConstSharedPtr
&value)
{
	poseROS.header = value->header;
	poseROS.pose.pose.position.x = value->pose.pose.position.x;
	poseROS.pose.pose.position.y = value->pose.pose.position.y;
	poseROS.pose.pose.orientation = value->pose.pose.orientation;
}


std::vector<float> Perception::getLatestLaserRanges()
{
    int numLasers = laserROS.ranges.size();

    std::vector<float> lasers(numLasers);

    //    std::cout << "LASER: " << numLasers << std::endl;
    for (int i = 0; i < numLasers; i++)
    {
        lasers[i] = laserROS.ranges[numLasers - i - 1];
        if (lasers[i] < 0)
            lasers[i] = 32.0; // max range from rosaria
    }
    //std::cout << laserROS.header.stamp.sec << std::endl;
    return lasers;
}

/*
std::vector<float> Perception::getLatestLaserRanges()
{
  return laserROS.ranges;
}

*/






std::vector<float> Perception::getLatestPose()
{

  float x = poseROS.pose.pose.position.x;
  float y = poseROS.pose.pose.position.y;
  auto q = poseROS.pose.pose.orientation;
  tf2::Quaternion tf_quat(q.x, q.y, q.z, q.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
  latestPose = {x, y, static_cast<float>(yaw)};
  //std::cout << "x: " << x << ", y: " << y << ", yaw: " << yaw << std::endl;

	return latestPose;
}



void Perception::receiveSonar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &value)
{
    //  STRUCTURE OF sensor_msgs::msg::PointCloud2

    // This message holds a collection of N-dimensional points, which may
    // contain additional information such as normals, intensity, etc. The
    // point data is stored as a binary blob, its layout described by the
    // contents of the "fields" array.

    // The point cloud data may be organized 2d (image-like) or 1d
    // (unordered). Point clouds organized as 2d images may be produced by
    // camera depth sensors such as stereo or time-of-flight.

    //Header header
    //    # Standard metadata for higher-level stamped data types.
    //    # This is generally used to communicate timestamped data
    //    # in a particular coordinate frame.
    //    #
    //    # sequence ID: consecutively increasing ID
    //    uint32 seq
    //    #Two-integer timestamp that is expressed as:
    //    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    //    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    //    # time-handling sugar is provided by the client library
    //    time stamp
    //    #Frame this data is associated with
    //    # 0: no frame
    //    # 1: global frame
    //    string frame_id
    sonarROS.header = value->header;
    
    // # 2D structure of the point cloud. If the cloud is unordered, height is
    // # 1 and width is the length of the point cloud.
    // uint32 height
    // uint32 width
    sonarROS.height = value->height;
    sonarROS.width = value->width;

    // # Describes the channels and their layout in the binary data blob.
    // PointField[] fields
    sonarROS.fields = value->fields;

    // bool    is_bigendian # Is this data bigendian?
    // uint32  point_step   # Length of a point in bytes
    // uint32  row_step     # Length of a row in bytes
    sonarROS.is_bigendian = value->is_bigendian;
    sonarROS.point_step = value->point_step;
    sonarROS.row_step = value->row_step;

    // uint8[] data         # Actual point data, size is (row_step*height)
    sonarROS.data = value->data;

    // bool is_dense        # True if there are no invalid points
    sonarROS.is_dense = value->is_dense;
}

Sonars Perception::getLatestSonarRanges()
{
    int numSonars = sonarROS.width;

    float x,y,z;
    
    std::vector<float> sonars(numSonars);
    std::vector<float> sonar_angle(numSonars);

    Sonars s;
    // std::cout << "SONAR: " << numSonars << std::endl;
    for(int n=0; n<numSonars; n++){
        memcpy (&x, &sonarROS.data[n * sonarROS.point_step + sonarROS.fields[0].offset], sizeof (float));
        memcpy (&y, &sonarROS.data[n * sonarROS.point_step + sonarROS.fields[1].offset], sizeof (float));
        memcpy (&z, &sonarROS.data[n * sonarROS.point_step + sonarROS.fields[2].offset], sizeof (float));        
        sonars[n] = sqrt(pow(x,2.0)+pow(y,2.0));
        sonar_angle[n] = atan2(y,x);
    }
    //std::cout << numSonars << "num sonars \n";
    s.range = sonars;
    s.angle = sonar_angle;
    // std::cout << std::endl;
    return s;
}



// Marca os pontos no mapa do laser e a posição absoluta no mapa
std::vector<float> Perception::updateMap(Sonars sonars)
  {
    //std::cout << "Entrou de novo" << "\n";
    for (size_t m = 0; m < MAP_SIZE; m++) {
      for (size_t n = 0; n < MAP_SIZE; n++) {
        if (map[m][n] == -5 || map[m][n] == -6 ){
          map[m][n] = 0;
        }
      }
    }
    //std::cout << "Entrou de novo2" << "\n";
    //std::cout << "nano segundos: " << nanoaux << std::endl;
    float x = latestPose[0];
    float y = latestPose[1];
    float yaw = latestPose[2];
    int cx = MAP_SIZE / 2 + int(x / RESOLUTION);
    int cy = MAP_SIZE / 2 + int(y / RESOLUTION);
    map[cx][cy] = -2;
    // Potencial Harmonico
    bool converged = false;
    int inter = 0;
    float max_diff = 0.0;
    //std::cout << "Entrou de novo3" << "\n";
    while ( !converged && inter <= MAX_ITERATIONS){
      //std::cout << "Entrou de novo33" << "\n";
      inter ++;
      for (int a = cx - 2/RESOLUTION; a < cx + 2/RESOLUTION - 1; a++){
        for (int b = cy - 2/RESOLUTION; b < cy + 2/RESOLUTION - 1; b++){
          if (map2[a][b] >= 0 && map2[a][b] < 1){
            float current_potential = map2[a][b];
            float ps_n = map2[a+1][b];
            float ps_s = map2[a-1][b];
            float ps_w = map2[a][b-1];
            float ps_e = map2[a][b+1];
            float new_potential = ( ps_n + ps_s + ps_w + ps_e)/4.0;
            float diff = std::abs(new_potential - current_potential);
            
            map2[a][b] = new_potential;
            if (diff > max_diff) {
              max_diff = diff;
            }
          }
        }
      }
      if (max_diff < 0.1) {
        converged = true;
      }
    }
    //std::cout << "Entrou de novo4" << "\n";
    /*
    for (int a = 1; a < MAP_SIZE - 1; a++){
        for (int b = 1; b < MAP_SIZE - 1; b++){
          if (map2[a][b] >= 0 && map2[a][b] < 1){
            float current_potential = map2[a][b];
            float ps_n = map2[a+1][b];
            float ps_s = map2[a-1][b];
            float ps_w = map2[a][b-1];
            float ps_e = map2[a][b+1];
            float new_potential = ( ps_n + ps_s + ps_w + ps_e)/4.0;
            float diff = std::abs(new_potential - current_potential);
            
            map2[a][b] = new_potential;
            if (diff > max_diff) {
              max_diff = diff;
            }
          }
        }
      }
    */
      //std::cout << "Entrou de novo5" << "\n";
    int gx = 0;
    int gy = 0;
    float avg = 1.0;
    float minVal = 1;
    localGradientFound = false;
    
      for (int a = cx - 1/RESOLUTION; a < cx + 1/RESOLUTION - 1; a++){
          for (int b = cy - 1/RESOLUTION; b < cy + 1/RESOLUTION - 1; b++){
              float ps_1 = map2[a-1][b+1];
              float ps_2 = map2[a][b+1];
              float ps_3 = map2[a+1][b+1];
              float ps_4 = map2[a-1][b];
              float ps_5 = map2[a][b];
              float ps_6 = map2[a+1][b];
              float ps_7 = map2[a-1][b-1];
              float ps_8 = map2[a][b-1];
              float ps_9 = map2[a+1][b-1];
              avg = (ps_1 + ps_2 + ps_3 + ps_4 + ps_5 + ps_6 + ps_7 + ps_8 + ps_9)/9;
              
            
              if (avg < minVal && avg > 0.1 ){
                float dist = std::sqrt((cx - a)*(cx - a) + (cy-b)*(cy-b));
                if (dist <= 1.0){
                  minVal = avg;
                  gx = a;
                  gy = b;
                  if (avg < 0.8f){
                    localGradientFound = true;
                  }
              }
            }
        }
      }
    
    if (!localGradientFound) {
        localFailureCount++;
        RCLCPP_WARN(rclcpp::get_logger("Perception"), "Local gradient not found (%d/%d)", localFailureCount, LOCAL_FAILURE_THRESHOLD);
        //std::cout << localFailureCount << "\n";
    } else {
        localFailureCount = 0;
        //RCLCPP_INFO(rclcpp::get_logger("Perception"), "Local gradient found. Resetting failure counter.");
    }
    //std::cout << "Antes do Dijkstra" << "\n";
    if (localFailureCount >= LOCAL_FAILURE_THRESHOLD && inDijkstra == false ){
      RCLCPP_ERROR(rclcpp::get_logger("Perception"), "[FALLBACK] Local pathfinding failed. Triggering Dijkstra.");
      localFailureCount = 0;
      /*
      for (int a = 1; a < MAP_SIZE - 31; a++){
          for (int b = 1 ; b < MAP_SIZE - 31; b++){
              int maxInC = -9999;
              int minInC = 10000;
              int maxInD = -9999;
              int minInD = 10000;
              for (int c = b; c < b+30; c++){
                if (map2[a][c] > 0.95f){
                  if (c > maxInC){
                    maxInC = c;
                  } 
                  if (c < minInC){
                    minInC = c;
                  }
                }
              }
              for (int d = a; d < a+30; d++){
                if (map2[d][b] > 0.95f){
                  
                  if (d > maxInD){
                    
                    maxInD = d;
                  } 
                  if (d < minInD){
                    minInD = d;
                  }
                }
              }
              
              std::cout << "mininD " << minInD << "\n";
              std::cout << "maxinD " << maxInD << "\n";
              std::cout << "maxInC " << maxInC << "\n";
              std::cout << "minInC " << minInC << "\n";
              
              std::cout << "a " << a << "\n";
              std::cout << "b " << b << "\n";
              
              float ps_1 = map2[a-1][b+1];
              float ps_2 = map2[a][b+1];
              float ps_3 = map2[a+1][b+1];
              float ps_4 = map2[a-1][b];
              float ps_5 = map2[a][b];
              float ps_6 = map2[a+1][b];
              float ps_7 = map2[a-1][b-1];
              float ps_8 = map2[a][b-1];
              float ps_9 = map2[a+1][b-1];
              float avg = (ps_1 + ps_2 + ps_3 + ps_4 + ps_5 + ps_6 + ps_7 + ps_8 + ps_9)/9;
              if (avg < minVal && avg > 0.05 && avg < 0.95f && (( b > minInC && b < maxInC ) || ( a > minInD && a < maxInD ))){
                minVal = avg;
                gx = a;
                gy = b;
            }
        }
      }
      */

      for (int a = 1; a < MAP_SIZE - 31; a++) {
          for (int b = 1 ; b < MAP_SIZE - 31; b++) {
              std::vector<int> obsC;
              std::vector<int> obsD;
              for (int c = b; c < b+30; c++) {
                  if (map2[a][c] > 0.99f) obsC.push_back(c);
              }
              // Find all obstacle rows in column 'b'
              for (int d = a; d < a+30; d++) {
                  if (map2[d][b] > 0.99f) obsD.push_back(d);
              }
              
              for (size_t i = 1; i < obsC.size(); i++) {
                  int left = obsC[i-1];
                  int right = obsC[i];
                  for (int mid = left+1; mid < right; mid++) {
                      float val = map2[a][mid];
                      if (val < minVal && val > 0.1f && val < 0.95f) {
                          minVal = val;
                          gx = a;
                          gy = mid;
                      }
                  }
              }
              
              for (size_t i = 1; i < obsD.size(); i++) {
                  int top = obsD[i-1];
                  int bottom = obsD[i];
                  for (int mid = top+1; mid < bottom; mid++) {
                      float val = map2[mid][b];
                      if (val < minVal && val > 0.1f && val < 0.95f) {
                          minVal = val;
                          gx = mid;
                          gy = b;
                      }
                  }
              }
          }
      }

      Cell source = {cx, cy};
      Cell dest   = {gx, gy};

      float dist[MAP_SIZE][MAP_SIZE];
      Cell parent[MAP_SIZE][MAP_SIZE];

      for (int i = 0; i < MAP_SIZE; ++i){
        for (int j = 0; j < MAP_SIZE; ++j) {
            dist[i][j] = std::numeric_limits<float>::max();
            parent[i][j] = {-1, -1};
        }
      }
      dist[source.row][source.col] = 0.0f;
      std::vector<std::pair<int, int>> directions = {{-1,0},{1,0},{0,-1},{0,1}};

      typedef std::tuple<float, int, int> State;
      std::priority_queue<State, std::vector<State>, std::greater<State>> pq;
      pq.push({0.0f, source.row, source.col});


      while (!pq.empty()) {
        auto [d, r, c] = pq.top(); 
        pq.pop();
        if (d > dist[r][c]) continue;

        for (auto [dr, dc] : directions) {
            int nr = r + dr, nc = c + dc;
            if (nr < 0 || nr >= MAP_SIZE || nc < 0 || nc >= MAP_SIZE) continue;
            if (map2[nr][nc] >= 0.999f || map2[nr][nc] <= 0.05f) continue; // obstacle
            float cost = map2[nr][nc] + 1; 
            if (dist[r][c] + cost < dist[nr][nc]) {
                dist[nr][nc] = dist[r][c] + cost;
                parent[nr][nc] = {r, c};
                pq.push({dist[nr][nc], nr, nc});
            }
        }
      }
      // Reconstruct path
      
      Cell at = dest;
      if (dist[at.row][at.col] == std::numeric_limits<float>::max()) {
          std::cout << "No path found!\n";
      } else {
          while (at.row != -1 && at.col != -1) {
              path.push_back(at);
              at = parent[at.row][at.col];
          }
          
          std::reverse(path.begin(), path.end());
          
          std::cout << "Shortest path from (" << source.row << "," << source.col << ") to ("
              << dest.row << "," << dest.col << "):\n";
          for (auto cell : path) {
              std::cout << "(" << cell.row << "," << cell.col << ") ";
          }
          
          inDijkstra = true;
          std::cout << "\nPath cost: " << dist[dest.row][dest.col] << std::endl;
      }

    }

    //std::cout << "Depois do Dijkstra" << "\n";
    
    /*
    std::cout << "GX: " << gx << "\n";
    std::cout << "GY: " << gy << "\n";
    std::cout << "CX: " << cx << "\n";
    std::cout << "CY: " << cy << "\n";
    float v = std::sqrt(cx*cx + cy*cy);
    float u = std::sqrt(gx*gx + gy*gy);
    float angleGoal = std::acos((cx*gx + cy*gy)/(v*u));
    
    float angDiff = angleGoal - yaw;
    float normalized_diff = std::fmod(angDiff + PI, 2 * PI);
    if (normalized_diff < 0) {
        normalized_diff += 2 * PI;
    }
    normalized_diff -= PI;
    std::cout << "Normalized diff: " << normalized_diff << "\n";
    */
    //std::cout << "Antes do Angulo" << "\n";
    if (gx != 0 && gy != 0 && inDijkstra == false){
      float angleGoal = std::atan2(gy - cy, gx - cx);
      float angDiff = angleGoal - yaw;
      if (angDiff >= 0){
        if (angDiff <= PI/4){
          linSpeed = 0.3;
          angSpeed = 0.3;
        } else {
          angSpeed = 0.3;
          linSpeed = 0.0;
        }
      } else {
        if (angDiff >= -1*PI/4){
          linSpeed = 0.3;
          angSpeed = -0.3;
        } else {
          angSpeed = -0.3;
          linSpeed = 0.0;
        }
      }
    } else{
      angSpeed = 0.0;
      linSpeed = 0.0;
    }
    std::vector<float> speed;
    speed.resize(2);
    speed[0] = linSpeed;
    speed[1] = angSpeed;
    // HIMM
    float raio;
    for (int k=0; k < int(8); k++)
    {
      raio = sonars.range[k];
      //std::cout << "sonar "<< k << ": " << sonars.range[k] << "\n";
      //std::cout << "angle "<< k << ": " << sonars.angle[k] << "\n";
      for (float v = 0.0; v <= raio; v += raio/5)
      {
        int px = cx + int(((v) * cos(sonars.angle[k]+yaw)) / RESOLUTION);
        int py = cy + int(((v) * sin(sonars.angle[k]+yaw)) / RESOLUTION); 
         
        if ( px != last_px && py != last_py && raio <= 3) {
          if ( v == raio){
            map[px][py] = map[px][py] + 3;
            map2[px][py] = 1;
          } else if (map[px][py] > 2) {
            map[px][py] = map[px][py] - 1;
          } else {
            map[px][py] = map[px][py];
          }
        } 
        last_px = px;
        last_py = py;
        for (float l = sonars.angle[k]+yaw-BETA; l < sonars.angle[k]+yaw+BETA && raio < 3; l+= (BETA)/20 )
        {
            int mx = cx + int(((v) * cos(l)) / RESOLUTION);
            int my = cy + int(((v) * sin(l)) / RESOLUTION); 
            if ( map[mx][my] == 0){
              map[mx][my] = -5;
            }
            
        }
      }

        


    }
    //std::cout << "Depois do Angulo" << "\n";

    // Bayesiana
    /*
    for (int k=0; k < int(8); k++)
    {
      
      for (size_t i = 0; i < laserROS.ranges.size(); ++i)
      {
        float r = laserROS.ranges[i];
        float angle = laserROS.angle_min + i * laserROS.angle_increment + yaw;
        if (r > 0.005 && r < MAX_RANGE && angle < sonars.angle[k]+yaw+BETA && angle >= sonars.angle[k]+yaw-BETA)
        { 
          float alpha;
          float sonar_angle; 
          float normalized_angle;
          float normalized_alpha;
          sonar_angle = fmodf(sonars.angle[k]+yaw + PI, 2.0 * PI);
          if (sonar_angle < 0.0) {
            sonar_angle += 2.0 * PI;
          }
          sonar_angle = sonar_angle - PI;

          normalized_angle = fmodf(angle + PI, 2.0 * PI);
          if (normalized_angle < 0.0) {
            normalized_angle += 2.0 * PI;
          }
          normalized_angle = normalized_angle - PI;
          angle = normalized_angle;

          std::cout << "Sonar Angle:" << sonar_angle << "\n";
          
          if (angle > sonar_angle){
            alpha = angle - sonar_angle;
          } else {
            alpha = sonar_angle - angle;
          }
          
          normalized_alpha = fmodf(alpha + PI, 2.0 * PI);
          if (normalized_alpha < 0.0) {
            normalized_alpha += 2.0 * PI;
          }
          normalized_alpha = normalized_alpha - PI;
          alpha = normalized_alpha;
          

          int lx = cx + int((r * cos(angle)) / RESOLUTION);
          int ly = cy + int((r * sin(angle)) / RESOLUTION);
          if (lx >= 0 && lx < MAP_SIZE && ly >= 0 && ly < MAP_SIZE)
          {
            std::cout << "Beta: " << BETA << "\n";
            std::cout << "Alpha: " << alpha << "\n";
            std::cout << "Laser: " << laserROS.ranges[i] << "\n";

            // Abordagem normal
            //float p_occupied = (((MAX_RANGE-laserROS.ranges[i])/MAX_RANGE) + (BETA-alpha)/BETA)/2;

            // Abordagem teste pra valores mais próximos ao robô, mas não deu certo, fica ruim
            //float p_occupied = ((0.5 - (MAX_RANGE-laserROS.ranges[i])/MAX_RANGE) + (BETA-alpha)/BETA)/2;
            
            // Testei com função parabólica para valores mais próximos a metade do raio -0.5x²+x, mas fica bem ruim
            //float p_occupied = (((-0.5f*laserROS.ranges[i]*laserROS.ranges[i] + laserROS.ranges[i])) + (BETA-alpha)/BETA)/2;

            float p_empty = 1 - p_occupied;
            
            if (map[lx][ly] <= 0){
              map[lx][ly] = 0.5;
            }
            float new_occupied = map[lx][ly]*p_occupied/(p_occupied*map[lx][ly] + (1-map[lx][ly])*p_empty);
            std::cout << "Occupied: " << p_occupied << "\n";
            std::cout << "New Occupied: " << new_occupied << "\n";
            map[lx][ly] = new_occupied;
          }
        }
      }
      float raio = sonars.range[k];  
      //std::cout << "sonar "<< k << ": " << sonars.range[k] << "\n";
      //std::cout << "angle "<< k << ": " << sonars.angle[k] << "\n";
      for (float v = 0.0; v < raio ; v += 0.8)
      {
        for (float l = sonars.angle[k]+yaw-BETA; l < sonars.angle[k]+yaw+BETA; l+= (BETA)/20 )
        {
            int mx = cx + int(((v) * cos(l)) / RESOLUTION);
            int my = cy + int(((v) * sin(l)) / RESOLUTION); 
             
            if ( map[mx][my] == 0){
              map[mx][my] = -5;
            }
            
        }
      }
    }
    */


    /*
    double current_time_seconds = laserROS.header.stamp.sec + laserROS.header.stamp.nanosec * 1e-9;
    double delta_time = 0.0;

    if (last_nanoseconds != 0) {
        double last_time_seconds = static_cast<double>(laserROS.header.stamp.sec) + static_cast<double>(last_nanoseconds) * 1e-9;
        delta_time = current_time_seconds - last_time_seconds;
        if (delta_time < 0) {
            delta_time += 1.0;
        }
        std::cout << "delta_time: " << delta_time << "\n";
    }
    //delta_time = 0.20;
    last_nanoseconds = laserROS.header.stamp.nanosec;
    if (delta_time < 1.0) 
    {
        float angular_change = angVel * delta_time;
        angle += angular_change;
        if (angle > M_PI) {
          angle -= 2 * M_PI;
        }
        if (angle <= -M_PI) {
          angle += 2 * M_PI;
        }
        //tf2::Quaternion tf_quat(q.x, q.y, q.z, q.w);
        tf2::Quaternion tf_quat;
        tf_quat.setRPY(0.0, 0.0, angle);
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, robot_yaw);
        
        robot_x += vel * cos(robot_yaw) * delta_time;
        robot_y += vel * sin(robot_yaw) * delta_time;
        
        std::cout << "angvel: " << angVel << "\n";
        std::cout << "robot_x: " << robot_x << "\n";
        std::cout << "robot_y: " << robot_y << "\n";
        std::cout << "robot_yaw: " << robot_yaw << "\n";

        map_x = MAP_SIZE / 2 + static_cast<int>(robot_x / RESOLUTION);
        map_y = MAP_SIZE / 2 + static_cast<int>(robot_y / RESOLUTION);
        std::cout << "M_PI: " << M_PI << "\n";
        std::cout << "map_x: " << map_x << "\n";
        std::cout << "map_y: " << map_y << "\n";
        map[map_x][map_y] = -3;
    }
    */
    
    
    // Ground Truth
    /*
    for (size_t i = 0; i < laserROS.ranges.size(); ++i)
    {
      float r = laserROS.ranges[i];
      float angle = laserROS.angle_min + i * laserROS.angle_increment + yaw;
      if (r > 0.005 && r < 10)
      {
        for (float v = 0.0; v < r ; v += r/15){
            int mx = cx + int(((v) * cos(angle)) / RESOLUTION);
            int my = cy + int(((v) * sin(angle)) / RESOLUTION);  
            map[mx][my] = -5; 
          }
        int lx = cx + int((r * cos(angle)) / RESOLUTION);
        int ly = cy + int((r * sin(angle)) / RESOLUTION);
        if (lx >= 0 && lx < MAP_SIZE && ly >= 0 && ly < MAP_SIZE)
        {
          map[lx][ly] = -4;
        }
      }
    }
    */
    //std::cout << "Antes de desenhar o robo" << "\n";

    // Desenha o robô(é um retângulo)
    //comprimento é o X = 38cm
    //largura é o Y = 45cm
    float cos_yaw = cos(yaw);
    float sin_yaw = sin(yaw);
    float size_l_robot = -0.19;
    float size_w_robot = -0.225;
    while (size_l_robot <= 0.19){
      size_w_robot = -0.225;
      while (size_w_robot <= 0.225){
        float rotated_offset_map_x = float(size_l_robot) * cos_yaw - float(size_w_robot) * sin_yaw;
        float rotated_offset_map_y = float(size_l_robot) * sin_yaw + float(size_w_robot) * cos_yaw;
        //std::cout << rotated_offset_map_x << "\n" ;
        //std::cout << rotated_offset_map_y << "\n" ;
        int w_robot = int(float(cy) + (rotated_offset_map_y)/float(RESOLUTION));
        int l_robot = int(float(cx) + (rotated_offset_map_x)/float(RESOLUTION));
        //std::cout << "Robot_Length: " << l_robot << "\n";
        //std::cout << "Robot_Width: " << w_robot << "\n";
        if (map[l_robot][w_robot] == 0){
          map[l_robot][w_robot] = -6;
        }
        size_w_robot = size_w_robot + 0.02f;
        //std::cout << "Robot_Width: " << size_w_robot << "\n";   
      }
      
      size_l_robot = size_l_robot + 0.02f;
      
    }

    //std::cout << "Depois de desenhar o robo" << "\n";
    
    // O que o robô pensa que tá fazendo
    /*
    std::cout << "laser " << laserROS.ranges[135] << "  x: " << x << "  y: " << y << "  yaw: " << yaw << std::endl;
    if ( vel > 0.0f) {
      for (size_t i = 0; i < laserROS.ranges.size(); ++i)
      {
        float r = laserROS.ranges[i];
        float angle = laserROS.angle_min + i * laserROS.angle_increment + robot_yaw;
        if (r > 0.05 && r < 4)
        {
          
          int ex = map_x + int((r * cos(angle)) / RESOLUTION);
          int ey = map_y + int((r * sin(angle)) / RESOLUTION);
          map[ex][ey] += 1;
        }
      }
    std::cout << "nano segundos: " << nanoaux << std::endl;
    }
    */
    /*
    // Calcula o erro quadrado médio da posição e do yaw
    predicted_x.push_back(map_x);
    predicted_y.push_back(map_y);
    predicted_yaw.push_back(robot_yaw);
    ground_truth_x.push_back(cx);
    ground_truth_y.push_back(cy);
    ground_truth_yaw.push_back(yaw);

    int n = predicted_x.size();

    double sum_error_x = 0.0;
    double sum_error_y = 0.0;
    double sum_error_yaw = 0.0;

    for (int i = 0; i < n; ++i) {
        double error_x = predicted_x[i] - ground_truth_x[i];
        double error_y = predicted_y[i] - ground_truth_y[i];
        double error_yaw = predicted_yaw[i] - ground_truth_yaw[i];
        sum_error_x += error_x * error_x;
        sum_error_y += error_y * error_y;
        sum_error_yaw += error_yaw * error_yaw;
    }
    double mse_x = sum_error_x / n;
    double mse_y = sum_error_y / n;
    double mse_yaw = sum_error_yaw / n;

    double rmse_x = std::sqrt(mse_x);
    double rmse_y = std::sqrt(mse_y);
    double rmse_yaw = std::sqrt(mse_yaw);

    std::cout << "RMSE X: " << rmse_x << std::endl;
    std::cout << "RMSE Y: " << rmse_y << std::endl;
    std::cout << "RMSE Y: " << rmse_yaw << std::endl;
    */
    //std::cout << "Antes da lógica do Dijkstra angulo" << "\n";
    if (dijkstraCounter > 100){
      localFailureCount = 200;
      dijkstraCounter = 0;
      inDijkstra = false;
      path.clear();

    }
    if (!path.empty() && minVal > 0.1 && map2[cx][cy] > 0.1f && map2[cx][cy] < 1.0f && inDijkstra == true) {
    // Get robot's current position
    std::cout << "[DEBUG] Dijkstra path was not cleared before update. Size = " << dijkstraPath.size() << "\n";
    dijkstraCounter++;
    
    int target_cx =path.front().row;
    int target_cy =path.front().col;
    
    /*
    float target_x = (target_cx - MAP_SIZE / 2) * RESOLUTION;
    float target_y = (target_cy - MAP_SIZE / 2) * RESOLUTION;
    std::cout << target_x << "\n";
    std::cout << target_y << "\n";
    float dx = target_x - x;
    float dy = target_y - y;
    
    float dist_to_target = std::sqrt(dx * dx + dy * dy);
    */
    float dist_to_target = std::sqrt(std::pow(target_cx - cx, 2) + std::pow(target_cy - cy, 2));
    
    // If robot is within 30cm of the target, pop it
    
    if (dist_to_target < 10.0f) {
        /*
        while(dist_to_target2 < 0.1f || !path.empty()){
          path.erase(path.begin());
          if (!path.empty()) {
            int target_cx = path.front().row;
            int target_cy = path.front().col;
            dist_to_target2 = std::sqrt(std::pow(target_cx - cx, 2) + std::pow(target_cy - cy, 2));
            std::cout << target_cx << " target_cx target_cy " << target_cy << "\n";
          }
          std::cout << target_cx << "\n";
          std::cout << target_cy << "\n";
          
        }
        */
       while (!path.empty()) {
          std::cout << std::endl;
          int target_cx = path.front().row;
          int target_cy = path.front().col;
          float dist_to_target2 = std::sqrt(std::pow(target_cx - cx, 2) + std::pow(target_cy - cy, 2));
          //std::cout << target_cx << " target_cx target_cy " << target_cy << "\n";
          //std::cout << dist_to_target << "dist_to_target" << "\n";
          // If close enough, pop and continue to the next point
          if (dist_to_target2 < 10.0f) {
              path.erase(path.begin());
          } else {
              // Not close to next point; stop popping.
              break;
          }
      }
       if (path.empty()) {
          localFailureCount = 0;
          linSpeed = 0.0f;
          angSpeed = 0.0f;
          inDijkstra = false;
          dijkstraCounter = 0;
      }
    } 
    std::cout << "Path size: " << path.size() << std::endl;
          for (const auto& cell : path) {
              map[cell.row][cell.col] = -5;
              std::cout << "(" << cell.row << "," << cell.col << ") ";
          }
    std::cout << "\nCX " << cx << "\n";
    std::cout << "CY " << cx << "\n";
    if (!path.empty()){
        float angle_to_target = std::atan2(target_cy-cy, target_cx-cx);
        float angle_diff = angle_to_target - yaw;
        if (angle_diff >= 0){
          if (angle_diff <= PI/4){
            linSpeed = 0.3;
            angSpeed = 0.3;
          } else {
            angSpeed = 0.3;
            linSpeed = 0.0;
          }
        } else {
          if (angle_diff >= -1*PI/4){
            linSpeed = 0.3;
            angSpeed = -0.3;
          } else {
            angSpeed = -0.3;
            linSpeed = 0.0;
          }
        }
      
    }
    
  }
    //std::cout << "Dps do lógica do Dijkstra angulo" << "\n";
    speed[0] = linSpeed;
    speed[1] = angSpeed;
    //std::cout << linSpeed << "\n";
    //std::cout << angSpeed << "\n";
    return speed;
  }

void Perception::drawMap()
{
  glClear(GL_COLOR_BUFFER_BIT);
  glPointSize(4);
  glBegin(GL_POINTS);
  for (int x = 0; x < MAP_SIZE; ++x)
  {
    for (int y = 0; y < MAP_SIZE; ++y)
    {
      if (map2[x][y] > 0 && map2[x][y] < 1 )
      {
        glColor3f(map2[x][y], 0.0f, 0.0f);
        glVertex2f(x, y);
      } else if (map2[x][y] == 1){
        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex2f(x, y);
      } else {
        glColor3f(0.0f, 0.0f, 0.0f);
        glVertex2f(x, y);
      }
    }
  }
  glEnd();
  glutSwapBuffers();
}

void Perception::drawMap2()
{
  glClear(GL_COLOR_BUFFER_BIT);
  glPointSize(4);
  glBegin(GL_POINTS);
  for (int x = 0; x < MAP_SIZE; ++x)
  {
    for (int y = 0; y < MAP_SIZE; ++y)
    {
      if (map[x][y] > 5)
      {
        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex2f(x, y);
      
      } else if (map[x][y] >= 0 && map[x][y] <= 1 )
      {
        glColor3f(map[x][y], map[x][y],map[x][y]);
        glVertex2f(x, y);
      }else if (map[x][y] == -5)
      {
        glColor3f(1.0f, 1.0f, 1.0f);
        glVertex2f(x, y);
      } else if (map[x][y] == -6)
      {
        glColor3f(0.0f, 1.0f, 1.0f);
        glVertex2f(x, y);
      }
      else if (map[x][y] == -2)
      {
        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex2f(x, y);
      }
    }
  }
  glEnd();
  glutSwapBuffers();
}

/*
void Perception::displayLoop()
{
  int argc = 1;
  char *argv[1] = {(char *)""};
  glClearColor(1.0f, 1.0f, 1.0f, 0.0f);    
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
  glutInitWindowSize(1000, 1000);
  glutCreateWindow("2D SLAM Map");
  glMatrixMode(GL_PROJECTION);
  gluOrtho2D(0, MAP_SIZE, 0, MAP_SIZE);
  glutDisplayFunc(Perception::drawMap2);
  glutIdleFunc(Perception::drawMap2);
  glutMainLoop();
}

void Perception::displayLoop2()
{
  int argc = 1;
  char *argv[1] = {(char *)""};
  glClearColor(1.0f, 1.0f, 1.0f, 0.0f);   
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
  glutInitWindowSize(300, 300);
  glutCreateWindow("2D Harmonic map");
  glMatrixMode(GL_PROJECTION);
  gluOrtho2D(0, MAP_SIZE, 0, MAP_SIZE);
  glutDisplayFunc(Perception::drawMap);
  glutIdleFunc(Perception::drawMap);
  glutMainLoop();
}
*/

void updateWindows() {
    // Set the context to the first window and post a redisplay request
    glutSetWindow(window1);
    glutPostRedisplay();

    // Set the context to the second window and post a redisplay request
    glutSetWindow(window2);
    glutPostRedisplay();
}


void Perception::displayLoop(){
  int argc = 1;
  char* argv[1] = {(char*)""};
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
  glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
 
  glutInitWindowSize(2*MAP_SIZE, 2*MAP_SIZE);
  window1 = glutCreateWindow("SLAM");
  glMatrixMode(GL_PROJECTION);
  gluOrtho2D(0, MAP_SIZE, 0, MAP_SIZE);
  glutDisplayFunc(Perception::drawMap2);
  
  glutInitWindowSize(2*MAP_SIZE, 2*MAP_SIZE);
  window2 = glutCreateWindow("Harmonico");
  glMatrixMode(GL_PROJECTION);
  gluOrtho2D(0, MAP_SIZE, 0, MAP_SIZE);
  glutDisplayFunc(Perception::drawMap);

  glutIdleFunc(updateWindows);
  
  glutMainLoop();
}