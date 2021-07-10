#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "math.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"


// For convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::cout;
using std::endl;


// Define constants
const double TIME_STEP = 0.02;
const double SPEED_LIMIT = 50.0;
const double MERGE_DISTANCE = 30.0;
const double CLOSE_DISTANCE = 25.0;
const double REACTION = 0.5;
const int INITIAL_LANE = 1;


class Point
{
  public:
    double x;
    double y;
    Point()
    {
      this->x = 0.0;
      this->y = 0.0;
    }
    Point(double x, double y)
    {
      this->x = x;
      this->y = y;
    }
};


class Car
{
  private:
    Point velocity;
  public:
    double speed;
    float d;
    double s;
    double future_s;
    Car(double velocity_x, double velocity_y, float d, double s, int prev_size);
    bool is_in_lane(int lane);
    bool is_too_close(double s);
    bool can_be_merged(double s);
};


Car::Car(double velocity_x, double velocity_y, float d, double s, int prev_size)
{
  this->velocity.x = velocity_x;
  this->velocity.y = velocity_y;
  this->speed = sqrt(pow(this->velocity.x, 2.0) + pow(this->velocity.y, 2.0));
  this->d = d;
  this->s = s;
  this->future_s = this->s + ((double)prev_size*TIME_STEP*this->speed);
}


bool Car::is_in_lane(int lane)
{
  if (this->d < (2+4*lane+2) && this->d > (2+4*lane-2))
  {
    return true;
  }
  else
  {
    return false;
  }
}


bool Car::is_too_close(double s)
{
  if ((this->future_s > s) && (this->future_s - s < CLOSE_DISTANCE))
  {
    return true;
  }
  else
  {
    return false;
  }
}


bool Car::can_be_merged(double s)
{
  if ((this->s > s - MERGE_DISTANCE) && (this->s < s + MERGE_DISTANCE))
  {
    return false;
  }
  else
  {
    return true;
  }
}


class AutonomousCar
{
  public:
    Point position;
    double s;
    double d;
    double yaw;
    double speed;
    double target_vel;
    int lane;
    bool too_close;
    bool safe;
    AutonomousCar();
    void update(double x, double y, double s, double d, double yaw, double speed);
};


AutonomousCar::AutonomousCar()
{
  this->lane = INITIAL_LANE;
  this->target_vel = 0;
}


void AutonomousCar::update(double x, double y, double s, double d, double yaw, double speed)
{
  this->position.x = x;
  this->position.y = y;
  this->s = s;
  this->d = d;
  this->yaw = yaw;
  this->speed = speed;
  this->too_close = false;
  this->safe = false;
}


// Create autonomous car object 
AutonomousCar autonomous_car = AutonomousCar();


int main()
{
  // Web socket object
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";

  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  // Load in map data from csv file
  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);
  string line;
  while (getline(in_map_, line))
  {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // Websocket communitcation
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode)
  {
    if (length && length > 2 && data[0] == '4' && data[1] == '2') 
    {
      auto s = hasData(data);
      if (s != "")
      {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry")
        {
        
          // Main car's localization data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];

          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          json msgJson;

          // Initialise trajectories to define a path made up of (x,y) points that the car will visit sequentially every .02 seconds.
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          // Determine how many points are remaining in the path from the last calculation
          int prev_size = previous_path_x.size();

          // Place car at end of last path
          if (prev_size > 0)
          {
            car_s = end_path_s;
          }

          // Update autonomous car object
          autonomous_car.update(car_x, car_y, car_s, car_d, car_yaw, car_speed);

          // Loop over all cars in scene
          for (int i = 0; i < sensor_fusion.size(); i++)
          {
            // Create car object
            Car car = Car(sensor_fusion[i][3], sensor_fusion[i][4], sensor_fusion[i][6], sensor_fusion[i][5], prev_size);

            // Is the car in my lane?
            if(car.is_in_lane(autonomous_car.lane))
            {

              // Is the car too close to me?
              if(car.is_too_close(autonomous_car.s))
              {
                autonomous_car.too_close = true;

                // Try to change to the left lane to overtake
                if (autonomous_car.safe == false and autonomous_car.lane > 0)
                {
                  autonomous_car.safe = true;
                  int target_lane = autonomous_car.lane - 1;
                  for (int j = 0; j < sensor_fusion.size(); j++)
                  {
                    // Create merge car object
                    Car merge_car = Car(sensor_fusion[j][3], sensor_fusion[j][4], sensor_fusion[j][6], sensor_fusion[j][5], prev_size);

                    // Is the merge car in the target lane?
                    if(merge_car.is_in_lane(target_lane))
                    {

                      // Is it safe to merge to the lane with this car here?
                      if(merge_car.can_be_merged(autonomous_car.s) == false)
                      { 
                        autonomous_car.safe = false;
                      }
                    }
                  }

                  // If it is safe change the terget lane
                  if (autonomous_car.safe == true)
                  {
                    autonomous_car.lane = target_lane;
                  }
                }

                // Try to change to the right lane to overtake
                if (autonomous_car.safe == false and autonomous_car.lane < 2)
                {
                  autonomous_car.safe = true;
                  int target_lane = autonomous_car.lane + 1;
                  for (int j = 0; j < sensor_fusion.size(); j++)
                  {

                    // Create merge car object
                    Car merge_car = Car(sensor_fusion[j][3], sensor_fusion[j][4], sensor_fusion[j][6], sensor_fusion[j][5], prev_size);

                    // Is the merge car in the target lane?
                    if(merge_car.is_in_lane(target_lane))
                    {

                      // Is it safe to merge to the lane with this car here?
                      if(merge_car.can_be_merged(autonomous_car.s) == false)
                      { 
                        autonomous_car.safe = false;
                      }
                    }
                  }

                  // If it is safe change the terget lane
                  if (autonomous_car.safe == true)
                  {
                    autonomous_car.lane = target_lane;
                  }
                }
              }
            }
          }

          // If too close to car enfront -> slow down
          if (autonomous_car.too_close)
          {
            autonomous_car.target_vel -= REACTION;
          }

          // If not -> reach just under speed limit
          else if (autonomous_car.target_vel < SPEED_LIMIT - 0.5)
          {
            autonomous_car.target_vel += REACTION;
          }

          // Create vector of points to act as waypoints to build splines
          vector<double> pstx;
          vector<double> psty;

          // Get the ref positions
          double ref_x = autonomous_car.position.x;
          double ref_y = autonomous_car.position.y;
          double ref_yaw = deg2rad(autonomous_car.yaw);

          // Add two waypoints of the currrent and last car position to ensure smoothness
          if (prev_size < 2)
          {
            double prev_car_x = autonomous_car.position.x - cos(autonomous_car.yaw);
            double prev_car_y = autonomous_car.position.y - sin(autonomous_car.yaw);
            pstx.push_back(prev_car_x);
            pstx.push_back(autonomous_car.position.x);
            psty.push_back(prev_car_y);
            psty.push_back(autonomous_car.position.y);
          }
          else
          {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            pstx.push_back(ref_x_prev);
            pstx.push_back(ref_x);
            psty.push_back(ref_y_prev);
            psty.push_back(ref_y);
          }

          // Add three waypoints in the distance
          vector<double> wp0 = getXY(autonomous_car.s+30, (2+4*autonomous_car.lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> wp1 = getXY(autonomous_car.s+60, (2+4*autonomous_car.lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> wp2 = getXY(autonomous_car.s+90, (2+4*autonomous_car.lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          pstx.push_back(wp0[0]);
          pstx.push_back(wp1[0]);
          pstx.push_back(wp2[0]);
          psty.push_back(wp0[1]);
          psty.push_back(wp1[1]);
          psty.push_back(wp2[1]);

          // Transform waypoints into AVs reference frame
          for (int i = 0; i < pstx.size(); i++)
          {
            double shift_x = pstx[i] - ref_x;
            double shift_y = psty[i] - ref_y;
            pstx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
            psty[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
          }
          
          // Add waypoints to spline
          tk::spline s;
          s.set_points(pstx, psty);

          // Start with the previous path
          for (int i = 0; i < prev_size; i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Define the target x,y position
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(pow(target_x, 2.0) + pow(target_y, 2.0));
          double x_add_on = 0;

          // Add new points to the path
          for (int i = 1; i <= 50 - prev_size; i++)
          {

            // Split spine into N points to ensure correct speed
            double N = (target_dist/(0.02*autonomous_car.target_vel/2.24));
            double x_point = x_add_on + target_x / N;
            double y_point = s(x_point);
            x_add_on = x_point;
            double x_ref = x_point;
            double y_ref = y_point;

            // Transform the points back to the world reference frame
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
            x_point += ref_x;
            y_point += ref_y;

            // Add the new points to the vector
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          // Websocket communitcation
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          auto msg = "42[\"control\","+ msgJson.dump()+"]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } 
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });
  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}