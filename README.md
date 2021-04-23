# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image Reference)

[image1]: ./screengrab_run.png "One Lap"

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.]

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Reflection

I create a list of (x,y) waypoints, evenly spaced at 30m, and interpolate them with a spline function to fill it in with more points to control the car's speed. First, I need to initialize our car's starting point. It will be either the car's current position or the previous path's end points. Our planner is fed with previous path data from the simulator. If the previous path is almost empty, I use the car's current location as the starting reference. Otherwise, I redefine the reference states to be the previous path end points. The related implementation code is shown below.

```c++
   // if the previous size is almost empty, use the car as starting reference
      if (prev_size < 2)
      {
        double prev_car_x = car_x - cos(car_yaw);
        double prev_car_y = car_y - sin(car_yaw);

        // use two points that make the path tangent to the car
        ptsx.push_back(prev_car_x);
        ptsx.push_back(car_x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(car_y);
      }
      // use the previous path's end points as reference
      else
      {
        // redefine reference state as previous path end points
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];

        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];

        ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
      }
```

The vectors `ptsx` and `ptsy` now contain 2 points each. Next, we add 3 more points to these vectors spaced at 30m, 60m and 90m ahead of our car with a total of 5 points in the vectors `ptsx` and `ptsy`.

```c++
   // getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
      vector<double> next_wp0 = getXY(car_s+30, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
      vector<double> next_wp1 = getXY(car_s+60, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
      vector<double> next_wp2 = getXY(car_s+90, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

      ptsx.push_back(next_wp0[0]);
      ptsx.push_back(next_wp1[0]);
      ptsx.push_back(next_wp2[0]);

      ptsy.push_back(next_wp0[1]);
      ptsy.push_back(next_wp1[1]);
      ptsy.push_back(next_wp2[1]);
```

Next, I shit car's reference to origin of global coordinates and its angle to 0 degrees. This help with the math of creating a path as well as helps avoid multiple `y` points for the same `x`.

```c++
      for (int i=0; i<ptsx.size(); ++i)
      {
        // shift car's reference to origin and the angle to 0 degrees
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
        ptsy[i] = shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw);
      }
```

Then I create a spline to interpolate between waypoints.

```c++
      tk::spline s;
      // set (x,y) points to the spline
      s.set_points(ptsx, ptsy);
```

I need to calculate how to break up spline points so that we can travel at the desired reference velocity. We set a target of 30m in the x direction ahead of our car and calculate the distance to target. `N` is the number of portions this distance is divided into. 

```c++
      double target_x = 30.0;
      double target_y = s(target_x);
      double target_dist = sqrt(target_x*target_x + target_y*target_y);

      double x_add_on = 0.0;

      // fill up rest of the planner now that it is filled up with the previous points (if any)
      for (int i=0; i<50-previous_path_x.size(); ++i)
      {
        double N = target_dist/(0.02*ref_vel/2.24); // divide by 2.24 to convert from mph to m/s
        double x_point = x_add_on + target_x/N;
        double y_point = s(x_point);
```

Last step would be to transform the car's reference back to normal after shifting it earlier and pushing the points to `next_x_vals` and `next_y_vals`.

```c++
        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // shift car's reference back to normal after shifting it earlier
        x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
        y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

        x_point += ref_x;
        y_point += ref_y;
        
        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
```

A screengrab of my car doing a run for more than 4.32 miles without any incident.

![alt text][image1]