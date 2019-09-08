# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Requrirement to the path planning

- The car is able to drive at least 4.32 miles without incident..

   - The top right screen of the simulator shows the current/best miles driven without incident. Incidents include exceeding acceleration/jerk/speed, collision, and driving outside of the lanes. Each incident case is also listed below in more detail.

- The car drives according to the speed limit.
	
   - The car doesn't drive faster than the speed limit. Also the car isn't driving much slower than speed limit unless obstructed by traffic.

-Max Acceleration and Jerk are not Exceeded.

   - The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.

- Car does not have collisions.
	
   - The car must not come into contact with any of the other cars on the road.

- The car stays in its lane, except for the time between changing lanes.
	

   - The car doesn't spend more than a 3 second length out side the lane lanes during changing lanes, and every other time the car stays inside one of the 3 lanes on the right hand side of the road.

- The car is able to change lanes

   - The car is able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic.


## Explanation of the code

In this section, I explain following parts of the code.
1. Finite state machine
2. Crusing & following a car on the ego lane
3. Lane change algorithm
4. Tragectory generation

### 1. Finite state machine
The state machie has following four state.

0. Inital state

	In this project ego car starts with stopping. To avoid lane changing with low speed, the machine remains this state until the velocity of ego vehicle exceed 40mph.
	
1. Lane keep

	In this state, the car keep current lane.
	
2. Prepare to change lane

	If the distance to following car is lower than 60m in s-coordinates, the car's state change to this state. In this state, the car prepare to change lane. Ditails are explained in section 3 "Lane change algorithm"

3. Lane change

	When the car decide to execute the lane change, the car's state change to this state. After lane change is completed, the state change to "Lane keep state". I set this state to avoid executing another lane change while changing lane.
	
```c++

// state
int state = 0;
// 0: start, accelarating
// 1: Lane Keeping
// 2: Lane Change Preparating
// 3: Lane Changing

```

### 2. Crusing & following a car on the ego lane

While crusing (there's no car ahead) the car should keep the maximum speed on the road and not exeed it. If there's a car in front of the ego vehicle, it should follow the car and not hit it.

```c++

double ref_vel = 0; //mph, target velocity
double max_acc = 5; //mph, max accelaration
bool too_close = false;
if((follow_dist_ego) < 30.0){ // if the distance to following car is lower than 30m
	too_close = true; // change flag to lowering velocity
}
if (too_close){
	// lowering velocity within max accelaration
	ref_vel -= max_acc * 2.237 * 0.02;
}else if (ref_vel < 49.5){
	// keep lane and acceralate to ref_vel within max accelaration
	ref_vel += max_acc * 2.237 * 0.02;
}

```
### 3. Lane change algorithm

- Lane change preparation

	If the distance to the following vehicle is lower than 60m, the state changes and the car prepare to lane change.


	```c++

	bool lc_prep = false; // flag to lane change preparation
	if((follow_dist_ego) < 60.0){
		    lc_prep = true;
		  }

	```

- Checking safty of changing lane.

	To chack the safety of changing lane, I use TTC (time to collision) and the distante to the car around the vehicle.
	
	```c++
	
	// check the car is in the left lane
            if (d<(2+4*(lane-1)+2) && d>(2+4*(lane-1)-2)){
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_car_s = sensor_fusion[i][5];
              
              check_car_s += ((double)prev_size * 0.02 * check_speed);
              
              // find a gap to a following vehicle
              double gap = check_car_s - car_s;
              
	      ...

              // check if there's enough space to lane change
              double ttc = gap / (check_speed - car_speed_mps);
              bool ttc_safe =  ( ttc > 2.5) || (ttc < -2.5);
              left_safe &= ttc_safe;
              
              bool dist_safe = (gap > 10) || (gap < -10);
              left_safe &= dist_safe;
            }
	
	```
	
	I set minimum TTC as 2.5 sec, since it takes around 2 sec to change lane.
	We should care the distance to side collision, even if the TTC is enough high (for example the car in next lane cruise exact same velocity as ego vehicle.) I sate the minimum distance to 10m.
	I tuned these parameters in the simulator and those values are enough to change lane safely and effectively.
	
	

- Choosing lane (cost function)
	
	While preparating lane change, the car compare the distance to following cars for each lane. This means that the cost function is distance to the following car.
	
	The distance is calcurated in each time using sensor fusion result. If there is no car on the lane, the value is set as high value. for example...
	
	```c++
	
	 // distance to following car
          double follow_dist_ego = 999; // on ego lane
          double follow_dist_l = 900; // on left lane
          double follow_dist_r = 900; // on right lane
          double follow_dist_ll = 900; // on left lane
          double follow_dist_rr = 900; // on right lane
          
          double car_speed_mps = car_speed / 2.24;
          
          for (int i=0; i<sensor_fusion.size(); i++){
            float d = sensor_fusion[i][6];
            
            // check the car is in the same lane
            if (d<(2+4*lane+2) && d>(2+4*lane-2)){
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_car_s = sensor_fusion[i][5];
              
              check_car_s += ((double)prev_size * 0.02 * check_speed);

              // find a gap to a following vehicle
              double gap = check_car_s - car_s;
              if ((gap > 0)&&(gap < follow_dist_ego)){
                follow_dist_ego = gap;
              }
            }
	
	```
	
	I set the initial value for the ego lane higer than others, since the car don't have to change lane if there's no car in the front.
	
	The car changes the lane which has the longest distance to the following car. If the target lane is safe, the state changes and the car execute lane change.
	
	```c++
	
	...
	}else if (state==2){       
            
            if(right_safe && (follow_dist_r > follow_dist_ego + 10) && (follow_dist_r > follow_dist_l)){
              lane += 1;
              state = 3;
              lc_prep = false;
            }else if(left_safe && (follow_dist_l > follow_dist_ego + 10)){
              lane -= 1;
              state = 3;
              lc_prep = false;
            }  
	  ...
	
	```
	
	Here I add 10m to "follow_dist_ego" to add hysteresis. This will help the car to repeat lane changing frequently.
	
	When vehicle is in left-end or right end-lane, the car also check the free space of next next lane. Without this function the car sometimes stuck when the following cars in ego lane and next lane are both slow.


- Lane change execution

	To avoid start another lane change while changing lane, the state remains "Lane changing" until the car position is center of the target lane. After it's dane, the state change to "lane keep".
	
	```c++
	
	...
	 }else if(state==3){
	    bool lc_done;
	    lc_done = (car_d<(2+4*lane+0.5) && car_d>(2+4*lane-0.5));

            ...
	    
	    if(lc_done){
	      state = 1;
	    }
	  }
	
	```
	
### 4. Tragectory generation

I use a spline to create smooth tragecory. The waypoints are calcurated in 30m-interval and a spline is fitted to them. 

If the interval is too high, the lane changing is going to be too slow. If it's too low, the lane change is going to be too sudden and violate the jerk limitation. I test with the simulater and 30m is enough to meet the requirement of the project.

```c++

#include "spline.h"

...

vector<double> next_wp0 = getXY(car_s+30, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(car_s+60, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(car_s+90, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

ptsx.push_back(next_wp0[0]);
ptsx.push_back(next_wp1[0]);
ptsx.push_back(next_wp2[0]);

ptsy.push_back(next_wp0[1]);
ptsy.push_back(next_wp1[1]);
ptsy.push_back(next_wp2[1]);

...

// create a spline
tk::spline s;
s.set_points(ptsx, ptsy);

...

```

## Reflection & Future work

I tested the algorithm with the simulator and it drives 11 miles withought incident in average. (I tested three times and it drives 9 miles, 15 miles and 10 miles). This meet the requirements of the project. 

Followings are the list of the possible future work.

- Prediction
	- Sometimes the car can't deal with a sudden cut-in and hit. Prediction algorithm will help the car to detect potential risk and brake in advance.
- Improving tragectory generation
	- Sometimes the car slightly out of the lane especially in a curve. This is because the car follow the exact spline. Polynomial fit or some cost function that penalize according to the distance frome the center of the lane.
