# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program


## Goals
Goal of this projectis to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.


## Rubic Points
### Compilation
* The code compiles correctly.
No changes were made in the cmake configuration. A new file was added src/spline.h. It is the cubic spline interpolation implementation, which is suggested in the classroom QA video. Also, "cmath" library is included to use exponential: exp().

### Valid Trajectories
* The car is able to drive at least 4.32 miles without incident..
The car could have droven 10 miles without incident.

* The car drives according to the speed limit.
No speed limit red message was seen.

* Max Acceleration and Jerk are not Exceeded.
No Acceleration/Jerk red message was seen.

* Car does not have collisions.
No collisions occured while driving.

* The car stays in its lane, except for the time between changing lanes.
The car stays in its lane most of the time but when a slow moving car in front is blocking the traffic.

* The car is able to change lanes
The car change lanes when the there is a slow car in front and it is safe to change lanes (no other cars around).


### Reflection
* There is a reflection on how to generate paths.

#### Prediction (Line 109 - 157 )
- Is there any car within 30m in front blocking the traffic ?
- Is there any car in right and/or left lane within 20m around, which disables us to change lane safely ?

#### Behavior (Line 159 - 267 )
- By using switch statement, I created 7 stetus as follows.
status #1) driving in mid lane
When a car in front is too close, check if lane change is possible or not.
If both lane change is possible, the clear distance and minimum velocity is considered to decide priority.
If both lane chage is not possible, then reduce the speed gradually.
After keep reducing speed more than 10 seconds, above mentioned checking procedure is done even when a car is not too close.

status #2) change lane from mid to right lane
Lane change is completed when d-value becomes +/- 0.2 of center of destination lane.

status #3) driving in right lane
When a car in front is too close, check if lane change is possible or not.
If lane chage left is not possible, then reduce the speed gradually.
After keep reducing speed more than 10 seconds, above mentioned checking procedure is done even when a car is not too close.

status #4) change lane from right to mid lane
Lane change is completed when d-value becomes +/- 0.2 of center of destination lane.

status #5) change lane from mid to left lane
Lane change is completed when d-value becomes +/- 0.2 of center of destination lane.

status #6) driving in left lane
When a car in front is too close, check if lane change is possible or not.
If lane chage right is not possible, then reduce the speed gradually.
After keep reducing speed more than 10 seconds, above mentioned checking procedure is done even when a car is not too close.

status #7) change lane from left to mid lane
Lane change is completed when d-value becomes +/- 0.2 of center of destination lane.




#### Trajectory (Line 279 - 367 )
This code does the calculation of the trajectory based on the speed and lane output from the behavior, car coordinates and past path points.

First, the last two points of the previous trajectory are used in conjunction three points at a far distance to initialize the spline calculation.
To make the work less complicated to the spline calculation based on those points, the coordinates are transformed to local car coordinates.


---
## Dependencies

* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)

