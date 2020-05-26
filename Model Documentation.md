# **Path Planning**

## Writeup

---

**Path Planning Project**

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.


## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/1971/view) individually and describe how I addressed each point in my implementation.

---
## Compilation

The code compiles. No changes to the CMakeLists.txt were made.

---
## Valid Trajectories

All the criteria are fullfilled most of the time. An incident can still occur, especually in the case when the car that the ego vehicle is following applies sudden strong braking. However, the frequency of incidents are one every ~10 miles or so.

---
## Reflection

### Processing of the sensor fusion data

The vehicles received from sensor fusion are assigned to the respective lanes based on their lateral Frenet coordinate. The vehicles outside the roadway where the ego vehicle is moving are ignored.

### Path planning logic

The output of this section are the desired lane and desired velocity. These are later used as a reference for trajectory generation. These parameters need to be tracked and updated, so they are defined as a global variables that are pased to the function by reference.

#### Choosing the right lane

The decision whether some lane is driveable or not is based on the following criteria:

##### Relative Longitudinal Frenet Displacement

If any vehicle finds itself in the specific area around the ego longitudinal Frenet position, the lane in which this vehicle is is considered undriveable.

##### Time-To-Colision (TTC)

If any vehicle is about to reach the longitudinal Frenet position of the ego vehicle within some given time, the lane in which this vehicle is is considered undriveable.

##### Lane Change

When ego lane becomes undriveable, the algorithm looks for the lane to change to. If more than one option is available, the car chooses the lane with the largest TTC of the first vehicle in front. When both lanes are empty the ego vehicle switches to the lane on the left.

##### Double Lane Change

If no driveable lanes for single lane change were found, the planner is considering the second left/right lane for the desired lane. This lane must fulfill the same criteria as lanes considered for single lane change in order to consider it driveable. Additionally, car should be able to transition through the lane in between. This lane also needs to fulfill the same criteria, but the Relative Longitudinal Frenet Displacement criterion is a bit looser, since the time spent in this lane is very short.

#### Choosing the right velocity

If ego lane becomes undriveable, and no lane is available for lane change, the velocity reference is adjusted so it matches the velocity of the first vehicle in front. The velocity is changed step-wise, so it doesn't violate the max acceleration criteria. If the ego comes too close to the vehicle in front, the velocity reference is set to the 3/4 of the velocity of that vehicle.

### Trajectory Generator

The whole logic used for trajectory generation is copied from the David's and Aaron's Q&A session. The only difference to their implementation is the generation of the three reference way-points in front. The distance of these points is adjusted to the vehicle velocity. If the vehicle is driving with the max velocity these points are further ahead. This allows us to do quick lane changes even if the velocity of the ego is small. The reference way-points together with the previous pathpoints are fitted to the spline. Spline is then sampled at the appropriate rate (20ms) to get the final trajectory.

### Problems

As mentioned before it could happen that the vehicle in front suddenly brakes strongly and causes the collision with ego, since the car can't react that fast. This can be solved by considering additional maneuvers like emergency braking, emergency steering or similar.

The vehicle is often not driving in the center of the lane, but often on the edges, which could be dangerous. This can be solved by refining the map way-points, or by doing the spline fitting of the points themselves.
