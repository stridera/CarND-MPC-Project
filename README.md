# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## Project Writeup

### The Model

The project has a vehicle that provides vehicle states and lets you modify two actuators.  The vehicle state we are given is the vehicles location (x and y) as well as the direction it's facing (psi).  Finally we're given the speed it's moving (v).  The actuators we have access to control the vehicle are the steering angle and the throttle (which includes going in reverse if required.)

We take the simple model of stepping through each of them, and calculating all the points where the car would be in the future assuming the actuators don't change.  For example, our code looks like this (vars renamed for readability):

```
x+n = next_x - (current_x + current_v * cos(current_psi) * time_step);
y+n = next_y - (current_y + current_v * sin(current_psi) * time_step);
psi+n = next_psi - (current_psi - current_v * current_actuator_psi / size_of_car * time_step);
v+n = next_v - (current_v + current_actuator_v * time_step);
```

You can see that I basically step through each timestep length and calculate where each point will be for each step up to the max.  This is run through a for loop that incrementally starts on the next waypoint.  So, the first time you get a line starting from x0.  Next time you bend the line from x1, then x2, and so on until you reach N.  This will give you the current route we plan to take that we solve against to find the best match to stay as close to the CTE as possible.

### Timestep Length and Elapsed Duration (N & dt)
N and dt are both variables that determine how far out we take our planned route.  dt is the time step we have between predicted path points, and N is the number of points we step out.  I selected 10 and 0.1 as my values.  I honestly selected 0.1 as my timestamp because it matched my latency and had a nice synergy.  I selected 10 steps in advance because it looked nice on in the simulator and appeared to work well.  (Not to mention them being pretty close to the recommended values given in the class.)  Having too high of a value would be useless since we would be mapping too far in advance and the future we guess would almost certainly change.  Making the values too small would in turn give us too narrow of a view of the future and make it so we can't plan actions for curves quick enough to have the car react.

### Polynomial Fitting and MPC Preprocessing

To handle the waypoints seen by the sensors, we first  had to take the current points found and convert them into vehicle coordinates.  We were then able to take these transformed points and run them through the polyfit, polyeval, and simple trig functions to get the current state that we were able to pass to the mpc solver.

### Model Predictive Control with Latency

To account for latency, I simply took where the vehicle currently was and added 0.1 sec.  While this is kinda cheating (since we know for a fact that our simulator is delayed by that amount), it let me supply points of where the vehicle would be given the current conditions.  This meant that any solutions provided were actually for a tenth of a second ahead of where the car actually was, and thus, would react exactly as expected once the latency was overcome.

We also add values to the cost function to make the system more likely to handle some actions than others to make the system smoother.  These values required some tuning, but once a good set of numbers were found, the car drove wonderfully.  When these numbers were too small, for example, the car would constantly overcorrect and would whip left and right until it was flung off the road.  If they were too large, it wouldn't respond fast enough and would just drive off the road on any curve.  I also used ref_v (preferred speed) to help adjust these, and attempted to get to a speed of 200, but things would always get a little iffy above 100.  Still, I think running the track at 100mph is a great result.

---

## Installation Dependencies

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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.