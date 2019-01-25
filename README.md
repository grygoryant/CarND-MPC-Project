# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Description

### 1. Model

The state vector contains system variables and errors:

![equation](http://latex.codecogs.com/gif.latex?%5Bx%2Cy%2C%5Cpsi%2Cv%2Ccte%2Ce%5Cpsi%5D),

where `x` and `y` are the vehicle position, ![equation](http://latex.codecogs.com/gif.latex?%5Cpsi) is orientation, `v` is speed. `cte` and ![equation](http://latex.codecogs.com/gif.latex?e%5Cpsi) are crosstrack error and the orientation error.

Update equations are as follows:

![equation](http://latex.codecogs.com/gif.latex?x_%7Bt+1%7D%20%3D%20x_t%20+%20v_t%20*%20cos%28%5Cpsi_t%29%20*%20dt)

![equation](http://latex.codecogs.com/gif.latex?y_%7Bt+1%7D%20%3D%20y_t%20+%20v_t%20*%20sin%28%5Cpsi_t%29%20*%20dt)

![equation](http://latex.codecogs.com/gif.latex?%5Cpsi_%7Bt+1%7D%20%3D%20%5Cpsi_t%20+%20%5Cfrac%7Bv_t%7D%7BL_f%7D%20*%20%5Cdelta_t%20*%20dt)

![equation](http://latex.codecogs.com/gif.latex?v_%7Bt+1%7D%20%3D%20v_t%20+%20a_t%20*%20dt)

![equation](http://latex.codecogs.com/gif.latex?cte_%7Bt+1%7D%20%3D%20f%28x_t%29%20-%20y_t%20+%20v_t%20*%20sin%28e%5Cpsi_t%29%20*%20dt)

![equation](http://latex.codecogs.com/gif.latex?e%5Cpsi_%7Bt+1%7D%20%3D%20%5Cpsi_t%20-%20%5Cpsi%20dest%20+%20%5Cfrac%7Bv_f%7D%7BL_f%7D%20*%20%5Cdelta_t%20*%20dt)

Actuators are:

![equation](http://latex.codecogs.com/gif.latex?%5Cdelta%20%5Cin%20%5B-25%5E%7B%5Ccirc%7D%3B%2025%5E%7B%5Ccirc%7D%5D) - steering angle

![equation](http://latex.codecogs.com/gif.latex?%5Cdelta%20%5Cin%20%5B-1%3B%201%5D) - throttle

### 2. `N` and `dt`

These values were choosen by trial and error, and they are 10 and 0.1 respectively. Values of `N` more than 10 are useless and computationally expensive for this project. `dt` value is connected with system latency, so 0.1 is reasonable value for this parameter.

### 3. Waypoints preprocessing

Waypoints are given by simulator as points in global coordinate system, so waypoints preprocessing is required. Given waypoints are shifted to the origin of car's coordinate system and then rotated by 90 degrees in order to follow car's orientation.

### 4. Latency

The simpliest way to consider system latency is just to predict the following state after the time frame which is equal to latency.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Build with Docker-Compose
The docker-compose can run the project into a container
and exposes the port required by the simulator to run.

1. Clone this repo.
2. Build image: `docker-compose build`
3. Run Container: `docker-compose up`
4. On code changes repeat steps 2 and 3.

## Tips

1. The MPC is recommended to be tested on examples to see if implementation behaves as desired. One possible example
is the vehicle offset of a straight line (reference). If the MPC implementation is correct, it tracks the reference line after some timesteps(not too many).
2. The `lake_track_waypoints.csv` file has waypoints of the lake track. This could fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We have kept editor configuration files out of this repo to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. We omitted IDE profiles to ensure
students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. Most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio and develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
