# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## Model Predictive Control in a Simulated Driving Environment

The goal of this project was to smoothly navigate a simulated course using feedback (cte and epsi) from sensors. Using the MPC and vehicle model from the Udacity course, the sensor feedback was modeled and returned to the simulator as a series of steering angles and throttle inputs. The overall goal was to integrate sensor readings to provide a safe and managed track through the simulator.

### Model

The model used in this project assumes four (4) states [X,Y,p,V] where X,Y are the coordinates of the vehicle, p represents the heading and V the velocity. Additionally we have two inputs, steering and accelerator. This 6 state model is referenced as the Global Kinematic Model.

This model ignores tire forces, gravity and mass. It is a simple kinematic model but for the purposes of testing the algorithm the other effects can be ignored (and the simulator doesn't model them anyways)

### Time estimation

Two parameters that were modeled were the N (number of time steps to estimate ahead) and dT (delta between the steps). So related as N*dT gives us the amount of time (T) we are looking ahead.

We found that looking ahead anymore than about 1sec produced unstable results especially around areas where there are frequent corners. Shorter time frames didn't provide the speed to respond to changes in the road fast enough. Increasing N allowed for a higher resolution of the points but tended to overfit the polynomial especially at slower speeds causing unstable inputs to the actuators (ie. crashing). Final values were N = 10 and dT = 0.1

### Control and latency

The predictive control module attempts to minimize a cost function in order to provide the navigation path. The cost function is calculated as the sum of;

1. Minimizing the CTE and heading error (cte,epsi,v)
2. Add a factor to ensure the speed is kept near the limit (steer, throttle)
3. Add some delta time parameters to minimize any oscillations from the feedback (delta,accel)

The following values were used;

```c++
int cte_weight = 600;
int epsi_weight = 600;
int v_weight = 1;
int steer_weight = 2;
int throttle_weight = 50;
int delta_weight = 100;
int accel_weight = 2;
```

The parameters were manually tuned with the goal to ensure that the safest and smoothest path was taken through the model. It likely exceeded the speed limit, but faster speeds are more likely to see errors occur so testing was done around 60-70mph.

Also, we inject a 100ms delay into the feedback to simulate signal latency with the actuators. While I'm not certain that the 100ms should be added into the model as the latency may not always be constant (investigating this), results from estimating out the state dT seconds showed very good results. The values were estimated out using the previous steering angle, throttle, and an estimate for the time delay.

```c++
double delay_x = v*delay_t;
double delay_y = 0;
double delay_psi = -v*steer_value / Lf * delay_t;
double delay_v = v + throttle_value*delay_t;
double delay_cte = cte + v*sin(epsi)*delay_t;
double delay_epsi = epsi-v*steer_value /Lf * delay_t;
```



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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
  * If you have MacOS and have [Homebrew](https://brew.sh/) installed you can just run the ./install-mac.sh script to install this.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/CarND-MPC-Project/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
