# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

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

## Implementation (Rubric Points)

### The Model

The model used is a Kinematic model neglecting the complex interactions between the tires and the road. The model equations are as follow:
![Model Equations](images/eqns.png)
```
x[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
y[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
psi[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
v[t] = v[t-1] + a[t-1] * dt
cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
```

Where:

- `x, y` : Car's position.
- `psi` : Car's heading direction.
- `v` : Car's velocity.
- `cte` : Cross-track error.
- `epsi` : Orientation error.

Actuator outputs are acceleration and delta (steering angle). The model combines the state and actuations from the previous timestep to calculate the state for the current timestep based on the equations above.

- `a` : Car's acceleration (throttle).
- `delta` : Steering angle.
### Timestep Length and Elapsed Duration (N & dt)
The number of points(`N`) and the time interval(`dt`) define the prediction horizon. The number of points impacts the controller performance as well. The values chosen for N and dt are 10 and 0.1, respectively. I chose these values after trying out different values for N in the range of 10 to 100, larger values of N made the model slow and also sometimes act erratically. While choosing the value for 'dt' I went with 0.1 after trying different values like 0.2, 0.25, 0.15, etc, as adjusting these values by even a small value made the system to behave erratically.

### Polynomial Fitting and MPC Preprocessing
The waypoints are preprocessed by transforming them to the vehicle's perspective (see main.cpp lines 108-113). This simplifies the process to fit a polynomial to the waypoints because the vehicle's x and y coordinates are now shifted to the origin (0, 0) and also making the orientation angle zero.

### Model Predictive Control with Latency
To acount for the latency the model equation were altered, as the original equations depended upon the actuations from the previous timestep but with a delay of 100ms (which happens to be the timestep interval) and the actuations are applied another timestep later.(See MPC.cpp lines 109-112).
An additional cost penalizing the combination of velocity and delta was included and results in much more controlled cornering.
