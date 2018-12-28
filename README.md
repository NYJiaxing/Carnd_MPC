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

## Work Flow

The vehicle model is a bicycle model, the control fectors including

* Vehicle's Position in X direction: x
* Vehicle's Position in Y direction: y
* Vehicle's orientation: psi
* Vehicle's velocity: v
* cross-track error: cte
* orientation error: epsi

The input will be:

* steering angle: alpha
* acceleration: A

The moving model for the vehicle is:

* x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
* y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
* psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
* v_[t] = v[t-1] + a[t-1] * dt
* cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
* epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt

## N (timestep length) and dt (elapsed duration between timesteps) values
The T refers to the duration over future predictions will be made. The T is the product of two other variables, N and dt.
The N and dt is the hyperparameters I can choose to tune the model. 

I start test the N = 20 and dt = 0.01, and several other combinations, N = 10, dt = 0.01, N = 20, dt = 0.1, N = 10, dt = 0.1 etc. The higher N value I choose, the higher offset the reference trajectory the vehicle will go, which means the vehicle will oscillate a lot just like high Kp value was signed in PID control project. So after several test, I set the N = 10 and dt = 0.1

## Polynomial Fitting and MPC Preprocessing

The control point will transform to vehicle's coordinate system first, the first control point will be the origin. Then, the orientation will be also set to 0 so the vehcile facing front. Vehicle will rotate based on the psi vaule. The final step is express the control point using Eigen vector, then send these points to polyfit function to fit the third order polynomial, which will be used to calculate the CTE.
