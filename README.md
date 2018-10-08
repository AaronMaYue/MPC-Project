# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## MPC
Model Predicr Cotrol uses an optimizer to find the control input and minimize the cost function. Here are the state vector $ state = \left[ x, y, \psi, v, cte, e\psi \right]$
### Duration
Set up everything required for the MPC loop. This consist of defining the duration of the trajectory, $ T = N * dt $, Here I set N = 10 and dt = 0.1. the duration T = 1s.
### Cost Function
`void operator()(ADvector& fg, const ADvector& vars)`

`fg` is the cost function and vehicle model/constrain is defined.
`vars` contains all variables used by the cost functin and model.
This si all one lont vector, I set N = 10, so the indices are show as follows:
$$ vars[0],\dots, vars[9] -> \left[ x_1,\dots,x_{10}\right] $$
$$ vars[10],\dots, vars[19] -> \left[ y_1,\dots,y_{10}\right] $$
$$ vars[20],\dots, vars[29] -> \left[ \psi_1,\dots,\psi_{10}\right] $$
$$ vars[30],\dots, vars[19] -> \left[ v_1,\dots,v_{10}\right] $$
$$ vars[40],\dots, vars[49] -> \left[ cte_1,\dots,cte_{10}\right] $$
$$ vars[50],\dots, vars[59] -> \left[ e\psi_1,\dots,e\psi_{10}\right] $$
$$ vars[60],\dots, vars[69] -> \left[ \delta_1,\dots,\delta_{9}\right] $$
$$ vars[70],\dots, vars[79] -> \left[ a_1,\dots,a_{9}\right] $$

fg[0] store the cost value, I sum all the components of the cost and store them at fg[0].
$$ J = \sum_{t=1}^{N}(cte_t - cte_{ref})^2 + (e\psi_t - e\psi_{ref})^2 + \dots $$

```
for (int i = 0; i < N; ++i)
    {
      fg[0] += 3500 * CppAD::pow(vars[cte_start + i], 2);
      fg[0] += 3500 * CppAD::pow(vars[epsi_start + i], 2);
      fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
    }

    for (int i = 0; i < N - 1; ++i)
    {
      fg[0] += CppAD::pow(vars[delta_start + i], 2);
      fg[0] += CppAD::pow(vars[a_start + i], 2);
      // penalty weight for steering and speed
      fg[0] += 500 * CppAD::pow(vars[delta_start + i] * vars[v_start + i], 2);
    }

    for (int i = 0; i < N - 2; ++i)
    {
      fg[0] += 200 * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
      fg[0] += CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
    }

```

Define the vehicle model and constraints
$$ x_{t + 1} = x_t + v_t * \cos(\psi_t) * dt$$
$$ y_{t+1} = y_t + x_t * \sin(psi_t) * dt $$
$$ \psi_{t+1} = \psi_t  - \frac{v_t}{L_f} * \delta_f * dt  $$
$$ v_{t+1} = v_t a_t * dt $$
$$ cte_{t+1} = f(x_t) - y_t + v_t *\sin(e\psi_t)*dt $$
$$ e\psi_t = \psi_t - \psi_{des_t} - \frac{v_t}{L_f}*\delta_t*dt $$

$$\delta \in \left[ -25^\circ, 25^\circ \right]$$
$$ a \in \left[ -1, 1\right] $$ 

Note that the $\delta$ is positive we rotate counter-clockwise, or turn left. In the simulator however, a positive value implies a right turn and a negative value implies a left turn. 

### Latency
In a real car, an actuation command won't execute instantly, so there will be a delay as the command propagtes throuth the system. a relaistic delay might be ont he order of 100ms.
dt = 0.1(100ms), so I set previous $\delta$(steering value) and $\alpha$(throttle value) state to the constraints instead of delay.

```
  previous_delta = steering_value
  previous_a = throttle_value
  //take previous state for delaing 100ms.
  vars_lowerbound[delta_start + 1] = previous_delta;
  vars_upperbound[delta_start + 1] = previous_delta;
  vars_lowerbound[a_start + 1] = previous_a;
  vars_upperbound[a_start + 1] = previous_a;
```




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

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
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
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
