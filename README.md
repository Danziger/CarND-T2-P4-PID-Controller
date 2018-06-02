CarND · T2 · P4 · Proportional-Integral-Derivative (PID) Controller Project
===========================================================================

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

<img src="output/images/004 - Simulator Rotated.png" alt="Particles Filter visualization on the simulator." />


Project Overview
----------------

In this project...

TODO

To test it, [Term 2 Simulator](https://github.com/udacity/self-driving-car-sim/releases) need to be used. The latest version of `main.cpp` used to run this project without the simulator can be found [here](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project/blob/06cbc9967bc62592723eef99b8c8035e4a22ea7b/src/main.cpp).

If you are looking for Udacity's started code project, you can find it [here](https://github.com/udacity/CarND-PID-Control-Project).


Dependencies
------------

- [Udacity's Self Driving Car Simulator](https://github.com/udacity/self-driving-car-sim/releases)
- [`cmake >= 3.5`](https://cmake.org/install/)
- `make >= 4.1` (Linux / [Mac](https://developer.apple.com/xcode/features/)), [`3.81` (Windows)](http://gnuwin32.sourceforge.net/packages/make.htm)
- `gcc/g++ >= 5.4` (Linux / [Mac](https://developer.apple.com/xcode/features/)), [`MinGW` (Windows)](http://www.mingw.org/)
- [`uWebSockets` commit `e94b6e1`](https://github.com/uWebSockets/uWebSockets). See the following section for installation instructions and additional details.


Installation
------------

This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets):

- `install-mac.sh` for Mac.
- `install-ubuntu`for either Linux or [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) (**please, make sure it is updated**).

For Windows, Docker or VMware coulso also be used as explained in the [course lectures](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77). Details about enviroment setup can also be found there.

If you install from source, checkout to commit `e94b6e1`, as some function signatures have changed in `v0.14.x`:

    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1

See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.


Build & Run
-----------

Once the install is complete, the main program can be built and run by doing the following from the project top directory:

1. Create a build directory and navigate to it: `mkdir build && cd build`
2. Compile the project: `cmake .. && make`
3. Run it: `./PID`

Or, all together (from inside the `build` directory): `clear && cmake .. && make && ./PID`

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d).


Reflection
----------

### P, I & D Coefficients & Implementation Details

First, let's see what each term (P, I and D) and the coefficients we can adjust to modify their contribution to the total error (Kp, Ki and Kd) do:

- **`P (Kp)`**: This term produces an output that is proportional to the current error. The larger this term, the larger the output, given a change in the error.
  If this term is too small, the controller would not be too responsive. In our case, this means the vehicle won't steer enough on turns and will probably go out of the track.
  If this term is too big, however, the controller would likely overshoot the setpoint (target) value and might become unstable. In our case, this means the vehicle will steer towards the center of the road, overshoot, turn the opposite way, overshoot again... Oscillating more and more each time until it goes out of the track or crashes into something.
  When adjusted properly, the vehicle would oscillate around the center of the road, but the oscillations won't be wide enough to make it crash or go out of the track.
  
- **`I (Ki)`**: This term produces an output that is proportional to the sum of the errors at each timestep. The longest an error is sustained over time, the larger this term becomes.
  If this term is too small, we might always have a steady-state error. In our case, the vehicle might always be offset a constant distance from the middle of the road due to errors on the steering system calibration or, when turning fast, the car might drift outside of the track, as the `P` term would not account for the extra steering that should be applied to counter that.
  If this term is too big, however, the controller would likely overshoot the setpoint. In our case, the vehicle might sharply increase the steering angle while going through a turn at a small CTE, which would probably cause an overshoot and instability, which might end up with the vehicle crashing or going out of the track.
  When adjusted properly, the vehicle will be able to stay closer to the center of the road while turning and will react quicker to sudden turns.
  
- **`D (Kd)`**: This term produces an output that is proportional to the slope of the error over time. The faster the error is reduced, the larger this term becomes.
  If this term is too small, the controller would probably be less stable. In our case, the vehicle will produce bigger oscillations.
  If this term is too big, the controller would probably undershoot the setpoint. In our case, as the vehicle starts steering towards the center of the road and the error starts decreasing, this term will make the vehicle steer in the opposite direction, increasing the CTE again, followed by another attempt to move towards the center of the road... as if there was some kind of magnetic repulsion between the vehicle and the center of the road.
  When adjusted properly, the vehicle will slightly counter-steer as it approaches the center of the road to avoid overshooting it, reducing the oscillations as well.

A few modifications have been introduced to this basic implementation of a PID controller, producing the following code:

    // Calculate the change in CTE and the accumulated CTE (weighted):
    const double pv_diff = pv - pv_prev_;

    // Update the previous CTE to calculate the diff in the next timestep:
    pv_prev_ = pv;

    pv_int_ = pv + Wi_ * pv_int_;

    // Calculate the contribution of each component to the total erro:
    err_p_ = (-Kp_ - Ap_ * alpha) * pv;
    err_i_ = (-Ki_ - Ai_ * alpha) * pv_int_;
    err_d_ = (-Kd_ - Ad_ * alpha) * pv_diff;

    // Calculate the total error:
    err_total_ = err_p_ + err_i_ + err_d_;
    
First, we can see how the integral term is not a sum, but a weighted sum: `pv_int_ = pv + Wi_ * pv_int_`. That means that this term would vanish over time before even the car crosses to the opposite side of the road, where it would have a CTE with opposite sign that will eventually bring the integral term back to 0.

Then, each coefficient is increased/decreased by a variable `alpha` multiplied by a second coefficient set (Ap, Ai and Ad). This way, instead of having a fixed set of coefficients that are likely to work better at some specific speed, we can have a base weight that is increased or decreased as the vehicle’s speed changes, which is a more intuitive approach.

For example, let's imagine a P-controller with a small Kp moving at a low speed. As the vehicle approaches a curve, it will start turning a bit, but as it's moving slowly, that small steering angle will be enough to make the car drive through the curve. However, if the car has the same Kp, but is traveling at a higher speed, chances are that small steering angle won't be enough to make the car turn enough to stay on the road and it will just go out straight or crash.

Note that when Ap, Ai and Ad are set to 0, the controller will behave exactly like a normal PID controller, as explained above.

Regarding he specific implementation used in the project, I have used one PID controller for the steering angle and an optional one can be used for the throttle. Alternatively, a fixed throttle can be used. The output of both controllers is constrained between a min and a max value, -1 and +1 in our case.

Moreover, for the steering controller, the final steering value is a weighted sum between the value outputted by the controller and the previous angle, which helps reduce noise and oscillations:

    double steer_value = 0.8 * pid_steer.update(CTE, 1.5 * speed, -1, 1) - 0.2 * angle/25;

Regarding the throttle controller, both the process variable and the setpoint are adjusted before being feed to the controller:

- The former (speed error) is constraint above 0, which allows the vehicle to accelerate sharply and reduce speed by only releasing the throttle. Alternatively, a small value > 0 could be set, which would allow the breaks to be actuated, but in a less intense way than the throttle.
- The latter (target speed) is linearly decreased in relation to the CTE from a maximum value of 100 to a minimum value of 30. That is, the more offset the vehicle is, the slower it must go.

    const double maxSpeed = 100;
    const double minSpeed = 30;
    const double targetSpeed = maxSpeed - min(abs(CTE), 2.0) * (maxSpeed - minSpeed) / 2;

    // This min(0, diff) makes the throttle response asymmetric: accelerate hard but break soft.
    // If the min is set to 0, the car will never break, it will just release the throttle.
    // Otherwise, if set to anything > 0, the car will break softer that it would without this.
    throttle = pid_throttle.update(min(0.0, speed - targetSpeed), CTE, -1, 1);


### Hyperparameters Tunning

Lastly, the tuning process was done manually by trial and error and by observing the effect of specific changes on the coefficients on the vehicle's behavior and on the contribution of each term to the total error (logged out to the terminal):

First, with Ki and Kd set to 0 and a fixed throttle that would make the car go at around 50, Kp was adjusted so that the vehicle would oscillate around the center of the road without crashing. Next, Kd was increased incrementally until the oscillations were reduced and the trajectory of the car around the track was much smoother. Lastly, Ki was increased incrementally until the vehicle was able to make turns with an smaller overall CTE.

Then, the second set of coefficients (Ap, Ai and Ad) was set to 1/100 of the value of the previous set of coefficients. That means that the sum of both coefficients at a speed of 100 would be twice the value of the K-coefficients alone. Next, the K-coefficients were reduced to compensate for the extra contribution of the A-coefficients and all of them were fine-tuned, observing how the vehicle was behaving on the track and what the contribution of each term was. For example:

- If the vehicle is overshooting, Kp (and maybe Ki) could be reduced.
- If the vehicle is overshooting while turning, Ki could be reduced.
- If the vehicle is too offset while turning, Ki could be increased.
- If the vehicle is undershooting or too unstable, Kd could be decreased.
- If oscillations need to be reduced, Kd could be increased.


Interesting Resources
---------------------

- [Self-Driving Car Project Q&A | PID Controller](https://www.youtube.com/watch?v=YamBuzDjrs8).


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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
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
