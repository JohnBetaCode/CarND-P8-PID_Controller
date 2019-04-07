<!-- 
**********************************************************************
https://review.udacity.com/#!/rubrics/1972/view

Project Specification
PID Controller

Compilation:
    (OK) - 1 - Your code should compile.:Code must compile without errors with cmake and make. Given that we've made CMakeLists.txt as general as possible, it's recommend that you do not change it unless you can guarantee that your changes will still compile on any platform.

Implementation:
    2 - The PID procedure follows what was taught in the lessons: It's encouraged to be creative, particularly around hyperparameter tuning/optimization. However, the base algorithm should follow what's presented in the lessons.

Reflection:
    3 - Describe the effect each of the P, I, D components had in your implementation: Student describes the effect of the P, I, D component of the PID algorithm in their implementation. Is it what you expected? Visual aids are encouraged, i.e. record of a small video of the car in the simulator and describe what each component is set to.

    4 - Describe how the final hyperparameters were chosen: Student discusses how they chose the final hyperparameters (P, I, D coefficients). This could be have been done through manual tuning, twiddle, SGD, or something else, or a combination!

Simulation: 
    5 - The vehicle must successfully drive a lap around the track: No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle).

# https://github.com/jeremy-shannon/CarND-PID-Control-Project
# https://github.com/udacity/CarND-PID-Control-Project
# 

**********************************************************************
-->

# P8-CarND-PID_Controller
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

<img src="writeup_files/banner.png" alt="drawing" width="700"/> 

## Overview

In this project, I used what I've learned about PID module. I Coded in C+a Proportional-Integral-Derivative Controller, or PID for short, in order to drive a simulated car around a virtual track. The project involves implementing the controller for car's steering angle but is used also to determine the car's throttle value.

---
## Description

Describe the effect each of the P, I, D components had in your implementation.

The P, or "proportional", component had the most directly observable effect on the car's behavior. It causes the car to steer proportional (and opposite) to the car's distance from the lane center (which is the CTE) - if the car is far to the right it steers hard to the left, if it's slightly to the left it steers slightly to the right.

The D, or "differential", component counteracts the P component's tendency to ring and overshoot the center line. A properly tuned D parameter will cause the car to approach the center line smoothly without ringing.

The I, or "integral", component counteracts a bias in the CTE which prevents the P-D controller from reaching the center line. This bias can take several forms, such as a steering drift (as in the Control unit lessons), but I believe that in this particular implementation the I component particularly serves to reduce the CTE around curves.

The final PID controller implementation performed much like in the following video (although, the controller performance suffered due to the screen recording consuming computation resources away from the websocket).

Final Parameters

The following video demonstrates the subtle difference in performance when the I component is removed from the controller. Notice that the center line is not followed as closely around curves.

I Parameter Removed

This final video demonstrates the disastrous effects of removing the D component from the controller. It begins to ring back and forth across the center line until finally leaving the track.

D Parameter Removed

    Describe how the final hyperparameters were chosen.

Hyperparameters were tuned manually at first. This was necessary because the narrow track left little room for error, and when attempting to automate parameter optimization (such as Twiddle) it was very common for the car to leave the track, thus invalidating the optimization. Once I found parameters that were able to get the car around the track reliably, I then implemented Twiddle. I felt it necessary to complete a full lap with each change in parameter because it was the only way to get a decent "score" (total error) for the parameter set. For this reason my parameter changes are allowed to "settle in" for 100 steps and are then evaluated for the next 2000 steps. In all, I allowed Twiddle to continue for over 1 million steps (or roughly 500 trips around the track) to fine tune the parameters to their final values (P: 0.134611, I: 0.000270736, D: 3.05349).

I also implemented a PID controller for the throttle, to maximize the car's speed around the track. The throttle PID controller is fed the magnitude of the CTE because it doesn't make sense to throttle up for right-side CTE and down for left-side CTE, for example. For this reason the throttle controller doesn't include an I component, which would only grow indefinitely. The throttle controller was also fine-tuned using the same Twiddle loop, simultaneously with the steering controller. Though this is not an ideal setup (tuning parameters for two different controllers simultaneously), it still mostly converged to a good (if I do say so myself) solution.

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

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

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

---
> **Date:** &nbsp; 04/06/2019  
> **Programmer:** &nbsp;John A. Betancourt G.   
> **Mail:** &nbsp;john.betancourt93@gmail.com  
> **Web:** &nbsp; www.linkedin.com/in/jhon-alberto-betancourt-gonzalez-345557129 

<img src="https://media0.giphy.com/media/l2Je6xTnSDh4dmA4o/giphy.gif" alt="drawing" width="400"/> 

Sorry for my English

<!-- Sorry for my English -->