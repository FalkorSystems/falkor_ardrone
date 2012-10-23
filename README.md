# falkor_ardrone: A ROS package to control an AR.Drone autonomously

## Introduction

"falkor_ardrone" is a [ROS](http://ros.org/ "Robot Operating System") package which uses the "ardrone_autonomy" package (originally from brown, now from AutonomyLab) to implement autonomous control functionality on an AR.Drone. This was originally a part of the [fork from "ardrone_autonomy"](http://github.org/FalkorSystems/ardrone_autonomy), but now it has been seperated out. The old forked repository still exists for history purposes but it is stale.

### Updates

- *October 23, 2012*: Create this seperate package

## Usage

Once the AR.Drone is connected to your computer via WiFi, run

     ```bash
     $ roslaunch falkor_ardrone track.launch

This launches the `ardrone_driver` from `ardrone_autonomy` to communicate with the AR.Drone, as well as the nodes needed for the AR.Drone to track the target. The cascade files used for detection are in the directory `cascade/`. Edit the `cascadefile` parameter in roslaunch to change which cascade file to use.

Once everything is up and running you should see a window with the front camera from the AR.Drone and a square yellow box around the detected object. The PS3 controller has the following commands:

* Right gamepad
** Triangle: Takeoff
** Circle: Reset
** Cross: Land
** Square: Start/Stop following

* Left gamepad
** Up: Bring AR.Drone closer (increase goal size of the target)
** Down: Move AR.Drone further away (decrease goal size of the target)

* Joysticks (Mode 1)
** Left Joystick: Pitch and Roll
** Right Joystick: Altitude and Yaw

## License

Copyright (c) 2012, Falkor Systems, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.