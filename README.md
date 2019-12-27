# Robotics-Toolbox
## Introduction

A robotics toolbox and a few useful scripts.

1. Robotic Toolbox
	- Robotics, Vision &amp; Control (rvctools) (c) [Peter Corke](http://www.petercorke.com) 1992-2017 
	- Robotics Toolbox Extensions  (rvcdlcs)  (c) star2dust 2019 ([what is new?](https://github.com/star2dust/Robotics-Toolbox/tree/master/rvcdlcs))

2. Input Scripts (hebinput) (c) [HebiRobotics](https://github.com/HebiRobotics/MatlabInput)
	-  HebiJoystick creates a joystick object. 
	-  HebiKeyboard creates a keyboard object.
	-  Library for getting keyboard and joystick input into MATLAB.


## Installation

For RVC, you can install it by following the procedures below.

- `git clone https://github.com/star2dust/Robotics-Toolbox.git`

- Add the toolbox folder to matlab path.
  
- Add `startup_rvc` and `startup_rte` to the last line of the file `startup.m` in your matlab default working folder (or just run it directly).

- Examples:

  ```matlab
  % add robotics toolbox
  addpath([toolpath 'Robotics-Toolbox/rvctools/release10.3.1'])
  addpath([toolpath 'Robotics-Toolbox/rvcdlcs'])
  startup_rvc
  startup_rte
  ```


