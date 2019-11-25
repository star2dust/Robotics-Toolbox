# Robotics-Toolbox
## Introduction

A robotics toolbox and a few useful scripts.

1. Robotic Toolbox
	- Robotics, Vision &amp; Control (rvctools) (c) Peter Corke 1992-2017 http://www.petercorke.com 
	- Robotics Toolbox Extensions  (rvcdlcs)  (c) 2019 star2dust 

2. Convex Hull Scripts (cvxhull)
	- inhull: Tests if a set of points are inside a convex hull (c) 2009 John D'Errico https://nl.mathworks.com/matlabcentral/fileexchange/10226-inhull 
	- inpoly: Fast point-in-polygon queries in MATLAB (c) Darren Engwirda https://github.com/dengwirda/inpoly 

3. Input Scripts (hebinput) (c) HebiRobotics https://github.com/HebiRobotics/MatlabInput 
	-  HebiJoystick creates a joystick object. 
	-  HebiKeyboard creates a keyboard object.
	-  Library for getting keyboard and joystick input into MATLAB.

4. STL Read & Write Scripts (stlreader) (c) 2011 Eric Johnson
	- stlread: Imports geometry from an STL file into MATLAB.
	- stlwrite: Write STL file from patch or surface data.
	- surf2stl: Write STL file from surface data.

5. Convex Set Projection Scripts (cvxproj) (c) 2019 star2dust
	- cvxproj: Calculate the projection of x to a convex set defined by Ax+b<=0.
	- circproj: Calculate the projection of x to a convex set defined by a circle B(c,r).

## Installation

For RVC, you can install it by following the procedures below.

- `git clone https://github.com/star2dust/Robotics-Toolbox.git`
- Add the toolbox folder to MATLAB path:
  ```matlab
  addpath('<RVCHOME>/rvctools/release10.3.1');
  addpath('<RVCHOME>/rvcdlcs/src');
  ```
- Add `startup_rvc` to the last line of the file `startup.m` in your MATLAB default working folder.

