# Robotics-Toolbox
## Introduction

A robotics toolbox and a few useful scripts.

- Robotics, Vision &amp; Control: (c) Peter Corke 1992-2017 http://www.petercorke.com (rvctools)
- Robotics Toolbox Extensions: (c) 2019 star2dust https://github.com/star2dust/Robotics-Toolbox.git (rvcdlcs)
- inhull: tests if a set of points are inside a convex hull (c) 2009 John D'Errico https://nl.mathworks.com/matlabcentral/fileexchange/10226-inhull (inhull)
- INPOLY: Fast point-in-polygon queries in MATLAB (c) Darren Engwirda https://github.com/dengwirda/inpoly (inpoly)
-  Library for getting keyboard and joystick input into MATLAB https://github.com/HebiRobotics/MatlabInput (hebinput)
- A STL file reader (c) 2011 Eric Johnson (stlreader)

## Installation

For RVC, you can install it by following the procedures below.

- `git clone https://github.com/star2dust/Robotics-Toolbox.git`
- Add the toolbox folder to MATLAB path:
  ```matlab
  addpath('<RVCHOME>/rvctools/release10.3.1');
  addpath('<RVCHOME>/rvcdlcs/src');
  ```
- Add `startup_rvc` to the last line of the file `startup.m` in your MATLAB default working folder.

