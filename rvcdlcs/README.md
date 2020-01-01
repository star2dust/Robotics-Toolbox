# Robotics-Toolbox-Extensions

## Introduction

A extension package for [Robotics Toolbox](http://petercorke.com/wordpress/toolboxes/robotics-toolbox) by Peter Corke. I add some new features for personal usage in my research works. I also integrated some useful third party scripts in this toolbox. 

## Features

- Updates on Jan. 01, 2020
	1. Add class `Cuboid`. (See `model/Cuboid.m`)
    2. Add class `MobileRobot`, which supports all kinds of mobile manipulators. (See `model/MobileRobot.m`) 
    3. All class above support `plot` and `animate` method like class`SerialLink`. 	
       - Example: See `example/mr_sim/mr_eg.m`.
- Updates on Dec. 31, 2019
	1. All third party packages are put inside `thirdparty` folder and managed by `startup_rte.m`.
    2. Add `qlim` support for `SerialLink` object construction by DH parameters. (`Qlim`: m x 2 matrix)
       - Example: `SerialLink([dh,sigma], 'base', Hb, 'tool', Ht, 'qlim', Qlim)`.
- Updates on Dec. 27, 2019
    1. Add translations between DH parameters and POE parameters.
    2. Add `LineStyle` support for `trplot2` and `trplot`. 
       -  Example:  `SE3.plot('style','-.')`. (4 styles: -. / : / .. / --)
    3. Add pose support for `SE2` and `SE3` object. 
       - Use `SE2.q` and `SE3.toqeul`, `SE3.toqrpy` to get pose.
       - Use `SE3.qeul` and `SE3.qrpy` to construct `SE3` object.
- Updates on Dec. 20, 2019
    1. Add calculation support for  4 x 4 double matrix.
       - `Adg`: adjoint transformation.
       - `invg`:  inverse transformation.
       - `bracket`: Lie bracket.
       - `wedge` and `vee`: `w^` and `Wˇ` calculation for twists.
    2. Add velocity and acceleration support for `calctraj`.

## Third Party Scripts

1. Convex Hull Scripts (cvxhull)
	- [inhull](https://nl.mathworks.com/matlabcentral/fileexchange/10226-inhull ): Tests if a set of points are inside a convex hull (c) 2009 John D'Errico 
	- [inpoly](https://github.com/dengwirda/inpoly ): Fast point-in-polygon queries in MATLAB (c) Darren Engwirda 

2. STL Read & Write Scripts (stlreader) (c) 2011 [Eric Johnson](https://nl.mathworks.com/matlabcentral/profile/authors/2990507-eric-johnson)
	- [stlread](https://nl.mathworks.com/matlabcentral/fileexchange/22409-stl-file-reader): Imports geometry from an STL file into MATLAB.
	- stlwrite: Write STL file from patch or surface data.
	- surf2stl: Write STL file from surface data.

3. Planning Scripts 
    - [graphsearch](https://nl.mathworks.com/matlabcentral/fileexchange/68871-robotpathplanning): Dijkstra, A star and dynamic planning on undirected graph (c) [muhammet balcilar](https://nl.mathworks.com/matlabcentral/profile/authors/7269297-muhammet-balcilar).  
    - [iris-distro](https://github.com/rdeits/iris-distro): Iterative regional inflation by SDP. 

4. Input Scripts (hebinput) (c) [HebiRobotics](https://github.com/HebiRobotics/MatlabInput)
	-  `HebiJoystick` creates a joystick object. 
	-  `HebiKeyboard` creates a keyboard object.
	-  Library for getting keyboard and joystick input into MATLAB.

## Installation

- See [here](https://github.com/star2dust/Robotics-Toolbox).
- To use `iris`, the [MOSEK-MATLAB](https://github.com/star2dust/MOSEK-MATLAB) toolbox is needed.

## References

1. [Murray, R. M. (1994). A mathematical introduction to robotic manipulation. CRC press.](https://www.crcpress.com/A-Mathematical-Introduction-to-Robotic-Manipulation/Murray/p/book/9780849379819)
2. [Deits, R., & Tedrake, R. (2015). Computing large convex regions of obstacle-free space through semidefinite programming. *Springer Tracts in Advanced Robotics*, *107*, 109–124.](http://groups.csail.mit.edu/robotics-center/public_papers/Deits14.pdf)
