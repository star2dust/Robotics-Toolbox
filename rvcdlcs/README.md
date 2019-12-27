# Robotics-Toolbox-Extensions

## Introduction

A extension package for [Robotics Toolbox](http://petercorke.com/wordpress/toolboxes/robotics-toolbox) by Peter Corke. I add some new features for personal usage in my research works. I also integrated some useful third party scripts in this toolbox. 

## Features

Updates on Dec. 27, 2019

1. Add translations between DH parameters and POE parameters.
2. Add 'LineStyle' support for `trplot2` and `trplot`. (use 'style').
3. Add pose support for SE2 and SE3. (use 'SE2.q' and 'SE3.toqeul', 'SE3.toqrpy' to get pose, 'SE3.qeul' and 'SE3.qrpy' to construct SE3 object.)

Updates on Dec. 20, 2019

1. Add calculation support (adjoint transformation`Adg`, inverse`invg`, Lie bracket`bracket`, wedge`^` and vee`ˇ` ) for SE(3) double matrix.
3. Add velocity and acceleration support for `calctraj`.
4. Add class for 2D and 3D rigid cuboid.
4. Add some functions for mR manipulator.

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

## Installation

- See [here](https://github.com/star2dust/Robotics-Toolbox).
- To use `iris`, the [MOSEK-MATLAB](https://github.com/star2dust/MOSEK-MATLAB) toolbox is needed.

## References

1. [Murray, R. M. (1994). A mathematical introduction to robotic manipulation. CRC press.](https://www.crcpress.com/A-Mathematical-Introduction-to-Robotic-Manipulation/Murray/p/book/9780849379819)
