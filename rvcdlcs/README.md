# Robotics-Toolbox-Extensions

## Introduction

A extension package for [Robotics Toolbox](http://petercorke.com/wordpress/toolboxes/robotics-toolbox) by Peter Corke. I add some new functions to based on the theories I learned from [R. Murray Book (1994)](https://www.crcpress.com/A-Mathematical-Introduction-to-Robotic-Manipulation/Murray/p/book/9780849379819).

## Features

- Add support for SE(3) matrix (double), now you can directly calculate the adjoint transformation, inverse, Lie bracket, wedge(^) and vee(ˇ) without transform it to SE3 object.
- Simplify the plot for SE3 and SE2 object, you can easily change the color and line style.
- Add support for velocity and acceleration in trajectory generation.
- Add some templates for rigid body and mR manipulator.
- Add translations between DH parameters and POE parameters.

## Third Party Scripts

1. Convex Hull Scripts (cvxhull)
	- [inhull](https://nl.mathworks.com/matlabcentral/fileexchange/10226-inhull ): Tests if a set of points are inside a convex hull (c) 2009 John D'Errico 
	- [inpoly](https://github.com/dengwirda/inpoly ): Fast point-in-polygon queries in MATLAB (c) Darren Engwirda 

2. STL Read & Write Scripts (stlreader) (c) 2011 [Eric Johnson](https://nl.mathworks.com/matlabcentral/profile/authors/2990507-eric-johnson)
	- [stlread](https://nl.mathworks.com/matlabcentral/fileexchange/22409-stl-file-reader): Imports geometry from an STL file into MATLAB.
	- stlwrite: Write STL file from patch or surface data.
	- surf2stl: Write STL file from surface data.

3. Planning Scripts 
    - ([graphsearch](https://nl.mathworks.com/matlabcentral/fileexchange/68871-robotpathplanning)): Dijkstra, A star and dynamic planning on Undirected Graph by [muhammet balcilar](https://nl.mathworks.com/matlabcentral/profile/authors/7269297-muhammet-balcilar).  
    - ([iris-distro](https://github.com/rdeits/iris-distro)): Iterative Regional Inflation by SDP. 

## Installation

- See [here](https://github.com/star2dust/Robotics-Toolbox).

- To use `iris`, the [MOSEK-MATLAB](https://github.com/star2dust/MOSEK-MATLAB) toolbox is needed.