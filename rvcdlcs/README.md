# Robotics-Toolbox-Expansion-Package

## Introduction

A expansion package for [Robotics Toolbox](http://petercorke.com/wordpress/toolboxes/robotics-toolbox) by Peter Corke. I add some new features for personal usage in my research works. I also integrated some useful third party scripts in this toolbox. 

## Update Log

- Updates on Apr. 04, 2020
    1. Add support for `SE2` and `SE3` object. 
       - Use `SE2.q` to get pose from a 1x6 vector. (discard some data)
       - Use `SE3.qeul` and `SE3.qrpy` to get pose from a 1x3 vector. (set new dimension to zeros)
    2. Update planning toolbox.
       - Add `Lidar` and `Map` class. Some old features are integrated to the static methods of `Map` class.
    3. Update robotics toolbox.
       - Update `MobilePlanarRevolute` to `PlanarRevolute` class. Some old features are integrated to the static methods of `PlanarRevolute` class.
    4. Add [`intersections`] to replace MATLAB's [`polyxpoly`] function in thirdparty scripts.

- Updates on Jan. 09, 2020
    1. Add class `CooperativeManipulation`, which supports simulation for mobile planar revolute manipulators.

- Updates on Jan. 05, 2020
    1. Add some planning functions in the folder `src/planning`.
    	- `environment`: Generate an obstacles environment with grid cell.
        - `ind2loc`: Get real coordinate from map index.
        - `map2graph`: Transfer map matrix to search graph for A* and Dijkstra.
    2. Add examples in the folder `example/path_planning`.
    3. Add edges for diagonal grid cells in `map2graph`. 

- Updates on Jan. 02, 2020
    1. Add class `MobilePlanarRevolute`, which supports mobile platform with m-dof planar revolute manipulator. (See `model/MobilePlanarRevolute.m`)
    2. Add examples for `Cuboid`, `MobileRobot` and `MobilePlanarRevolute`. (See `example/model_sim`)
    3. Add POE and DH construction method for class `MobileRobot`.

- Updates on Jan. 01, 2020
	1. Add class `Cuboid`. (See `model/Cuboid.m`)
    2. Add class `MobileRobot`, which supports all kinds of mobile manipulators. (See `model/MobileRobot.m`) 
    3. All class above support `plot` and `animate` methods same as class`SerialLink`. 	
       - Example: See `example/model_sim/mr_eg.m`.

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
	- [`inhull`](https://nl.mathworks.com/matlabcentral/fileexchange/10226-inhull ): Tests if a set of points are inside a convex hull by John D'Errico 
	- [`inpoly`](https://github.com/dengwirda/inpoly ): Fast point-in-polygon queries in MATLAB by Darren Engwirda 
    - [`intersections`](https://www.mathworks.com/matlabcentral/fileexchange/11837-fast-and-robust-curve-intersections): Fast and Robust Curve Intersections by Douglas Schwarz
2. STL Read & Write Scripts (stlreader) by 2011 [Eric Johnson](https://nl.mathworks.com/matlabcentral/profile/authors/2990507-eric-johnson)
	- [`stlread`](https://nl.mathworks.com/matlabcentral/fileexchange/22409-stl-file-reader): Imports geometry from an STL file into MATLAB.
	- `stlwrite`: Write STL file from patch or surface data.
	- `surf2stl`: Write STL file from surface data.
3. Searching Algorithm on Undirected Graph (graphsearch)  by [muhammet balcilar](https://nl.mathworks.com/matlabcentral/profile/authors/7269297-muhammet-balcilar)
    - `dijkstra`: Dijkstra planning algorithm.
    - `astar`: A* planning algorithm.
	- `dynamicpathplanning`: Dynamic path planning algorithm.
4. Iterative Regional Inflation by SDP (iris-distro) by [Robin Deits](https://github.com/rdeits)
    - `+iris`: Package for generating maximum convex region.
5. Input Scripts (hebinput) by [HebiRobotics](https://github.com/HebiRobotics/MatlabInput)
	- `HebiJoystick`: Creates a joystick object. 
	- `HebiKeyboard`: Creates a keyboard object.
6. Bidirectional Conversion between D-H Parameters and POE Parameters (dh2poe) by [Liao Wu](https://www.researchgate.net/profile/Liao_Wu4)
	- `POE2DH_Joint`: Demonstrate Lemma 1 and 2.
	- `POE2DH_Tool`: Demonstrate Lemma 3.
	- Details see [Wu, L., Crawford, R., & Roberts, J. (2017)](https://ieeexplore.ieee.org/document/7968294/).


## Installation

- See [here](https://github.com/star2dust/Robotics-Toolbox).
- To use `+iris`, the [MOSEK-MATLAB](https://github.com/star2dust/MOSEK-MATLAB) toolbox is needed.
