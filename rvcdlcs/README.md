# Robotics Toolbox Expansions  (rvcdlcs)

A lot of useful scripts for research works. ([what is new?](https://github.com/star2dust/Robotics-Toolbox/tree/master/rvcdlcs/LOG.md))

- New function for `SE2`, `SE3`.

    1. `SE2.q`: Get translation and angle in a row vector. 
    2. `SE3.qrpy`/`SE3.qeul`: Construct an SE(3) object from translation and rpy/eul angles
    3. `SE3.toqrpy`/`SE3.toqeul`: Get translation and rpy/eul angle in a row vector.
    4. `SE2.q`, `SE3.qrpy`, `SE3.qeul` support both 1x3 and 1x6 vectors.
    
- New `LineStyle` option for `SE2.plot` and `SE3.plot`. Example:  `SE3.plot('style','-.')`. (4 styles: -. / : / .. / --)
	
- New calculation functions for  4 x 4 double matrix. (See folder `src/common`)

	1. `Adg`: adjoint transformation.
	2. `invg`:  inverse transformation.
	3. `bracket`: Lie bracket.
	4. `wedge` and `vee`: calculation for twists.
	
- New functions for DH parameters and POE parameters. (See folder `src/robotics`)

    1. `poe2dh`: Transfer POE parameters to DH parameters.
    2. `dh2poe`: Transfer DH parameters to POE parameters.
    
- New `qlim` option for `SerialLink` object construction by DH parameters. (`Qlim`: m x 2 matrix)

    - Example: `SerialLink([dh,sigma], 'base', Hb, 'tool', Ht, 'qlim', Qlim)`.
    
- New classes for robotics. (See folder `src/robotics` and `example/model_tutorials`)

    1. `Cuboid`: Rigid Cuboid 3D Model class (rpy).
    2. `Cylinder`: Rigid Cylinder 3D Model class (rpy).
    3. `MobileRobot`: Mobile Robot 3D Model class (rpy).
    4. `MobilePlanarRevolute`: Mobile Planar Revolute Robot 3D Model class (rpy, stdDH). 
    
    <img src="https://s2.ax1x.com/2020/01/10/l4pMMF.jpg" alt="l4pMMF.jpg" border="0"  width="375" />
    
    5. `CooperativeManipulation`: Cooperative Manipulation 3D Model class (rpy, stdDH). 
    
    <img src="https://s2.ax1x.com/2020/01/10/l4p3Z9.gif" alt="l4p3Z9.gif" border="0" width="350" /><img src="https://s2.ax1x.com/2020/01/10/l4plqJ.gif" alt="l4plqJ.gif" border="0" width="350" />
    
- New functions for path planning. (See folder `src/planning` and `example/path_planning`).

    1. `environment`: Generate 3D obstacle environment with grid cell. 
    2. `ind2loc`: Get real coordinate from map index.
    3. `map2gphA`: Transfer map matrix to adjancent matrix A of graph.
    4. `dijkstra`: Dijkstra algorithm for graph searching.
    5. `astar`: A* algorithm for graph searching.

    <img src="https://s2.ax1x.com/2020/01/10/l4pQr4.jpg" alt="l4pQr4.jpg" border="0"  width="350" /><img src="https://s2.ax1x.com/2020/01/10/l4PdMT.gif" alt="l4PdMT.gif" border="0"  width="350" />

## Installation

For RVC, you can install it by following the procedures below.

- `git clone https://github.com/star2dust/Robotics-Toolbox.git`
- Add the toolbox folder to MATLAB path.
- Add `startup_rvc` and `startup_rte` to the last line of the file `startup.m` in your MATLAB default working folder (or run it directly).
- An example for `startup.m`:

  ```matlab
  % set toolpath
  toolpath = <the path where you put toolbox>
  % add robotics toolbox
  addpath([toolpath 'Robotics-Toolbox/rvctools/release10.3.1'])
  addpath([toolpath 'Robotics-Toolbox/rvcdlcs/src'])
  startup_rvc
  startup_rte
  ```

## References

1. [P.I. Corke. (2017). Robotics, Vision & Control. Springer. ISBN 978-3-319-54413-7.](http://petercorke.com/wordpress/toolboxes/robotics-toolbox)
2. [Murray, R. M. (1994). A mathematical introduction to robotic manipulation. CRC press.](https://www.crcpress.com/A-Mathematical-Introduction-to-Robotic-Manipulation/Murray/p/book/9780849379819)
3. [Deits, R., & Tedrake, R. (2015). Computing large convex regions of obstacle-free space through semidefinite programming. *Springer Tracts in Advanced Robotics*, *107*, 109-124.](http://groups.csail.mit.edu/robotics-center/public_papers/Deits14.pdf)
4. [Wu, L., Crawford, R., & Roberts, J. (2017). An analytic approach to converting POE parameters into D-H parameters for serial-link robots. IEEE Robotics and Automation Letters, 2(4), 2174-2179.](https://ieeexplore.ieee.org/document/7968294/)




