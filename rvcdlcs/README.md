# Robotics-Toolbox-Extensions

## Introduction

A extension package for [Robotics Toolbox](http://petercorke.com/wordpress/toolboxes/robotics-toolbox) by Peter Corke. I add some new functions to based on the theories I learned from [R. Murray Book (1994)](https://www.crcpress.com/A-Mathematical-Introduction-to-Robotic-Manipulation/Murray/p/book/9780849379819).

## Features

- Add support for SE(3) matrix (double), now you can directly calculate the adjoint transformation, inverse, Lie bracket, wedge(^) and vee(ˇ) without transform it to SE3 object.
- Simplify the plot for SE3 and SE2 object, you can easily change the color and line style.
- Add support for velocity and acceleration in trajectory generation.
- Add some templates for rigid body and mR manipulator.


## Installation

- Make sure you have already installed the Robotics Toolbox of Peter Corke

- `git clone https://github.com/star2dust/Robotics-Toolbox-Extensions.git`

- Add the project folder to matlab path

- Add `startup_rte` to the last line of the file `startup.m` in you matlab default working folder (or just run it directly)

- Examples:

  ```matlab
  % add robotics toolbox
  addpath([toolpath 'Robotics-Toolbox/rvctools/release10.3.1'])
  addpath([toolpath 'Robotics-Toolbox/rvcdlcs'])
  startup_rvc
  startup_rte
  ```

  
