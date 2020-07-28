# Robotics, Vision &amp; Control (rvctools)

## Introduction

- The book [Robotics, Vision & Control, second edition](http://www.petercorke.com/RVC) (Corke, 2017) is a detailed introduction to mobile robotics, navigation, localization; and arm robot kinematics, Jacobians and dynamics illustrated using the Robotics Toolbox for MATLAB.
- The manual [robot.pdf (3.8 MB)](http://petercorke.com/wordpress/?ddownload=343) is a PDF file is a printable document (over 400 pages). It is auto-generated from the comments in the MATLAB code. You can find this in the Toolbox as rvctools/robot/robot.pdf.

## Version

- The version of toolbox is 10.3.1. 
- For newest version, see https://petercorke.com/toolboxes/robotics-toolbox/.
- My personal scripts and installation, see [rvcdlcs](https://github.com/star2dust/Robotics-Toolbox/tree/master/rvcdlcs).

## Installation

For RVC, you can install it by following the procedures below.

- `git clone https://github.com/star2dust/Robotics-Toolbox.git`
- Add the toolbox folder to MATLAB path.
- Add `startup_rvc` to the last line of the file `startup.m` in your MATLAB default working folder (or run it directly).
- An example for `startup.m`:

  ```matlab
  % set toolpath
  toolpath = <the path where you put toolbox>
  % add robotics toolbox
  addpath([toolpath 'Robotics-Toolbox/rvctools/release10.3.1'])
  startup_rvc
  ```

## References

1. [P.I. Corke. (2017). Robotics, Vision & Control. Springer. ISBN 978-3-319-54413-7.](http://petercorke.com/wordpress/toolboxes/robotics-toolbox)


