### <img src="https://i.imgur.com/ct8e7T4.png" height="80" />

[![Build Status](http://build.ros.org/buildStatus/icon?job=Kdev__xpp__ubuntu_xenial_amd64)](http://build.ros.org/view/Kdev/job/Kdev__xpp__ubuntu_xenial_amd64/) [![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.1037901.svg)](https://doi.org/10.5281/zenodo.1037901)

<img align="right" src="https://i.imgur.com/qI1Jfyl.gif" />

Xpp is a [ROS package](http://wiki.ros.org/xpp) for the visualization of motion-plans for legged robots. It
draws support areas, contact forces and motion trajectories in RVIZ and displays URDFs for specific robots, including a one-legged, a two-legged hopper and [HyQ]. More example motions can be seen in this [video](https://www.youtube.com/watch?v=0jE46GqzxMM&feature=youtu.be), generated by the library [towr](https://github.com/ethz-adrl/towr).  

by [:globe_with_meridians: Alexander W. Winkler](https://awinkler.github.io/ "Go to homepage")

[<img src="https://i.imgur.com/uCvLs2j.png" height="50" />](http://www.adrl.ethz.ch/doku.php "Agile and Dexterous Robotics Lab")  &nbsp; &nbsp; &nbsp; &nbsp;[<img src="https://i.imgur.com/gYxWH9p.png" height="50" />](http://www.rsl.ethz.ch/ "Robotic Systems Lab")           &nbsp; &nbsp; &nbsp; &nbsp; [<img src="https://i.imgur.com/aGOnNTZ.png" height="50" />](https://www.ethz.ch/en.html "ETH Zurich") 


## <img align="center" height="20" src="https://i.imgur.com/fjS3xIe.png"/> Install
This package has the following dependencies:

| Name | Min. Ver. | Description | Install
| --- | --- | --- | --- |
| [ROS] |  indigo | [catkin], [roscpp], ... | ```sudo apt-get install ros-<ros-distro>-desktop-full``` |
| [Eigen] | v3.2.0 | Library for linear algebra |  ```sudo apt-get install libeigen3-dev``` |

This package in part of the ROS [debians](http://wiki.ros.org/xpp), so you can directly run 

  ```bash
  sudo apt-get install ros-<ros_distro>-xpp
  ```

If you want to build this package from source to make your own modifications, execute

  ```bash
  cd catkin_workspace/src
  git clone https://github.com/leggedrobotics/xpp.git
  cd ..
  catkin_make
  source ./devel/setup.bash
  ```

## <img align="center" height="20" src="https://i.imgur.com/026nVBV.png"/> Unit Tests

Make sure everything installed correctly by running the unit tests through

    $ catkin_make run_tests
    
or if you are using [catkin tools].

    $ catkin build xpp_vis --no-deps --verbose --catkin-make-args run_tests


## <img align="center" height="20" src="https://i.imgur.com/vAYeCzC.png"/> Usage

A few examples for different robots are provided in the `xpp_examples` package. For starters, run

    $ roslaunch xpp_examples hyq_bag.launch  // or any other launch file in this package
    
<img align="right" src="https://i.imgur.com/BzOnbkS.gif" />


## <img align="center" height="20" src="https://i.imgur.com/dHQx91Q.png"/> Citation

If you use this work in an academic context, please cite the currently released version <a href="https://doi.org/10.5281/zenodo.1135005"><img src="https://zenodo.org/badge/DOI/10.5281/zenodo.1135005.svg" alt="DOI" align="center"></a> as shown [here](https://zenodo.org/record/1135005/export/hx#.Wk3szDCGPmF).


##  <img align="center" height="20" src="https://i.imgur.com/H4NwgMg.png"/> Bugs & Feature Requests

Please report bugs, request features or ask questions using the [Issue Tracker](https://github.com/leggedrobotics/xpp/issues). In case you want to contribute your own robot URDF, add the URDF + inverse kinematics function and create a PR.  

[HyQ]: https://www.iit.it/research/lines/dynamic-legged-systems
[ROS]: http://www.ros.org
[towr]: http://wiki.ros.org/towr
[rviz]: http://wiki.ros.org/rviz
[catkin tools]: http://catkin-tools.readthedocs.org/
[Eigen]: http://eigen.tuxfamily.org
[Fa2png]: http://fa2png.io/r/font-awesome/link/
