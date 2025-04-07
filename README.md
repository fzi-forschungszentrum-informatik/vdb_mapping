VDB Mapping Core Library
===
DISCLAIMER: This library is still under development. Be warned that some interfaces will be changed and/or extended in the future.

The VDB Mapping core library was primarily developed to be used in combination with the corresponding [ROS wrapper](https://github.com/fzi-forschungszentrum-informatik/vdb_mapping_ros) or [ROS2 wrapper](https://github.com/fzi-forschungszentrum-informatik/vdb_mapping_ros2)

## Getting Started

### Requirements
This library requires [OpenVDB](https://www.openvdb.org/) as it is build around it. 
The library requires at least Version 8.3 and should work with all versions above.

As the apt packages are quite outdated for most systems, we recommend building at least OpenVDB v9.0.0 from source using the provided [build instructions](https://github.com/AcademySoftwareFoundation/openvdb)

### Build instructions

The library can be either used as plain c++ library or in combination with the afore mentioned ROS wrapper.

#### Dependencies

The library requires the following dependencies to build correctly
``` bash
apt-get install -y libeigen3-dev
apt-get install -y libtbb-dev
apt-get install -y libpcl-dev
apt-get install -y libilmbase-dev
```

#### Plain cmake
To build this package as a standalone library, follow the usual cmake building steps:
``` bash
git clone https://github.com/fzi-forschungszentrum-informatik/vdb_mapping
cd vdb_mapping
mkdir build && cd build
cmake ..
make -j8
make install
```

#### ROS Workspace
In case you want build this library inside of a ROS workspace in combination with [VDB Mapping ROS](https://github.com/fzi-forschungszentrum-informatik/vdb_mapping_ros), you cannot use catkin_make since this library is not a catkin package.
Instead you have to use [catkin build](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html) or [catkin_make_isolated](http://docs.ros.org/independent/api/rep/html/rep-0134.html) to build the workspace.

``` bash
# source global ros
source /opt/ros/<your_ros_version>/setup.{zsh/bash}

# create a catkin workspace
mkdir -p ~/catkin_ws/src && cd catkin_ws

# clone packages
git clone https://github.com/fzi-forschungszentrum-informatik/vdb_mapping

# install dependencies
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -y

# build the workspace.
catkin build

# source the workspace
source deve/setup.bash

```

#### ROS2 Workspace

``` bash
# source global ros
source /opt/ros/<your_ros_version>/setup.{zsh/bash}

# create a catkin workspace
mkdir -p ~/colcon_ws/src && cd ~/colcon_ws/src

# clone packages
git clone https://github.com/fzi-forschungszentrum-informatik/vdb_mapping
git clone https://github.com/fzi-forschungszentrum-informatik/vdb_mapping_ros2

# install dependencies
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -y

# build the workspace.  
colcon build

# source the workspace
source install/setup.bash
```
## Citation

Thanks that you read until here and please let us know if you run into any issues or have suggestions for improvements.
If you use our work in your publications please feel free to cite our manuscript:
```bibtex
  @inproceedings{besselmann2021vdb,
  title={VDB-Mapping: a high resolution and real-time capable 3D mapping framework for versatile mobile robots},
  author={Besselmann, M Grosse and Puck, Lennart and Steffen, Lea and Roennau, Arne and Dillmann, R{\"u}diger},
  booktitle={2021 IEEE 17th International Conference on Automation Science and Engineering (CASE)},
  pages={448--454},
  year={2021},
  organization={IEEE}
  doi={10.1109/CASE49439.2021.9551430}}
}
```
or for the remote mapping case:
```bibtex
@incollection{besselmann2022remote,
  title={Remote VDB-Mapping: A Level-Based Data Reduction Framework for Distributed Mapping},
  author={Besselmann, Marvin Grosse and R{\"o}nnau, Arne and Dillmann, R{\"u}diger},
  booktitle={Robotics in Natural Settings: CLAWAR 2022},
  pages={448--459},
  year={2022},
  publisher={Springer}
  doi={10.1007/978-3-031-15226-9_42}
}
```

## Acknowledgement

The research leading to this package has received funding from the German Federal Ministry of Education and Research under grant agreement No. 13N14679:  

<a href="https://www.bmbf.de/">
  <img src="https://robdekon.de/user/themes/robdekon/images/BMBF_gefoerdert_2017_web.de.svg"
  alt="bmbf" height="80">
</a>  
  
ROBDEKON - Robotic systems for decontamination in hazardous environments  
More information: [robdekon.de](https://robdekon.de/)  


<a href="https://robdekon.de/">
  <img src="https://robdekon.de/user/themes/robdekon/images/robdekon_logo_web.svg"
  alt="robdekon_logo" height="40">
</a>  
