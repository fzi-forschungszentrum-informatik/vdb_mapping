VDB Mapping Core Library
===
DISCLAIMER: This library is still under development. Be warned that some interfaces will be changed and/or extended in the future.

The VDB Mapping core library was primarily developed to be used in combination with the corresponding [ROS wrapper](https://github.com/fzi-forschungszentrum-informatik/vdb_mapping_ros).

## Getting Started

### Requirements
This library requires [OpenVDB](https://www.openvdb.org/) as it is build around it. This library was initially developed using Version 5.0 and should work with all versions above.  
 Either install the apt package:  
 Ubuntu 18.04
 ``` bash
 sudo apt install libopenvdb5.0
 ```
 Ubuntu 20.04
 ``` bash
 sudo apt install libopenvdb6.2
 ```
or compile the package from source using the provided [build instructions](https://github.com/AcademySoftwareFoundation/openvdb)

### Build instructions

The library can be either used as plain c++ library or in combination with the afore mentioned ROS wrapper.

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

#### ROS workspace
In case you want build this library inside of a ROS workspace in combination with [VDB Mapping ROS](https://github.com/fzi-forschungszentrum-informatik/vdb_mapping_ros), you cannot use catkin_make since this library is not a catkin package.
Instead you have to use [catkin build](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html) or [catkin_make_isolated](http://docs.ros.org/independent/api/rep/html/rep-0134.html) to build the workspace.

``` bash
# source global ros
source /opt/ros/<your_ros_version>/setub.bash

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
