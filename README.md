# LIRS-USAR-SIM-sample



## Getting started

## Setup
### Create ROS workspace
```sh
mkdir -p catkin_ws/src
```
### Clone repo
```sh
git clone https://gitlab.com/LIRS_Projects/lirs-usar-sim-sample.git catkin_ws/src/usar_sample -b melodic-devel
```
### Install dependencies
Go into ```usar_sample/hazardous_zones``` folder and run:
```sh
./dependencies.sh
```
### Compile project
Go into ```catkin_ws``` folder and run:
```sh
catkin_make
```
### Source ROS workspace
```sh
source devel/setup.bash
```

## Simulation
### Run Husky robot within virtual contamination area
```sh
roslaunch hazardous_zones usar_modeling.launch
```
