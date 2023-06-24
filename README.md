# LIRS-USAR-SIM-sample
## Demo
### RViz & Gazebo
![1](https://github.com/MukievMukhammad/hazardousGazebo/assets/42972450/23f268b9-6675-4303-82d3-7b2a17ada191)
### Gas propogation simulation in RViz
<iframe width="560" height="315" src="https://www.youtube.com/embed/-cnFyk64-_k" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

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
