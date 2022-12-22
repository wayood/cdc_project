# PF1 development
this is a development for our project.

# Requirement
* ros melodic
# Installation

```
$ chmod +x install.sh
$ ./install.sh
```
visual saliency installation
```
$ cd visual_saliency_ros
$ git submodule update --init --recursive
$ cd ..
$ catkin build -DCMAKE_BUILD_TYPE=Release visual_saliency
$ source devel/setup.bash
```
# Note

## docker installation

*https://qiita.com/iganari/items/fe4889943f22fd63692a

## use ros melodic in docker(GUI)

*https://qiita.com/karaage0703/items/957bdc7b4dabfc6639da

