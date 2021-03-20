# Footstep Affordance

[![Build Status](https://travis-ci.org/RajPShinde/footstep_affordance.svg?branch=master)](https://travis-ci.org/RajPShinde/footstep_affordance)
[![License BSD 3-Clause](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://github.com/RajPShinde/footstep_affordance/blob/master/LICENSE)
---

## Overview
This is a ROS package to create a cost map of a terrain based on the slope for providing possible foothold locations for legged robots. It uses the 3D-Occupancy from the octomap to create a height map of the terrain, this height map is used to obtain slope at each point using PCA.  

<p align="center">
<img src="data/doggo.gif"/>
</p>
<p align="center">
<img src="data/costmap.png" width="600"/>
</p>

## Dependencies
1. Ubuntu 16.04 and above
2. ROS Kinetic and above
3. C++ 11/14/17
4. octomap-ros
5. octomap-server
4. Eigen

## Subscriptions
1. ```/octomap_binary```

## Publications
1. ```/heightMapVisualization```
2. ```/costMapVisualization```

## Params
1. ```world_frame```(default:/odom)
2. ```robot_frame```(default:/base_link)

## Citations
C. Mastalli, I. Havoutis, M. Focchi, D. G. Caldwell, C. Semini, Motion planning for challenging locomotion: a study of decoupled and coupled approaches, IEEE International Conference on Robotics and Automation (ICRA), 2017

## Disclaimer
```
BSD 3-Clause License

Copyright (c) 2020, Raj Shinde
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```
