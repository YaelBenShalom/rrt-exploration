# RRT Search

## Table of Contents

- [Overview](#overview)
  - [Simple RRT](#simple-rrt)
  - [Planning a Path with Obstacles](#planning-a-path-with-obstacles)
- [Usage and Configuration Instructions](#usage-and-configuration-instructions)
- [Results](#results)


## Overview

A Rapidly-Exploring Random Tree (RRT) is a fundamental path planning algorithm in robotics, first developed by Steven LaValle in 1998.
Path planning is the task of moving a robot from one location to another, while avoiding obstacles and satisfying constraints.

<p align="center">
    <img align="center" src="https://github.com/YaelBenShalom/RRT-Challenge/blob/master/images/4.gif">
</p>
        
### Simple RRT

Simple RRT implementation in a two-dimensional domain, _D=[0,100]X[0,100]_

### Planning a Path with Obstacles

RRT implementation in a two-dimensional domain, _D=[0,100]X[0,100]_, with circular obstacles.

## Usage and Configuration Instructions

1. Clone the repository:
  
  `git clone https://github.com/YaelBenShalom/RRT-Challenge.git`

2. Generate a Rapidly-Exploring Random Tree (RRT) using:

  `python3 random_tree.py`

  (The number of obstacles can be changed from within the code).

## Results

An RRT implementation without obstacles:
<p align="center">
    <img align="center" src="https://github.com/YaelBenShalom/RRT-Challenge/blob/master/images/Task_1(3).png">
</p>

An RRT implementation with 20 obstacles:
<p align="center">
    <img align="center" src="https://github.com/YaelBenShalom/RRT-Challenge/blob/master/images/Task_2(5).png">
</p>
