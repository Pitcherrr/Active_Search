# Active Search

## Reinforcment Learning for Active Search and Grasp in Clutter 

Welcome to the repository for my honours thesis.

Active Search is an extension of the work done by M.Breyer for the ETH Zurich Autonomous Systems Lab.

Active Search allows a Franka Panda robot arm to navigate an unseen environment and find a hidden target object knowing only the target object's bounding box.

## Use:
* Requires a CUDA capable GPU
* Requires ROS Neotic
* Clone repo
```
git clone --recurse-submodules https://github.com/Pitcherrr/Active_Search.git
```

```
catkin build active_search
```

```
source devel/setup.bash
```

```
python3 src/active_search/scripts/tester_threaded.py
```

## TODO:

* Add a requirements.txt