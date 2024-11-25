## On PC 

`ssh uq-nuc`

* go to https://localhost:6789/desk/ to use the arm web GUI

    * Unlock breaks
    * Activate FCI

## On NUC

In one terminal:

`src_ros1`

`roscore`

In the 2nd terminal:
`src_as_ws`

`src_envars`

`src_as_venv`

`roslaunch uq_bringup realsense.launch`

In the 3rd terminal:

`src_as_ws`

`src_envars`

`src_as_venv`

`roslaunch active_search uq_hw.launch` 

## On main 

In one terminal:

`ros_ridgeback`

`cd dev_ws/active_search`

`source devel/setup.bash`

`roslaunch active_search desktop.launch`

In a 2nd terminal:

`ros_ridgeback`

`cd dev_ws/active_search && source devel/setup.bash`

### created virtual env for vgn by 'python3 -m venv --system-site-packages .venv'

`source src/vgn/.venv/bin/activate`

`source devel/setup.bash`

`python3 src/active_search/scripts/run.py nbv`

### the ros-noetic-trac-ik is install from
### https://github.com/HIRO-group/trac_ik/blob/master/README.md


If changing policy:

in run.py pick controller from lines 106->108