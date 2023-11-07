## On PC 

* go to https://localhost:6789/desk/ to use the arm web GUI

## On NUC

In one terminal:

`src_ros1`

`roscore`

In a 2nd terminal:

`src_as_ws`

`src_envars`

`src_as_venv`

`roslaunch active_search uq_hw.launch` 

## On main pc

In one terminal:

`ros_ridgeback`

`cd .../active_search`

`source devel/setup.bash`

`roslaunch active_search desktop.launch`

In a 2nd terminal:

`ros_ridgeback`

`python3 src/active_search/scripts/run.py nbv`