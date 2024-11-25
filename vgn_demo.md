## On desktop (activate robot interface)

`ssh uq-nuc`

* go to https://localhost:6789/desk/ to use the arm web GUI

    * Unlock breaks
    * Activate FCI

## On NUC (start robot controller and vgn node)

In one terminal: start roscore

`src_ros1`

`roscore`

In the 2nd terminal: start camera, camera is pluged into NUC
`src_as_ws`

`src_envars`

`roslaunch uq_bringup realsense.launch`

In the 3rd terminal: start the robot controller and vgn node

`src_as_ws`

`src_envars`

`cd active_search_ws/src/vgn`

`source .venv/bin/activate`

`roslaunch vgn nuc_panda_grasp.launch` 

## On desktop (visualize and vgn grasp)

In 1st terminal: rviz

`ros_ridgeback`

`cd dev_ws/active_search`

`source devel/setup.bash`

`roslaunch vgn vgn_desktop.launch`

In a 2nd terminal: vgn grasp

`ros_ridgeback`

`source devel/setup.bash`

`cd dev_ws/active_search/src/vgn`

`source .venv/bin/activate`

`python scripts/panda_grasp.py --model data/models/vgn_conv.pth`