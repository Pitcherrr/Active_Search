#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/tom/dev_ws/thesis_ws/src/robot_helpers"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/tom/dev_ws/thesis_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/tom/dev_ws/thesis_ws/install/lib/python3/dist-packages:/home/tom/dev_ws/thesis_ws/build/robot_helpers/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/tom/dev_ws/thesis_ws/build/robot_helpers" \
    "/usr/bin/python3" \
    "/home/tom/dev_ws/thesis_ws/src/robot_helpers/setup.py" \
     \
    build --build-base "/home/tom/dev_ws/thesis_ws/build/robot_helpers" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/tom/dev_ws/thesis_ws/install" --install-scripts="/home/tom/dev_ws/thesis_ws/install/bin"
