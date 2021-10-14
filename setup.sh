#!/bin/bash
# https://github.com/willinum/SPEC21/wiki/custom-model-for-PX4-sitl-GAZEBO-simulation

if [ -z $1 ] || [ $1 == "--help" ]; then
    echo "Usage: setup.sh PATH_TO_PX4"
    echo "       For example:"
    echo "       setup.sh /home/user/PX4-Autopilot"
elif ! [ -d $1 ]; then
    echo "No directory"
else
    SCRIPT=$(realpath $0)
    SCRIPTPATH=$(dirname $SCRIPT)
    cp -r $SCRIPTPATH/models/* $1/Tools/sitl_gazebo/models
    cp $SCRIPTPATH/airframes/* $1/ROMFS/px4fmu_common/init.d-posix/airframes
    cp $SCRIPTPATH/airframes/* $1/build/px4_sitl_default/etc/init.d-posix/airframes
    # Uncomment for RTPS
    #cp $SCRIPTPATH/airframes/* $1build/px4_sitl_rtps/etc/init.d-posix/airframes

    lineNum=$(grep -n "set(models" $1/platforms/posix/cmake/sitl_target.cmake | head -n 1 | cut -d: -f1)
    lineNum=$((lineNum+1))
    gardIrisOK=$(grep -n "gardener_iris" $1/platforms/posix/cmake/sitl_target.cmake | head -n 1 | cut -d: -f1)
    if [ -z $gardIrisOK ]; then
        sed -i "${lineNum}i \\\tgardener_iris" $1/platforms/posix/cmake/sitl_target.cmake
    else
        echo "gardener_iris already contains in sitl_target.cmake"
    fi
    gardDroneOK=$(grep -n "gardener_drone" $1/platforms/posix/cmake/sitl_target.cmake | head -n 1 | cut -d: -f1)
    if [ -z $gardIrisOK ]; then
        sed -i "${lineNum}i \\\tgardener_drone" $1/platforms/posix/cmake/sitl_target.cmake
    else
        echo "gardener_drone already contains in sitl_target.cmake"
    fi

    echo "Setup completed"
fi
