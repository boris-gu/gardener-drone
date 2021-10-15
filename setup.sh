#!/bin/bash
# https://discuss.px4.io/t/create-custom-model-for-sitl/6700/17
# https://github.com/willinum/SPEC21/wiki/custom-model-for-PX4-sitl-GAZEBO-simulation
RED='\033[1;31m'
YELLOW='\033[1;33m'
NOCOLOR='\033[0m'

if [ -z $1 ] || [ $1 == "--help" ]; then
    echo "Usage: setup.sh PATH_TO_PX4"
    echo "  For example:"
    echo "  setup.sh /home/user/PX4-Autopilot"
elif [ ! -d $1 ]; then
    echo -en "${RED}[ERR]: ${NOCOLOR}"
    echo "Directory $1 not found"
else
    SCRIPT=$(realpath $0)
    SCRIPTPATH=$(dirname $SCRIPT)

    #1 Create a model under Tools/sitl_gazebo/models
    MODELS=$1/Tools/sitl_gazebo/models
    if [ -d $MODELS ]; then
        cp -r $SCRIPTPATH/models/* $MODELS
    else
        echo -en "${RED}[ERR]: ${NOCOLOR}"
        echo "Directory $MODELS not found"
    fi

    #2 Create a world file in Tools/sitl_gazebo/worlds
    WORLDS=$1/Tools/sitl_gazebo/worlds
    if [ -d $WORLDS ]; then
        cp -r $SCRIPTPATH/worlds/* $WORLDS
    else
        echo -en "${RED}[ERR]: ${NOCOLOR}"
        echo "Directory $WORLDS not found"
    fi

    #3 Create an airframe file under ROMFS/px4fmu_common/init.d-posix/airframes
    AIRFRAMES=$1/ROMFS/px4fmu_common/init.d-posix/airframes
    if [ -d $AIRFRAMES ]; then
        cp $SCRIPTPATH/airframes/* $AIRFRAMES
    else
        echo -en "${RED}[ERR]: ${NOCOLOR}"
        echo "Directory $AIRFRAMES not found"
    fi

    #4 Add the airframe name to the file platforms/posix/cmake/sitl_target.cmake
    FILE_SITL_TARGET=$1/platforms/posix/cmake/sitl_target.cmake
    if [ -f $FILE_SITL_TARGET ]; then
        lineNum=$(grep -n "set(models" $FILE_SITL_TARGET | head -n 1 | cut -d: -f1)
        lineNum=$((lineNum+1))
        gardIrisOK=$(grep -n "gardener_iris" $FILE_SITL_TARGET | head -n 1 | cut -d: -f1)
        if [ -z $gardIrisOK ]; then
            sed -i "${lineNum}i \\\tgardener_iris" $FILE_SITL_TARGET
        else
            echo -en "${YELLOW}[WARNING]: ${NOCOLOR}"
            echo "gardener_iris already contains in sitl_target.cmake"
        fi
        gardDroneOK=$(grep -n "gardener_drone" $FILE_SITL_TARGET | head -n 1 | cut -d: -f1)
        if [ -z $gardIrisOK ]; then
            sed -i "${lineNum}i \\\tgardener_drone" $FILE_SITL_TARGET
        else
            echo -en "${YELLOW}[WARNING]: ${NOCOLOR}"
            echo "gardener_drone already contains in sitl_target.cmake"
        fi
    else
        echo -en "${RED}[ERR]: ${NOCOLOR}"
        echo "File $FILE_SITL_TARGET not found"
    fi

    #5 Add the airframe name to the file ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
    FILE_CMAKELISTS=$1/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
    if [ -f $FILE_CMAKELISTS ]; then
        lineNum=$(grep -n "px4_add_romfs_files(" $FILE_CMAKELISTS | head -n 1 | cut -d: -f1)
        lineNum=$((lineNum+1))
        gardIrisOK=$(grep -n "131301_gardener_iris" $FILE_CMAKELISTS | head -n 1 | cut -d: -f1)
        if [ -z $gardIrisOK ]; then
            sed -i "${lineNum}i \\\t131301_gardener_iris" $FILE_CMAKELISTS
        else
            echo -en "${YELLOW}[WARNING]: ${NOCOLOR}"
            echo "gardener_iris already contains in CMakeLists.txt"
        fi
        gardDroneOK=$(grep -n "131302_gardener_drone" $FILE_CMAKELISTS | head -n 1 | cut -d: -f1)
        if [ -z $gardIrisOK ]; then
            sed -i "${lineNum}i \\\t131302_gardener_drone" $FILE_CMAKELISTS
        else
            echo -en "${YELLOW}[WARNING]: ${NOCOLOR}"
            echo "gardener_drone already contains in CMakeLists.txt"
        fi
    else
        echo -en "${RED}[ERR]: ${NOCOLOR}"
        echo "File $FILE_CMAKELISTS not found"
    fi

    echo "Setup completed"
fi