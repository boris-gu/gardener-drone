# gardener-drone
Drone model (modified Iris) for Gazebo simulator.  
Модифицированный Iris c ROS камерой для Gazebo simulator.

Так как PX4-Autopilot быстро развивается, за основу взята стабильная версия 1.12.3.  
Клонировать:  
```bash
git clone --recurse-submodules --branch v1.12.3 https://github.com/PX4/PX4-Autopilot.git
```

Добавьте модель, запустив `setup.sh PATH_TO_PX4`. Этот скрипт внесет все необходимые изменения в PX4-Autopilot.

## Запуск
```bash
cd <PX4-Autopilot_clone>
DONT_RUN=1 make px4_sitl_default gazebo
source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default && \
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd) && \
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 gardener_drone.launch
```

Для просмотра изображения с камеры запустите команду ниже и выберите
/gardener_drone/usb_cam/image_raw:
```bash
rqt_image_view rqt_image_view
```