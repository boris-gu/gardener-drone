# gardener-drone
Drone model (modified Iris) for Gazebo simulator.
Модифицированный Iris c ROS камерой для Gazebo simulator.

Так как PX4-Autopilot быстро развивается, за основу взята стабильная версия 1.12.3.  
Клонировать:  
```bash
git clone --recurse-submodules --branch v1.12.3 https://github.com/PX4/PX4-Autopilot.git
```

Добавьте модель, запустив `setup.sh PATH_TO_PX4`. Этот скрипт внесет все необходимые изменения в PX4-Autopilot.

Для запуска симуляции используйте:
```bash
make px4_sitl gazebo_gardener_iris
```
или (с камерой):
```bash
make px4_sitl gazebo_gardener_drone
```

## Известные проблемы
1. gardener_drone взлетает без камеры. Она остается на земле.