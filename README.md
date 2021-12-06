# gardener-drone
Drone model (modified Iris) for Gazebo simulator.  
Модифицированный Iris c ROS камерой для Gazebo simulator.

Так как PX4-Autopilot быстро развивается, за основу взята стабильная версия 1.12.3.  
Клонировать:  
```bash
git clone --recurse-submodules --branch v1.12.3 https://github.com/PX4/PX4-Autopilot.git
```

Добавьте модель, запустив `setup.sh <PATH_TO_PX4>`. Этот скрипт внесет все необходимые изменения в PX4-Autopilot.

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
Если необходимо запустить дрон в пустом мире, воспользуйтесь командой:
```bash
roslaunch px4 gardener_empty.launch
```
Если необходимо запустить другую модель добавьте параметр `vehicle:=<name>`. Например, команда ниже запускает пустой мир с дроном без камеры:
```bash
roslaunch px4 gardener_empty.launch vehicle:=gardener_iris
```

Для просмотра изображения с камеры запустите команду ниже и выберите
/gardener_drone/usb_cam/image_raw:
```bash
rqt_image_view rqt_image_view
```

## Скрипты
**simple_offb.py** - пример управления дроном (взлет)

**points_offb.py** - более сложный пример управления дроном (полет по квадрату зигзагом с изменением yaw). Проверка состояния дрона вынесена в отдельный поток

**mission_offb.py** - "прокачанная" версия points_offb.py. Дрон не отсылает одну и ту же точку определенное время, а проверяет, достиг ли он цели. Появилась возможность задать yaw по направлению полета.

**aruco_detected.py** - распознавание маркеров, результат публикуется в _gardener_aruco_detected_, можно посмотреть с помощью **rqt_image_view**

**aruco_location.py** - определение расстояния до маркеров, результат публикуется в _gardener_aruco_location_, можно посмотреть с помощью **rqt_image_view**

**aruco_calibration.py** - калибровка камеры дрона

<span style="color:red">**ВНИМАНИЕ**  
Перед запуском **aruco_location.py** необходимо получить файл калибровки камеры с помощью **aruco_calibration.py**.
Но я предварительно откалибровал камеру (файл calibration_save.yaml), так что повторно этого делать не нужно.
</span>

## Калибровка
1. Запустите симулятор и rqt_image_view
2. В симуляторе добавьте модель _ArUco calibration table_ и разверните ее к дрону
3. Запустите скрипт **aruco_calibration.py**, он начнет публиковать видео в _gardener_calibration_, можно посмотреть с помощью **rqt_image_view**.
4. Скрипт сделает череду фотографий (40 штут, через каждые 10 секунд), при этом необходимо двигать калибровочную таблицу. Таблица должна полностью попадать в кадр. Для хорошей калибровки надо получить как можно больше хороших фотографий таблицы в различных положениях.
5.  Первое число в изображении, публикуемом в _gardener_calibration_ - это количество секунд, прошедшее с прошлого снимка. Второе - количество сделаных фотографий. Когда скрипт сделает очередное фото, то на видео появится "вспышка" - цвета изображения на миг инвертируются.
