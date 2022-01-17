# gardener-drone
Drone model (modified Iris) for Gazebo simulator.  
Модифицированный Iris c оптическим потоком и камерой Intel RealSense для Gazebo simulator.

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
roslaunch px4 mavros_posix_sitl.launch vehicle:=gardener_drone
```
Если необходимо запустить другую модель добавьте параметр `vehicle:=<name>`. Например:
```bash
roslaunch px4 gardener_drone.launch vehicle:=iris
```

## Данные с дрона
Дрон публикует в топики ROS данные с камеры RealSense.

Для просмотра изображения с камеры запустите команду `rqt_image_view` и выберите
/realsense/color/image_raw.
![rqt_image_view](/img/001_rqt_image_view.png "Отображение топика /realsense/color/image_raw")


Для просмотра облака точек, генерируемое камерой RealSense совершите следующие шаги:
1. Запустите команду `rviz`
2. Слева в поле "Fixed Frame" вместо "map" введите "gardener"
3. Слева внизу нажмите кнопку Add->By topic->realsense/depth/points/PointCloud2, нажмите OK

![gazebo](/img/002_gazebo.png "Отображение коптера в Gazebo")
![rviz](/img/003_rviz.png "Облако точек в rviz")

## Скрипты
**offb_simple.py** - пример управления дроном (взлет)

**offb_points.py** - более сложный пример управления дроном (полет по квадрату зигзагом с изменением yaw). Проверка состояния дрона вынесена в отдельный поток

**offb_mission.py** - "прокачанная" версия offb_points.py. Дрон не отсылает одну и ту же точку определенное время, а проверяет, достиг ли он цели. Появилась возможность задать yaw по направлению полета.

**offb_velocity.py** - пример управления дроном при помощи задания скоростей (полет по спирали вверх и вниз)

**offb_keyboard.py** - управление дроном при помощи клавиатуры  
Стрелки - ROLL и PITCH  
WASD - YAW и высота

<span style="color:red">**ВНИМАНИЕ**  
В скрипте **offb_keyboard.py** для чтения клавиатуры необходим pynput. Эта библиотека плохо работает в Wayland. Поэтому либо переключитесь в сеанс X.Org, либо запустите терминал при помощи XWayland (в моем случае это была команда `GDK_BACKEND=x11 tilix`).
</span>

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
