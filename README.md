# Симуляция для космического робота

## Глобальная настройка окружения

Работает на линукс хосте, vm или нативно.

Основные инструменты: [ROS2](https://docs.ros.org/en/humble/Installation.html), [GazeboSim](https://gazebosim.org/docs/all/getstarted).

> Версия ROS2 - **humble**, gazebo - **harmonic**.
> Это не стандартное, но поддерживаемое сочетание версий. Так нужно для работы в виртуалке, плюс "официальный" релиз gazebo для ros2 harmonic уже довольно старый. Если будут любые проблемы - сразу переходим на нативный линукс и gazebo fortress.

<details>
<summary>О настройке VM</summary>

Проблемы с другими вариантами:

- **Нативно на MacOS**

Я перепробовал кучу вариантов, и собрать ros2 на маке, и поставить gazebo из brew, и robostack + gazebo (и из конды, и из brew), и докер, но ничего не работало удовлетворительно. Основной затык на маке - версия gazebo, работающая с humble *и при этом работающая на маке (garden и дальше)* не поддерживается официально для humble. Для линукса есть бинарные пакеты, для мака ros_gz надо собирать руками. Это, как и сборку самого ros2, можно сделать, но тут уже получается такой наворот кастомного окружения, что найти какую-либо помощь в интернете потом будет нереально.

- **Docker**

Прокинуть туда GPU - суровое приключение, плюс novnc все таки вносит заметные тормоза.

------

Соответственно, остается только виртуалка. Пробовал parallels - прокинуть gpu можно, но parallels-tools в ubuntu 22.04 ставятся только если понизить ядро до 5.17. Это я тоже сделал, но добиться приемлемой производительности не получилось. Во fusion же все круто заработало, но только через `--render-engine ogre`. `ogre2` - черный экран в gazebo, но это известная проблема (есть тикеты на гитхабе) которую, похоже, фиксить не будут (и обновлением драйверов не решается). Тем не менее, просто `ogre` - официально поддерживаемое и рекомендуемое решение, так что тут получилось все сделать без кастома и работает быстро (60 fps с vsync).

**ИТОГО**:
```
VMware Fusion
Ubuntu 22.04 (у меня xubuntu)
ROS2 Humble
Gazebo Harmonic
ros-humble-ros-gzharmonic

gz sim --render-engine ogre ...
```

</details>

Установка связки ros2-gazebo: `sudo apt install ros-humble-ros-gzharmonic` если вы ставили gazebo из репозиториев `osrfoundation`.

## Инструменты для разработки

Много для чего нужна Node (vscode, zx и пр.). Ставим LTS версию: [инструкция](https://github.com/nodesource/distributions#using-ubuntu-3)

- **jsonnet**: шаблонизатор
- **zx**: позволяет писать нормальные bash-скрипты.
- **just**: запускалка команд, типо vscode tasks но для командной строки. Ставим [отсюда](https://github.com/casey/just#packages) через prebuilt-mcr (предпоследняя строка)
- **xacro**: стандартная ros-тулза для работы с xml (ros-humble-xacro)
- **colcon**: сборщик ros-пакетов (`sudo apt install python3-colcon-common-extensions`)
- **pip**: само собой. `sudo apt install python3-pip`

И добавьте `export PATH=$PATH:/home/igor/.local/bin` в `.bashrc`.
