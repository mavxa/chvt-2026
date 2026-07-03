# ЧВТ 2026

Вариант задания:

- старт: ArUco ID `0`;
- цель: ArUco ID `33`;
- препятствие: появляется после достижения цели на ArUco ID `21`;
- координаты препятствия: `x=-3.0 y=3.0`.

## Запуск

Терминал 1, симулятор:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch ar_webots_fms_ros2 module2.launch.py
```

Терминал 2, алгоритм:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/zed/chvt-2026
python3 main.py
```

Скрипт сам запускает препятствие после достижения цели командой:

```bash
ros2 launch ar_webots_fms_ros2 spawn_object.launch.py x:=-3.0 y:=3.0 angle_z:=0.0 object_name:=obstacle
```

## Что делает алгоритм

1. Ждет стартовую ArUco-метку из `/RMC2/aruco_id`.
2. Строит граф сетки ArUco-маркеров.
3. Находит кратчайший маршрут до ID `33` алгоритмом BFS.
4. Печатает маршрут в терминал.
5. Едет по маршруту, последовательно проверяя достижение маркеров по `/RMC2/aruco_id`.
6. После достижения ID `33` запускает spawn препятствия на ID `21`.
7. Строит обратный маршрут до старта, исключая ID `21`.
8. Возвращается на старт.
9. Пишет лог в `reports/mission_*.log`.

## Полезные команды проверки

```bash
ros2 topic echo /RMC2/aruco_id
ros2 topic echo /RMC2/cmd_vel
ros2 topic echo /RMC2/scan
```

Ручной аварийный стоп:

```bash
ros2 topic pub /chvt/emergency_stop std_msgs/msg/Bool "{data: true}" --once
```

Снять стоп:

```bash
ros2 topic pub /chvt/emergency_stop std_msgs/msg/Bool "{data: false}" --once
```
