# Платформа на аэроподшипниках

Приложить силу от пропеллера: `gz topic -t "/model/platform/joint/prop_left_joint/cmd_thrust" -m gz.msgs.Double  -p "data: 0.1"`.

Сккорость пропеллера: `gz topic -t "/platform/gazebo/command/motor_speed/0" -m gz.msgs.Actuators  -p "velocity: 0.1"`.
