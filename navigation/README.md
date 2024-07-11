# OrcaRL2

```bash

https://osrf.github.io/ros2multirobotbook/intro.html

https://github.com/open-rmf/rmf_demos

https://open-rmf.github.io/rmf-panel-js/

https://github.com/open-rmf/rmf_demos/tree/foxy/rmf_demos_panel/task_lists

ros2 run rmf_demos_tasks dispatch_patrol -p coe lounge -n 3 --use_sim_time

ros2 run rmf_demos_tasks dispatch_delivery -p pantry -ph coke_dispenser -d hardware_2 -dh coke_ingestor --use_sim_time

ros2 run rmf_demos_tasks dispatch_loop -s coe -f lounge -n 3 --use_sim_time
ros2 run rmf_demos_tasks dispatch_delivery -p pantry -pd coke_dispenser -d hardware_2 -di coke_ingestor --use_sim_time




[
{"task_type":"Loop", "start_time":0, "priority":0, "description": {"num_loops":5, "start_name":"coe", "finish_name":"lounge"}},
{"task_type":"Delivery", "start_time":0, "priority":0, "description": {"option": "coke"}},
{"task_type":"Loop", "start_time":0, "priority":0, "description": {"num_loops":5, "start_name":"pantry", "finish_name":"supplies"}}
]

```