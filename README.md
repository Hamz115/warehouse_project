### For the Simulation:

#### ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diffbot_base_controller/cmd_vel_unstamped

#### cd ~/ros2_ws && source install/setup.bash
#### ros2 launch localization_server localization.launch.py map_file:=warehouse_map_sim.yaml

#### cd ~/ros2_ws && source install/setup.bash
#### ros2 launch path_planner_server pathplanner.launch.py mode:=sim

#### python3 ~/ros2_ws/src/warehouse_project/nav2_apps/scripts/sim_move_shelf_to_ship.py

### For the Real Robot:

#### ros2 run teleop_twist_keyboard teleop_twist_keyboard

#### cd ~/ros2_ws && source install/setup.bash
#### ros2 launch localization_server localization.launch.py map_file:=warehouse_map_real.yaml

#### cd ~/ros2_ws && source install/setup.bash
#### ros2 launch path_planner_server pathplanner.launch.py mode:=real

#### python3 ~/ros2_ws/src/warehouse_project/nav2_apps/scripts/move_shelf_to_ship.py
