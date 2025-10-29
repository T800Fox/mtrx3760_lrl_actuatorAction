# Actuator Action Node
## Interface
### Modes
- MODE_ABS_LINEAR = 0
- MODE_ABS_ANGULAR = 1
- MODE_VEL_LINEAR = 2
- MODE_VEL_ANGULAR = 3
### Goal Message
- uint8 mode
- float64 magnitude
## Setup
1. Bringup Turtlebot3 actuators; `ros2 launch turtlebot3_bringup robot.launch.py`
2. Launch Slamtoolbox Online Async (TF needs, might be able to go without if serving a map?); `ros2 launch mtrx3760_lrl_warehousebot online_async_launch.py`
3. Spin up the Actuator Action Server Node; `ros2 run mtrx3760_lrl_warehousebot actuator_action_server`
4. Commands can be sent w/ the Actuator Debug Client Node, or via the command line (replace '#'); `ros2 action send_goal /actuator lrl_action_interface/action/Test "{mode: # , magnitude: #}"`