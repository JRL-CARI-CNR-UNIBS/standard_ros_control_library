# robust inverse dynamics control

required [rosdyn](https://github.com/JRL-CARI-CNR-UNIBS/robot_control_algorithms)

required parameters:
``` yaml
mimic_controller:
  type: robot_control/MimicController
  joint_names:
  - jnt1
  - jnt2
  - jnt3
  
  setpoint_topic_name: "/manipulator/joint_target"
  natural_frequency: 20   # natural frequency of the closed loop
  damping: 1              # relative damping of the closed loop
  robustness_gain: 0.01   # robustness gain

```
