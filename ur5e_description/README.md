# Notes for me
- Run launch file:
```
ros2 launch ur5e_description ur5e_sim.launch.py
```

- View camera feed: In the top-left dropdown menu, select `/camera/image_raw`
```
ros2 run rqt_image_view rqt_image_view
```

- Send a move command:
```
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'], points: [{positions: [1.57, -1.57, 1.57, 0.0, 0.0, 0.0], time_from_start: {sec: 2, nanosec: 0}}]}" -1
```

- Only edit `ur5e.xacro` if URDF needs to be modified.
