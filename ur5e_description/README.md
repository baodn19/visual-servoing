# Notes for me
- Run launch file:
```
ros2 launch ur5e_description ur5e_sim.launch.py
```

- View camera feed: In the top-left dropdown menu, select `/camera/image_raw`
```
ros2 run rqt_image_view rqt_image_view
```

- Only edit `ur5e.xacro` if URDF needs to be modified.