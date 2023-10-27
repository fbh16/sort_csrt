# 启动无人机斜向前向后飞行节点

```bash
python3 drone_circular_move.py
```

# 启动小车做圆周运动节点

```bash
python3 car_circular_drive.py
```

# 生成自定义规格的标定板

```bash
# generate_checkerboard.py rows columns [square_size_in_meters]
e.g.
python3 generate_checkerboard.py 7 9 0.108
```

# 将标定板加载至仿真环境中

```bash
roslaunch [rospkg] outdoor_flight_gazebo.launch
```

