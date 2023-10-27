# Fusion Search Zone Tracking

## detect

Using YOLOv8-ros to detect target in Gazebo  simulation environment.

```bash
cd your_workspace
source devel/setup.bash
roslaunch yolov8_ros v8.launch
```

## track

Utilizing the Kalman filter to predict the target's position in the world frame, and project the prediction onto the image frame to generate a new search area. This new search area is then fused with the native search area of CSRT.

```bash
roslaunch projection fst.launch
rosbag play your_ros_bag
```

## instruction of result

https://github.com/fbh16/sort_csrt/assets/87756936/19b6681d-c3a9-4b97-a555-bff8d5262bf2

The purple point in Rviz represents the Kalman Filter's prediction in the world frame, while the green bounding box in the OpenCV window indicates the tracking results of CSRT.

# CSRT based on SORT framework

## Evaluate

### Evaluation of small target detection model

| *Sequence name* | *Class*    | *Images* | *Labels* | *P*   | *R*   | *mAP@.5* | *mAP@.5：.95* |
| --------------- | ---------- | -------- | -------- | ----- | ----- | -------- | ------------- |
| Car1            | car        | 505      | 516      | 0.981 | 0.999 | 0.960    | 0.906         |
| Car2            | car        | 565      | 614      | 0.993 | 0.995 | 0.956    | 0.543         |
| gym             | pedestrian | 300      | 798      | 1     | 1     | 0.995    | 0.995         |
| river           | pedestrian | 275      | 825      | 1     | 1     | 0.995    | 0.995         |
| Grass           | pedestrian | 319      | 319      | 1     | 1     | 0.995    | 0.995         |
| War1            | pedestrian | 169      | 244      | 0.984 | 1     | 0.995    | 0.847         |
| War2            | pedestrian | 107      | 250      | 0.999 | 0.996 | 0.995    | 0.854         |
| War3            | pedestrian | 156      | 314      | 1     | 0.997 | 0.995    | 0.739         |
| All             | All        | 2396     | 3880     | 0.995 | 1     | 0.97     | 0.859         |

### Performance Evaluation of SORT Tracking

The evaluation of the SORT tracker was conducted on eight sequences. These eight sequences collectively contained 5,625 target detections, of which 5,059 were correctly detected. During the tracking process across the eight sequences, ID switches occurred a total of 39 times, resulting in an average tracking accuracy of 87.182%.

| *Sequence name* | *MOTA* | *IDF1* | *MOTP* | *HOTA* | *MT* | *ML* | *IDSW* | *AssA* |
| --------------- | ------ | ------ | ------ | ------ | ---- | ---- | ------ | ------ |
| Car1            | 99.814 | 99.907 | 91.817 | 92.593 | 1    | 0    | 0      | 92.509 |
| Car2            | 92.599 | 85.204 | 76.999 | 69.399 | 1    | 0    | 1      | 55.684 |
| Grass           | 51.274 | 20.747 | 93.576 | 23.462 | 0    | 0    | 7      | 10.806 |
| Gym             | 65.693 | 75.211 | 85.066 | 60.464 | 2    | 2    | 6      | 65.445 |
| River           | 96.97  | 98.471 | 71.154 | 68.386 | 3    | 0    | 0      | 69.387 |
| War1            | 93.391 | 96.695 | 63.227 | 60.56  | 2    | 0    | 0      | 60.763 |
| War2            | 97.808 | 97.408 | 77.879 | 75.031 | 3    | 0    | 1      | 74.787 |
| War3            | 96.474 | 98.237 | 90.317 | 88.309 | 2    | 0    | 0      | 89.535 |
| All             | 87.182 | 84.373 | 79.438 | 71.037 | 14   | 2    | 15     | 67.425 |

### Evaluation of multi-CSRT tracking

The evaluation of the CSRT tracker was conducted on eight sequences. These eight sequences collectively contained 5,625 target detections, of which 5,542 were correctly detected. During the tracking process across the eight sequences, ID switches occurred a total of 68 times, resulting in an average tracking accuracy of 89.209%.

| *Sequence name* | *MOTA* | *IDF1* | *MOTP* | *HOTA* | *MT* | *ML* | *IDSW* | *AssA* |
| --------------- | ------ | ------ | ------ | ------ | ---- | ---- | ------ | ------ |
| Car1            | 99.443 | 99.722 | 90.887 | 91.106 | 1    | 0    | 0      | 91.356 |
| Car2            | 93.092 | 91.058 | 75.477 | 67.973 | 2    | 0    | 3      | 74.1   |
| Grass           | 80.573 | 89.242 | 98.832 | 79.765 | 1    | 0    | 0      | 80.087 |
| Gym             | 75.042 | 57.652 | 92.295 | 57.449 | 3    | 0    | 25     | 79     |
| River           | 92.848 | 96.418 | 70.372 | 67.374 | 3    | 0    | 0      | 70.284 |
| War1            | 93.103 | 96.552 | 63.347 | 60.772 | 2    | 0    | 0      | 63.286 |
| War2            | 91.352 | 93.959 | 76.566 | 70.898 | 3    | 0    | 8      | 77.62  |
| War3            | 96.154 | 78.662 | 95.285 | 80.101 | 2    | 0    | 2      | 94.965 |
| All             | 89.209 | 85.448 | 81.546 | 71.648 | 17   | 0    | 38     | 78.224 |

### Evaluation of multi-KCF tracking

The evaluation of the KCF tracker was conducted on eight sequences. These eight sequences collectively contained 5,625 target detections, of which 4,763 were correctly detected. During the tracking process across the eight sequences, ID switches occurred a total of 54 times, resulting in an average tracking accuracy of 78.169%.

| *Sequence name* | *MOTA* | *IDF1* | *MOTP* | *HOTA* | *MT* | *ML* | *IDSW* | *AssA* |
| --------------- | ------ | ------ | ------ | ------ | ---- | ---- | ------ | ------ |
| Car1            | 99.443 | 99.722 | 90.887 | 91.106 | 1    | 0    | 0      | 91.356 |
| Car2            | 90.572 | 91.625 | 90.231 | 91.985 | 1    | 0    | 0      | 90.876 |
| Grass           | 80.573 | 89.242 | 98.835 | 76.713 | 1    | 0    | 0      | 76.713 |
| Gym             | 73.205 | 52.197 | 91.642 | 51.376 | 2    | 0    | 28     | 38.362 |
| River           | 92.848 | 96.418 | 70.385 | 67.384 | 3    | 0    | 0      | 68.338 |
| War1            | 93.103 | 96.552 | 63.347 | 60.772 | 2    | 0    | 0      | 60.913 |
| War2            | 87.089 | 93.144 | 77.437 | 67.9   | 2    | 0    | 0      | 68.567 |
| War3            | 96.474 | 98.237 | 95.306 | 91.666 | 2    | 0    | 0      | 94.941 |
| All             | 78.169 | 80.689 | 82.289 | 67.792 | 13   | 2    | 28     | 74.264 |

### Evaluation of CSRT based on SORT framework

The evaluation of CSRT Tracker based-on SORT framework was conducted on 9 sequences, with a total of 6,468 detections. 6,012 of these were correctly detected, with a total of 36 changes in ID. resulting in an average tracking accuracy of 75.387%.

| *Sequence name* | *MOTA* | *IDF1* | *MOTP* | *HOTA* | *MT* | *ML* | *IDSW* | *AssA* |
| --------------- | ------ | ------ | ------ | ------ | ---- | ---- | ------ | ------ |
| Car1            | 74.76  | 82.847 | 74.579 | 61.886 | 2    | 2    | 3      | 67.775 |
| Car2            | 99.443 | 99.722 | 72.495 | 71.116 | 1    | 0    | 0      | 71.245 |
| Car3            | 41.941 | 71.306 | 73.728 | 57.396 | 1    | 1    | 2      | 62.804 |
| Grass           | 60.828 | 75.93  | 78.165 | 47.351 | 0    | 0    | 0      | 47.351 |
| Gym             | 38.022 | 49.039 | 70.429 | 41.691 | 2    | 20   | 3      | 39.409 |
| River           | 97.818 | 98.9   | 75.342 | 73.108 | 3    | 0    | 0      | 74.107 |
| War1            | 88.793 | 94.388 | 59.963 | 57.047 | 2    | 0    | 0      | 58.006 |
| War2            | 95.737 | 97.212 | 73.091 | 69.767 | 3    | 0    | 1      | 70.384 |
| War3            | 90.064 | 74.98  | 64.65  | 60.16  | 2    | 0    | 1      | 46.506 |
| All             | 75.387 | 81.987 | 70.99  | 57.995 | 16   | 23   | 10     | 63.747 |
