# iMap

Map representation library for duckietown intersections 

<img src="documentation/imap_example.png" alt="" style="width: 200px;"/>

## Conventions

The iMap library implements four kinds of intersection, the 4-way intersection (which is symmetric to rotation) and the three possible 3-way intersections (from "duckiebot's perspective": left-straight (3SL), left-right (3LR), straight-right (3SR)). All coordinate systems are implemented such that the duckiebot starts in the upper half of the image, therefore the coordinate system is introduced in the middle of the upper lane (see figure). 

## Visualization

iMap contains visualization functionalities for a planned trajectory, the duckiebot's pose and further points to visualize (e.g. keypoints for localization) and publishes the resulting graphic as sensor_msgs::Img (viewable e.g in rqt_image_view). 

## Demo

```
roslaunch imap test_visualization.launch
```
