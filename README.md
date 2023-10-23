# PointCloud2 Merge
### Package that can be used to merge **2-8** sensor_msgs/PointCloud2 
Usage :
```bash
roslaunch pointcloud2_merge pointcloud2_merge.launch
```

Args:
```xml
<arg name="input_topics" default="[/camera1/depth/color/points, /camera2/depth/color/points]" />
<arg name="output_topic" default="/points_concat" />
<arg name="output_frame_id" default="base_link" />
```
`input_topics`: Arrya comma seperated 2 to 8 topics 
`output_topic`: Output topic name for the super pointcloud2
`output_frame_id`: The frame of refrence that you want the point cloud to be published in.

### Note:
This package considers the frame id in the header of all the input pointclouds and the then with refrence converts and combines in the output frame id. Hence make sure that your frame id is correct.
