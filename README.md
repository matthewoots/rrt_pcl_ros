# RRT ON POINTCLOUD FOR ROS
Development of RRT on ROS using `sensor_msgs::PointCloud2` received by published topic. 
Using custom generalized `Bspline` used in https://github.com/matthewoots/px4-path-planner to generate a b-spline path within the RRT

---

### Dependencies
Rely on https://github.com/matthewoots/swarm_param_publisher to start running, **swarm_param** node issues the data needed by this module.
```
# First clone swarm publisher into your workspace/src
git clone https://github.com/matthewoots/swarm_param_publisher 

# Then after compiling
roslaunch swarm_param sample.launch
# This will send rrt_pcl_ros with 2 sets of rrt params and also with the pointcloud data
```

```
# Use this to save pcl2 topic
rosrun pcl_ros pointcloud_to_pcd input:=/ <pcl_topic_name> _prefix:=/home/<user>/<arbitrary prefix>_

# Use this to publish pcl2 topic 
rosrun pcl_roscd_to_pointcloud <pcl_file_name>.pcd 0.1 _frame_id:=/map
```

---

### Current Params for RRT
These are the current parameters for tuning the RRT, sometimes the search does not give a good result if the **boundaries are too small** and the **threshold is too large**.

There are several working parameters for **single agent** and for **oblate ellipsoids** that you can refer to https://github.com/matthewoots/swarm_param_publisher.

The current working parameters are as shown below
```yaml
is_using_formation: true
is_circle_formation: false
runtime_error: 10.0
circle_radius: 2.5
number_of_runs: 10

start_position: [0.0, 0.0, 1.5]
end_position: [13.0, 0.0, 1.5]
```