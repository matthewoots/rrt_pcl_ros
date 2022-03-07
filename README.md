# RRT ON POINTCLOUD FOR ROS
Development of RRT on ROS using `sensor_msgs::PointCloud2` received by published topic. 

```
# Use this to save pcl2 topic
rosrun pcl_ros pointcloud_to_pcd input:=/ <pcl_topic_name> _prefix:=/home/<user>/<arbitrary prefix>_

# Use this to publish pcl2 topic 
rosrun pcl_roscd_to_pointcloud <pcl_file_name>.pcd 0.1 _frame_id:=/map
```

Using custom generalized `Bspline` used in https://github.com/matthewoots/px4-path-planner to generate a b-spline path within the RRT

---

### Current Params for RRT
These are the current parameters for tuning the RRT, sometimes the search does not give a good result if the **boundaries are too small** and the **threshold is too large**.

There are several working parameters for **single agent** and for **oblate ellipsoids**.

The current working parameters are as shown below
```yaml
string _file_location change in launch
The rest of the parameters change in yaml
# RRT Parameters
start_delay: 3.0 # Represents the time for mockamap to startup
step_size: 1.5 # Represents node step size when doing RRT
obs_threshold: 0.9 # Acceptable distance from the threshold
random_multiplier: 1.0 # Random multiplier to push start and end points out of obstacles
line_search_division: 4 # Split the line search portion for any collision

xybuffer: 4.0 # XY buffer for cropping of pointcloud
zbuffer: 4.0 # Z buffer for cropping of pointcloud 
passage_size: 4.5 # In this case it is a Y buffer, since its suppose to be align to X

min_height: 1.0 # Minimum height (will clamp)
max_height: 5.0 # Maximum height (will clamp)

## Bspline Parameters
bs_order: 6

## Box Constrain Parameters
max_boundaries: 0.7 # Bounding box maximum constrain
corridor_size: 0.6 # This affect cropping about the Y
division: 4 # Same as the line search for collision
safety_radius: 0.5 # Safety radius of the agent
```