# gazebo_sfm_plugin
A plugin for simulation of human pedestrians in ROS2 and Gazebo.

**Tested in ROS2 Galactic and Gazebo 11** 

The persons are affected by the obstacles and other persons using the [Social Force Model](https://github.com/robotics-upo/lightsfm)


![](https://github.com/robotics-upo/gazebo_sfm_plugin/blob/master/media/images/capture3.jpg)


## Plugin configuration

The plugin can be applied to each Gazebo Actor indicated in the Gazebo world file.

An example snippet is shown next:

```html
<actor name="actor1">
	<pose>-1 2 1.25 0 0 0</pose>
	<skin>
		<filename>walk.dae</filename>
		<scale>1.0</scale>
	</skin>
	<animation name="walking">
		<filename>walk.dae</filename>
		<scale>1.000000</scale>
		<interpolate_x>true</interpolate_x>
	</animation>
	<!-- plugin definition -->
	<plugin name="actor1_plugin" filename="libPedestrianSFMPlugin.so">
		<velocity>0.9</velocity>
		<radius>0.4</radius>
		<animation_factor>5.1</animation_factor>
		<people_distance>6.0</people_distance>
		<!-- weights -->
		<goal_weight>2.0</goal_weight>
		<obstacle_weight>80.0</obstacle_weight>
		<social_weight>15</social_weight>
		<group_gaze_weight>3.0</group_gaze_weight>
		<group_coh_weight>2.0</group_coh_weight>
		<group_rep_weight>1.0</group_rep_weight>
		<ignore_obstacles>
			<model>cafe</model>
			<model>ground_plane</model>
		</ignore_obstacles>
		<trajectory>
			<cyclic>true</cyclic>
			<waypoint>-1 2 1.25</waypoint>
			<waypoint>-1 -8 1.25</waypoint>
		</trajectory>
	</plugin>
</actor>
```
The parameters that can be configured for each pedestrian are:

### General params

*  ```<velocity>```. Maximum velocity (*m/s*) of the pedestrian.
*  ```<radius>```. Approximate radius of the pedestrian's body (m).
*  ```<animation_factor>```. Factor employed to coordinate the animation with the walking velocity.
* ```<people_distance>```.  Maximum detection distance of the surrounding pedestrians.

### SFM Weights

*  The weight factors that modify the navigation behavior. See the [Social Force Model](https://github.com/robotics-upo/lightsfm) for further information.

### Obstacle params

* ```<ignore_obstacles>```.  All the models that must be ignored as obstacles, must be indicated here. The other actors in the world are included automatically.

### Trajectory params

* ```<trajectory>```. The list of waypoints that the actor must reach must be indicated here. 

	- ```<waypoint>```. Each waypoint must be indicated by its coordinates X, Y, Z in the world frame.
	- ```<cyclic>```. If true, the actor will start the waypoint sequence when the last waypoint is reached.

## Dependencies

* You must download and install the Social Force Model library, lightsfm https://github.com/robotics-upo/lightsfm

## Compilation

* This is a ROS2 package so it must be placed inside a ROS2 workspace and compiled through the regular colcon compiler. 
```sh
colcon build --packages-select gazebo_sfm_plugin
```

## Example

An example Gazebo world can be launched through:
```sh
ros2 launch gazebo_sfm_plugin cafe_ros2.launch.py
```

# Motion prediction changes

In order to use this plugin to predict the future poses of each agent, some modifications to the original source code were made. 

**Note:** A new instance of PedestrianSFMPlugin is created for each actor with sfmPlugin specified in the world file. (This can help to understand the way this sfm plugin works).
## ROS2

A ROS2 node was created to access all the functionalities of ROS2. 

The following ROS2 components were created:

* **publish_future_poses_timer_**: Timer to trigger calculation or future poses and publication of paths. This timer is executed at a rate in Hz specified by the tag *<publish_rate>*. This tag was added to the existing plugin of each actor in the world file. 
* **PathPublisher_:** Publisher created for each actor. Publishes the path of future poses to the topic */agent_path_{name}*, where *name* corresponds to the actor model name . Message type: *nav_msgs::msg::Path*.


## Next poses calculation

The calculation of the future poses for each actor is done in the function *Calculate_path_timerCallback*, which is a callback of the timer *publish_future_poses_timer_*. 

To perform the calculations without affecting the real model in gazebo, a copy of *sfmActor* is created. The attribute *groupId* is set to -1 to specify that the copy does not belong to any group. 

After this, a message called *path* of type *nav_msgs::msg::Path* is created, the frame_id is set to *base_link* and the time stamp is added. A message called *next_pose* of type geometry_msgs::msg::PoseStamped is also created.

To calculate the future poses, the functions *computeForces* and *updatePosition* are called using the copy of *sfmActor* and the resulting position and orientation is saved in *next_pose* and added to *path*. This procedure is done n times, where n corresponds to the number of iterations (calculated using *look_ahead_time* and *prediction_time_step* parameters) as follows:


$$iterations = \frac{\text{look ahead time}}{\text{prediction time step}}$$




## Optimize handle functions

When loading a world with many models, the whole system performance and the *real time factor* of the simulation in gazebo were pretty low. To improve this, the functions *HandleObstacles* and *HandlePedestrians* were optimized.

Both functions are called every time when the function *OnUpdate* is executed. The latter is used to update the models in gazebo and is called at a very high rate, that means, the fewer calculations performed on *HandleObstacles* and *HandlePedestrians*, the faster the *real time factor* in gazebo will be.

In the original code, all models belonging to the world were checked to determine if they were pedestrians or obstacles. To optimize this, a tag called *<include_models>* was added to each actor plugin in the world file. In this tag, all the models (obstacles or pedestrians) to be taken into account are included. This information is then used in the constructor of each actor to fill the vectors *include_obstacles_indexes* and *include_pedestrians_indexes* with the indexes of the required models. 

With this, we only need to traverse once the whole array of models in the world for each actor and then use only the elements with indexes specified by *include_obstacles_indexes* and *include_pedestrians_indexes* to perform the calculations in the functions *HandleObstacles* and *HandlePedestrians*. 

**Note:** In the tag *<include_models>*, if model name has a * at the end, it is a wildcard and all models beginning with that name will be included.


## Added tags in plugin 

Here is a brief summary of the added tags in the world file for each actor plugin.

* **<look_ahead_time>**: Time in the future to calculate future poses
* **<prediction_time_step>**: time step to calculate future poses using copy of sfmActor
* **<publish_rate>**: Rate in Hz to publish path 
* **<include_models>**: Models to include as obstacles or pedestrians. If model name has a * at the end, it is a wildcard and all models beginning with that name will be included.

Below can be seen a fragment of the world file with the plugin for an actor including the added tags. 

```html
      <plugin name="actor1_plugin" filename="libPedestrianSFMPlugin.so">
        <look_ahead_time>5.0</look_ahead_time>
        <prediction_time_step>0.2</prediction_time_step>
        <publish_rate>1</publish_rate>
        <velocity>1.2</velocity>
        <radius>0.4</radius>
        <animation_factor>5.1</animation_factor>
        <animation_name>walking</animation_name>
        <people_distance>6.0</people_distance>
        <!-- weights -->
        <goal_weight>2.0</goal_weight>
        <obstacle_weight>80.0</obstacle_weight> <!--10.0-->
        <social_weight>15</social_weight> <!--2.1-->
        <group_gaze_weight>3.0</group_gaze_weight>
        <group_coh_weight>2.0</group_coh_weight>
        <group_rep_weight>1.0</group_rep_weight>
        <include_models>       
          <model>kiwi</model>
          <model>actor*</model>
        </include_models>
        <trajectory>
          <cyclic>true</cyclic>
          <waypoint>3 2 1.25</waypoint>
          <waypoint>3 -8 1.25</waypoint>
          <waypoint>-1 -8 1.25</waypoint>
          <waypoint>-1 2 1.25</waypoint>          
        </trajectory>
      </plugin>
    </actor>
```

**Note:** Tag *<ignore_obstacles>* is not included since it is no longer used.
## Visualization

Finally, rviz2 was used to visualize the published paths of the future poses for each actor. The name of the topic is */agent_path_{name}*, where *name* corresponds to the actor model name. Frame_id was of the header for each published path was set to *base_link*. The figure below shows the paths of 4 different actors. 

![rviz2](https://user-images.githubusercontent.com/102924128/210798948-c0a717e2-5bb8-46cb-8fda-348b0109de66.png)



Check out the result! 

https://user-images.githubusercontent.com/102924128/210597434-9655f2f3-bc5b-4dc5-98e3-5027f6b4c319.mp4

In case of questions, comments or suggestions, feel free to contact one of the collaborators of this fork. We like to collaborate C:





