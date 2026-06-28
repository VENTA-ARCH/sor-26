[//]: # (Image References)


[image2]: ./assets/addingcam.png "Adding a camera"
[image3]: ./assets/addingcam2.png "Adding a camera"
[image4]: ./assets/cam_rqt.png "Adding a camera"
[image5]: ./assets/compressed_rqt.png "Adding a camera"
[image6]: ./assets/compressed_rviz.png "Adding a camera"
[image7]: ./assets/rqt_reconfigure.png "rqt reconfigure"
[image8]: ./assets/lidar.png "lidar"
[image9]: ./assets/visualize_lidar.png "lidar"
[image10]: ./assets/decay.png "lidar"


# WinteROS Week 2
# Part-2 : GAZEBO SENSORS
 <!--
## This is how far we will get by the end of this lesson: 
  <a href="https://youtu.be/0Xokl5dHRoQ"><img width="600" src="./assets/youtube-gazebo.png"></a>  

  <a href="https://youtu.be/ELwRqeNR_NA"><img width="600" src="./assets/youtube-gazebo-1.png"></a>  
-->

In the first part of this week, you built a simulated robot that you can drive around manually. Now, it's time to verify that our robot can actually **sense** the world.

A robot that cannot sense its environment is just a remote-controlled car. In this lesson, we will equip our robot with two critical sensors:

- **Camera (Visual Perception):** To see the world in color.  
- **LiDAR (Spatial Perception):** To detect obstacles and measure distances.

We will focus on the **integration pipeline**:

- Adding the physical links in **URDF**
- Configuring the **Gazebo plugins**
- Establishing the **ROS 2 Bridge**

By the end of this module, you will be able to visualize what the robot sees in **RViz2** in real-time.

## Download ROS package

To download the starter package clone the following git repo  to your colcon workspace:
```bash
git clone https://github.com/ysdesha/winteros_week2_sensors.git
```
## Test the starter package

After we download the package from GitHub, let's rebuild the workspace and source the `install/setup.bash` file to make sure ROS and its tools are aware about the new package.

Then we can test the package with the usual launch file:
```bash
ros2 launch erc_gazebo_sensors spawn_robot.launch.py
```

And we can also start a teleop node:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

# Camera

To add a camera - and every other sensors later - we have to change 2 files:
1) The `erc_bot.urdf`: we have to define the position, orientation and other physical properties of the camera in this file. This is not necessarily simulation dependent, we have to do these same changes in the urdf in case of a real robot with a real sensor.
2) The `erc_bot.gazebo`: this is fully simulation dependent, we have to define the properties of the simulated camera in this file.

Let's add the camera first to the `erc_bot.urdf`:

```xml
  <!-- STEP 7 - Camera -->
  <joint type="fixed" name="camera_joint">
    <origin xyz="0.225 0 0.075" rpy="0 0 0"/>
    <child link="camera_link"/>
    <parent link="base_link"/>
    <axis xyz="0 1 0" />
  </joint>

  <link name='camera_link'>
    <pose>0 0 0 0 0 0</pose>
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia
          ixx="1e-6" ixy="0" ixz="0"
          iyy="1e-6" iyz="0"
          izz="1e-6"
      />
    </inertial>

    <collision name='collision'>
      <origin xyz="0 0 0" rpy="0 0 0"/> 
      <geometry>
        <box size=".03 .03 .03"/>
      </geometry>
    </collision>

    <visual name='camera_link_visual'>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size=".03 .03 .03"/>
      </geometry>
    </visual>

  </link>

  <gazebo reference="camera_link">
    <material>Gazebo/Red</material>
  </gazebo>

  <joint type="fixed" name="camera_optical_joint">
    <origin xyz="0 0 0" rpy="-1.5707 0 -1.5707"/>
    <child link="camera_link_optical"/>
    <parent link="camera_link"/>
  </joint>

  <link name="camera_link_optical">
  </link>
```

As we see above, the camera is a 3 x 3 x 3 cm cube, attached to the `base_link` with a fixed joint. But there is another thing, `camera_link_optical` which is connected to the `camera_link` through the `camera_optical_joint`! The purpose of this additional link and joint to solve the conflict between 2 different conventional coordinate systems:
- By default, URDF uses the right-handed coordinate system with X forward, Y left, and Z up.
- However, many ROS drivers and vision processing pipelines expect a camera’s optical axis to be aligned with Z forward, X to the right, and Y down.

`camera_optical_joint` applies a static rotation so that the camera data will be interpreted correctly by ROS tools that assume the Z-forward convention for image and depth sensors.

Now let's add the simulated camera into `erc_bot.gazebo`:

```xml
  <gazebo reference="camera_link">
    <sensor name="camera" type="camera">
      <camera>
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>15</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <!-- Noise is sampled independently per pixel on each frame.
               That pixel's noise value is added to each of its color
               channels, which at that point lie in the range [0,1]. -->
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
        <optical_frame_id>camera_link_optical</optical_frame_id>
        <camera_info_topic>camera/camera_info</camera_info_topic>
      </camera>
      <always_on>1</always_on>
      <update_rate>20</update_rate>
      <visualize>true</visualize>
      <topic>camera/image</topic>
    </sensor>
  </gazebo>
```

> Don't forget that the above code snippets must be placed within the already existing `<robot>` tag!

With the above plugin we define a couple of things for Gazebo, let's see the important ones one by one:
- `<gazebo reference="camera_link">`, we have to refer to the `camera_link` that we defined in the `urdf`
- `<horizontal_fov>1.3962634</horizontal_fov>`, the field of view of the simulated camera
- `width`, `height`, `format` and `update_rate`, properties of the video stream
- `<optical_frame_id>camera_link_optical</optical_frame_id>`, we have to use the `camera_link_optical` that we checked in details above to ensure the right static transformations between the coordinate systems
- `<camera_info_topic>camera/camera_info</camera_info_topic>`, certain tools like rviz requires a `camera_info` topic that describes the physical properties of the camera. The topic's name must match camera's topic (in this case both are `camera/...`)
- `<topic>camera/image</topic>`, we define the camera topic here

We can rebuild the workspace and try our changes, but it will not yet work. The camera's red cube model is visible but the topics aren't available for ROS (we can check it for example with `rqt`)

> It's possible to reload the urdf without restarting the nodes by setting the parameter from the terminal:
> ```bash
> ros2 param set /robot_state_publisher robot_description "$(xacro $(ros2 pkg prefix erc_gazebo_sensors)/share/erc_gazebo_sensors/urdf/erc_bot.urdf)"
> ```

![alt text][image2]

With the new Gazebo simulator topics are not automatically forwarded as we already saw it in  previous lesson, we have to use the `parameter_bridge` of the `ros_gz_bridge` package. It has [a very detailed readme](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge) what kind of topic types can be forwarded between ROS and Gazebo. We have to extend the arguments of the `parameter_bridge` in our launch file:

```python
    # Node to bridge /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            "/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",

        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
```

Let's rebuild the workspace and try it again:

```bash
ros2 launch erc_gazebo_sensors spawn_robot.launch.py
```

![alt text][image3]
![alt text][image4]

## Image transport

We can see that both `/camera/camera_info` and `/camera/image` topics are forwarded. Although this is still not the ideal way to forward the camera image from Gazebo. ROS has a very handy feature with it's image transport protocol plugins, it's able to automatically compress the video stream in the background without any additional work on our side. But this feature doesn't work together with `parameter_bridge`. Without compression the 640x480 camera stream consumes almost 20 MB/s network bandwidth which is unacceptable for a wireless mobile robot.

Therefore there is a dedicated `image_bridge` node in the `ros_gz_image` package. Let's modify our launch file to the following:

```python
    # Node to bridge /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            #"/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",

        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # Node to bridge camera image with image_transport and compressed_image_transport
    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/camera/image",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time'),
             'camera.image.compressed.jpeg_quality': 75},
        ],
    )
```

We also have to add the new node to the `launchDescription`:

```python
launchDescriptionObject.add_action(gz_image_bridge_node)
```

After rebuild we can try it using `rqt` and we will see huge improvement in the bandwith thanks to the `jpeg` compression.
![alt text][image5]

> If compressed images are not visible in rqt, you have to install the plugins you want to use:
> - `sudo apt install ros-jazzy-compressed-image-transport`: for jpeg and png compression
> - `sudo apt install ros-jazzy-theora-image-transport`: for theora compression
> - `sudo apt install ros-jazzy-zstd-image-transport`: for zstd compression

But we face another issue, this time in RViz, the uncompressed camera stream is visible as before but the compressed one isn't due to the following warning:

```
Camera Info
Expecting Camera Info on topic [/camera/image/camera_info]. No CameraInfo received. Topic may not exist.
```

It's because RViz always expect the `image` and `the camera_info` topics with the same prefix which works well for:

`/camera/image` &#8594; `/camera/camera_info`

But doesn't work for:

`/camera/image/compressed` &#8594; `/camera/image/camera_info`

because we don't publish the `camera_info` to that topic. We could remap the `camera_info` to that topic, but then the uncompressed image won't work in RViz, so it's not the desired solution.

But there is another useful tool that we can use, the `relay` node from the `topic_tools` package:

```python
    # Relay node to republish /camera/camera_info to /camera/image/camera_info
    relay_camera_info_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay_camera_info',
        output='screen',
        arguments=['camera/camera_info', 'camera/image/camera_info'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
```

Of course, don't forget to add it to the `launchDescription` too:
```python
launchDescriptionObject.add_action(relay_camera_info_node)
```

> If `topic_tools` is not installed you can install it with `sudo apt install ros-jazzy-topic-tools`

Rebuild the workspace and let's try it!

```bash
ros2 launch erc_gazebo_sensors spawn_robot.launch.py
```
![alt text][image6]

## rqt reconfigure

We already set up the `jpeg` quality in the `image_bridge` node with the following parameter:
```python
'camera.image.compressed.jpeg_quality': 75
```

But how do we know what is the name of the parameter and what other settings do we can change? To see that we will use the `rqt_reconfigure` node.

First start the simulation:
```bash
ros2 launch erc_gazebo_sensors spawn_robot.launch.py
```

Then start rqt_reconfigure:
```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

![alt text][image7]

We can play with the parameters here, change the compression or the algorithm as we wish and we can monitor it's impact with `rqt`.

# Lidar 

LIDAR (an acronym for “Light Detection and Ranging” or “Laser Imaging, Detection, and Ranging”) is a sensing technology that uses laser light to measure distances. LIDAR sensor typically emits pulses of laser light in a scanning pattern (2D or 3D) and measures how long it takes for the light to return after hitting nearby objects. From this, the system computes distances to obstacles or surfaces in the environment. By continuously scanning the surroundings, the LIDAR provides a 2D or 3D map of distances to any objects around the robot. Lidars are simple and important sensors of almost every mobile robot applications, it's widely used in Simultaneous Localization and Mapping (SLAM) algorithms which use LIDAR scans to build a map of the environment in real time while also estimating the robot’s pose (position and orientation) within that map.

First, we start with a simple 2D lidar, let's add it to the urdf:

```xml
  <!-- STEP 10 - Lidar -->
  <joint type="fixed" name="scan_joint">
    <origin xyz="0.0 0 0.15" rpy="0 0 0"/>
    <child link="scan_link"/>
    <parent link="base_link"/>
    <axis xyz="0 1 0" rpy="0 0 0"/>
  </joint>

  <link name='scan_link'>
    <inertial>
      <mass value="1e-5"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia
          ixx="1e-6" ixy="0" ixz="0"
          iyy="1e-6" iyz="0"
          izz="1e-6"
      />
    </inertial>
    <collision name='collision'>
      <origin xyz="0 0 0" rpy="0 0 0"/> 
      <geometry>
        <box size=".1 .1 .1"/>
      </geometry>
    </collision>

    <visual name='scan_link_visual'>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename = "package://erc_gazebo_sensors/meshes/lidar.dae"/>
      </geometry>
    </visual>
  </link>
```

Then add the plugin to the `erc_bot.gazebo` file:

```xml
  <gazebo reference="scan_link">
    <sensor name="gpu_lidar" type="gpu_lidar">
      <update_rate>10</update_rate>
      <topic>scan</topic>
      <gz_frame_id>scan_link</gz_frame_id>
      <lidar>
        <scan>
          <horizontal>
            <samples>720</samples>
            <!--(max_angle-min_angle)/samples * resolution -->
            <resolution>1</resolution>
            <min_angle>-3.14156</min_angle>
            <max_angle>3.14156</max_angle>
          </horizontal>
          <!-- Dirty hack for fake lidar detections with ogre 1 rendering in VM -->
          <!-- <vertical>
              <samples>3</samples>
              <min_angle>-0.001</min_angle>
              <max_angle>0.001</max_angle>
          </vertical> -->
        </scan>
        <range>
          <min>0.05</min>
          <max>10.0</max>
          <resolution>0.01</resolution>
        </range>
        <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
        </noise>
        <frame_id>scan_link</frame_id>
      </lidar>
      <always_on>1</always_on>
      <visualize>true</visualize>
    </sensor>
  </gazebo>
```

> If you are using OGRE 1 rendering in VM and lidar reading is not properly rendered (it renders only within a small circle around the sensor) you can try to enable 3 vertical samples with very small vertical angles.

Before we can test our lidar we have to update the `parameter_bridge` to forward the lidar scan topic:

```python
    # Node to bridge /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            #"/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
```

Let's try it in the simulation!

```bash
ros2 launch erc_gazebo_sensors spawn_robot.launch.py
```

![alt text][image8]

We can also verify the rendering of lidars in Gazebo with the `Visualize Lidar` tool:

![alt text][image9]

If we increase decay time of the visualization of lidar scans and we drive around the robot we can do a very simple "mapping" of the environment. Although in the next lesson we will see that mapping algorithms are more complicated, usually this a good quick and dirty test on real robots if odometry, lidar scan and the other components are working well together.

![alt text][image10]



