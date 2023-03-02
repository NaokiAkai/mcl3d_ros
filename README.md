# mcl3d_ros
mcl3d_ros is a ROS package of 3D-LiDAR-based Monte Carlo localization (MCL) and also contains localization methods using distance-field-based optimization and extended-Kalman-filter-based fusion. The localizers work without INS such as odometry. Of course, INS can be available and it improves localization performance (current version does not support IMU).

![](doc/demo.gif)



# Main characteristics

mcl3d_ros can efficiently perform 3D-LiDAR-based MCL. To achieve it, mcl3d_ros uses measurement model optimization using a distance field and fusion of it with MCL via importance sampling. In short, mcl3d_ros uses two probabilistic distributions to sample the particles and fuses them. For more details, please refer this [preprint](https://arxiv.org/abs/2303.00216).

```
@misc{akai_arxiv2023_mcl3d,
  url = {https://arxiv.org/abs/2303.00216},
  author = {Akai, Naoki},
  title = {Efficient Solution to {3D-LiDAR}-based {Monte Carlo} Localization with Fusion of Measurement Model Optimization via Importance Sampling},
  publisher = {arXiv},
  year = {2023}
}
```



![](doc/mcl3d_ros_image.png)





# Install

```
$ cd /your/catkin_ws/src
$ git clone https://github.com/NaokiAkai/mcl3d_ros.git
$ cd /your/catkin_ws
$ catkin_make
$ source devel/setup.bash
```

We tested mcl3d_ros on Ubuntu 20.04 with Noetic.





# How to use

Please first prepare a pcd file of your target environment and build a distance field of the pcd file.

```
$ roslaunch mcl3d_ros pc_to_df.launch pcd_file:=/your/pcd/file.pcd yaml_file_path:=/your/yaml/file.yaml map_file_name:=your_map_name.bin
```

The distance field is saved as a binary file and its parameters are saved at the given yaml file. A distance field can also be built from a point cloud published as a ROS message.

Then, localization can be executed.

```
$ roslaunch mcl3d_ros mcl.launch map_yaml_file:=/your/yaml/file.yaml
```

A configuration file for rviz is prepared in the rviz directory.

```
$ rviz -d rviz/mcl3d_ros.rviz
```

Note that other map formats are not supported currently.



# Parameters

There are launch files at the launch directory. Descriptions of all the parameters can be seen in the launch files.

### 

# License

This software is subject to the [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0.html) License.
