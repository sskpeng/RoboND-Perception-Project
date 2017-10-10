# Project: Perception Pick & Place
### In this lesson, we practiced using several image process techniques to extract perception information and recognize objects from images. Then, with the object information, we can command the robot to perform pick and place motion accordingly.

#### This project is separated into two sections - the exercise and the project itself.

#### The main focus on the exercise part is to build the perception functions step by step. Here, we started from downsizing the image grid, reducing focus area, segmentation, RANASAC plane fitting, clustering, and then use a small machine learning technique to train the object features for object recognition.

#### In the project, we implemented the perception functions to a well-prepared pick & place testing cell with a PR2 robot to test the object recognition function. We also used the same machine learning technique to train the potential objects in advance and let the robot to recognize and pick up the objects.
---
[image1]: ./writeup_pic/pic01_tabletop.png
[image2]: ./writeup_pic/pic02_downsampling.png
[image3]: ./writeup_pic/pic03_passthrough.png
[image4]: ./writeup_pic/pic04_inlier.png
[image5]: ./writeup_pic/pic05_outlier.png
[image6]: ./writeup_pic/pic06_points.png
[image7]: ./writeup_pic/pic07_clustering.png
[image8]: ./writeup_pic/pic08_figure_1_Confusion_matrix.png
[image9]: ./writeup_pic/pic09_figure_2_Normalized_confusion_matrix.png
[image10]: ./writeup_pic/pic10_recog.png
[image11]: ./writeup_pic/pic11_world1.png
[image12]: ./writeup_pic/pic12_world2.png
[image13]: ./writeup_pic/pic13_world3.png



## Exercise 1, 2 and 3 Pipeline Implemented:
In the RoboND-Perception-Exercise repository, a static set of image data are provided to practice implementing the perception pipeline. In the first two exercises, we implemented several perception filtering functions to segment RGB images. Then in the exercise 3, we use Support Vector Machine (SVM) to perform object recognition.

Here is the original tabletop image.
![alt text][image1]

These functions include:

### 1. VoxelGrid Downsamplying Filter
This function is used to reduce the sampling amount in the imaging process. In the picture below, the voxel grid size is set to 0.01 cubic meter.

```
vox = cloud.make_voxel_grid_filter()
LEAF_SIZE = 0.01
vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
```

![alt text][image2]
### 2. PassThrough Filter
This function is used to filter out and keep only image data from certain area in space. The following codes are for setting up a filter in Z-axis. In the project, three separated filters are used to limit the space in X, Y and Z directions.

```
passthrough_1 = cloud_filtered.make_passthrough_filter()
filter_axis_1 = 'z'
passthrough_1.set_filter_field_name(filter_axis_1)
axis_1_min = 0.6
axis_1_max = 1.2
passthrough_1.set_filter_limits(axis_1_min, axis_1_max)
```
![alt text][image3]
### 3. RANASAC Plane Fitting
To identify points belong to a particular model from the entire image dataset.
The table fits the rectangle plane and is extracted from the dataset from all the objects above or below it.

```
seg = cloud_filtered.make_segmenter()
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)
max_distance = 0.01
seg.set_distance_threshold(max_distance)
```
![alt text][image4]

Here is the remaining objects in the dataset.
![alt text][image5]

### 4. Outlier Removal Filter
To remove noise effect due to external factors. Since the provided image has no noise, there is no need to implemented in the exercise. But this filter will be used in the Perception project.

### 5. Euclidean Clustering
Clustering is a process to find similarities among individual points.
Density-Based Spatial Clustering of Applications with Noise (DBSACN), also known as "Euclidean Clustering", is used in this exercise and the following project for clustering points based on their density or distance between point in a cluster.

```sf
ec = white_cloud.make_EuclideanClusterExtraction()
ec.set_ClusterTolerance(0.05)
ec.set_MinClusterSize(10)
ec.set_MaxClusterSize(1000)
```
Here is the entire point cloud published in ROS in rviz.
![alt text][image6]

Here is the point cloud published to the ROS after clustering.
![alt text][image7]


### 6. Support Vector Machine (SVM)
SVM is a supervised machine learning algorithm to be used to trained for object recognition.

In this exercise, we first run the file `capture_feature.py` to capture the objects' features in rviz environment, and then run `train_svm.py ` to train SVM. The following pictures show the recognition result after training the SVM with 10 orientations for all of the seven objects and with using `hsv` instead of `xyz`. As the result shows, the success rate is around 70~80 %. Later in the project, I tried to capture 25 orientations for the 8 objects and could reach 90 % success rate.

![alt text][image8]
![alt text][image9]
![alt text][image10]


## Pick and Place Setup

### In the final project, we implemented the perception functions based on the practice and used the same machine learning technique to train the potential objects to a well-prepared pick & place testing cell with a PR2 robot to test the object recognition function. The goal for this project is to recognize objects in three different environment setups and required output for this project is to generate `output_x.yaml` files containing the location information of the objects.

### All the main programs are placed in the `project.py` file, which is edited based on the provided `project_template.py` file. In the file, we focused on three sections. In the `__init__` section, we added a few Publisher to broadcast the several different information for the other ROS programs in the project. The `pcl_callback` function is the main program for perception process. And the `pr2_mover` function is the function we command the robot to act based on the recognized object and their locations.


### Here shows the results of required output for object recognition

![alt text][image11]
![alt text][image12]
![alt text][image13]

In addition, the output.yaml files are placed under /output folder.

### Additional function for this project is to broadcast the obstacles surrounding the robot including the tables, bins, and also the recognized objects, so that the motion planner can take those into account and avoids collision in the pick and place motion.

In the end, I implemented codes to rotate the robot around to capture obstacles surrounding itself. And broadcast the obstacles including the tables, bins, and the objects on the table. The obstacles should be updated for every pick and exclude the current object to pick.

### Additional notes
To execute the project, run the `rosrun` command while under the folder `RoboND-Perception-Project\pr2_robot\scripts` and then launch the project.

```sf
rosrun pr2_robot project.py
roslaunch pr2_robot pick_and_place_project.launch
```

To change the world environment, modify `pr2_robot/launch/pick_place_project.launch`.

```sf
<arg name="world_name" value="$(find pr2_robot)/worlds/test1.world"/>
<rosparam command="load" file="$(find pr2_robot)/config/pick_list_1.yaml"/>
```

To improve the robot grab, modify the files `pr2_robot/worlds/testx.world`, and add friction sections for each object.

```sh
<friction>
  <torsional>
    <ode/>
  </torsional>
  <ode mu="1.0" mu2="1.0" fdir1="0 0 1"/>
</friction>
```
