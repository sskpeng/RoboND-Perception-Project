# Project: Perception Pick & Place
### Purpose/goal of this Project

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
In the RoboND-Perception-Exercise repository, a static set of image data are provided to practice implementing the perception pipeline. In the first two exercieses, we implemented several perception filtering functions to segment RGB images. Then in the exercise 3, we use Support Vector Machine (SVM) to perform object recognition.

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


## Exercuse 1, 2 and 3 Pipeline Implemented:

### Here explain the goal of this project

### Here explain the code layout

### Here shows the results of required output for object recognition

![alt text][image11]
![alt text][image12]
![alt text][image13]

In addition, the output.yaml files are placed under /output folder.

### Here shows the additional function of the code

Rotates robot to capture obstacles surrounding itself.
Brocast the obstacels including the tables, bins, and the objects on the table. The obstacles needs to be updated whenever robot comes to pick up next object, and also need to exclude the current object to pick.
