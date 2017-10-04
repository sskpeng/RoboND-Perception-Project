## Project: Perception Pick & Place
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



# Exercuse 1, 2 and 3 Pipeline Implemented:
In the RoboND-Perception-Exercise repository, a static set of image data are provided to practice implementing the perception pipeline. In the first two exercieses, we implemented several perception filtering functions to segment RGB images. Then in the exercise 3, we use Support Vector Machine (SVM) to perform object recognition.

Here is the original tabletop image.
![alt text][image1]

These functions include:

#### 1. VoxelGrid Downsamplying Filter
To reduce the sampling amount in the imaging process.
![alt text][image2]
#### 2. PassThrough Filter
To filter out and keep only image data from certain area in space.
![alt text][image3]
#### 3. RANASAC Plane Fitting
To identify points belong to a particular model from the entire image dataset.
The table fits the rectangle shape and is extracted from the dataset.
![alt text][image4]
Objects in the remaining dataset
![alt text][image5]

#### 4. Outlier Removal Filter
To remove noise effect due to external factors. Since the provided image has no noise, there is no need to implemeted in the exercise. But this filter will be used in the Perception project.

#### 5. Euclidean Clustering
Clustering is a process to find similarities among individual points.
Density-Based Spatial Clustering of Applications with Noise (DBSACN). Also known as "Euclidean Clustering". This algorithm is a nice alternative to k-means when you don' t know how many clusters to expect in your data, but you do know something about how the points should be clustered in terms of density (distance between points in a cluster).

Here is the entire point cloud published in ROS in rviz.
![alt text][image6]

Here is the point cloud published to the ROS after clustering.
![alt text][image7]


#### 6. Support Vector Machine (SVM)
SVM is a supervised machine learning algorithm to be used to trained for object recognition.
![alt text][image8]
![alt text][image9]
![alt text][image10]


	

# Extra Challenges: Complete the Pick & Place
7. To create a collision map, publish a point cloud to the `/pr2/3d_map/points` topic and make sure you change the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
8. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`
9. Rotate the robot back to its original state.
10. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
11. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
12. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
13. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!

## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  

#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.
Here is an example of how to include an image in your writeup.

![demo-1](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

And here's another image!
![demo-2](https://user-images.githubusercontent.com/20687560/28748286-9f65680e-7468-11e7-83dc-f1a32380b89c.png)

Spend some time at the end to discuss your code, what techniques you used, what worked and why, where the implementation might fail and how you might improve it if you were going to pursue this project further.  
