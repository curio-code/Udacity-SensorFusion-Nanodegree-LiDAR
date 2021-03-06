# LiDAR Obstacle Detection

![alt text](https://github.com/curio-code/Udacity-SensorFusion-Nanodegree-LiDAR/blob/master/media/video.gif)

**Lidar** sensing gives us high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return. Also we can tell a little bit about the object that was hit by measuring the intesity of the returned signal. Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range. While lidar sensors gives us very high accurate models for the world around us in 3D, they are currently very expensive, upwards of $60,000 for a standard unit.

## Code PipeLine
![alt text](https://github.com/curio-code/Udacity-SensorFusion-Nanodegree-LiDAR/blob/master/media/flowchart.png)

## Filtering the LiDAR Data
Filtering of point cloud was done so as to Downsample the pointcloud and to exclude unwanted points such as points on car roof top and points located outside out of region of interest.

  1. ```pcl::VoxelGrid``` function was utilized to downasample the point cloud.
  2. ```pcl::CropBox``` function was utilized to exclude the rooftop points and points outside ROI.
  
## Segmentaion
Segmentation divides the scene into plane and objects. PCL inbuilt RANSAC algorithm was utilized to implement segmentation. However a self written RANSAC algorithm can be found at ```quiz->ransac```.<br /> 
The complete explaination and implementation has been disscussed at ```quiz/ransac/RansacQuiz.md```. 
<br /> 

![alt text](https://github.com/curio-code/Udacity-SensorFusion-Nanodegree-LiDAR/blob/master/media/ransac1.png)
![alt text](https://github.com/curio-code/Udacity-SensorFusion-Nanodegree-LiDAR/blob/master/media/ransac2.png)

## Clustering
![alt text](https://github.com/curio-code/Udacity-SensorFusion-Nanodegree-LiDAR/blob/master/media/clustering.png)


## Installation - Linux
```
$> git clone https://github.com/curio-code/Udacity-SensorFusion-Nanodegree-LiDAR.git
$> cd Udacity-SensorFusion-Nanodegree-LiDAR
$> mkdir build && cd build
$> cmake ..
$> make
$> ./environment
```

## PCL Installation

Install PCL, C++

The link here is very helpful, 
https://larrylisky.com/2014/03/03/installing-pcl-on-ubuntu/

A few updates to the instructions above were needed.

* libvtk needed to be updated to libvtk6-dev instead of (libvtk5-dev). The linker was having trouble locating libvtk5-dev while building, but this might not be a problem for everyone.

* BUILD_visualization needed to be manually turned on, this link shows you how to do that,
http://www.pointclouds.org/documentation/tutorials/building_pcl.php
