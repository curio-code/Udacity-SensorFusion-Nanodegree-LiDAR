/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors
#include <chrono>
#include <string>
#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <unordered_set>
#include "kdtree.h"


std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    
    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    //renderRays(viewer,lidar->position,inputCloud);
    // TODO:: Create point processor
    //renderPointCloud(viewer, inputCloud,"inputCloud");
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);
    int clusterId = 0;
    bool render_clusters=true;
    bool render_box=true;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters){
        if (render_clusters){

            std::cout<<"Cluster Size ";
            pointProcessor.numPoints(cluster);
            renderPointCloud(viewer, cluster, "objectCloud"+std::to_string(clusterId), colors[clusterId]);
//        }
  //      if (render_box){
            Box box = pointProcessor.BoundingBox(cluster);
            renderBox(viewer,box, clusterId, colors[clusterId]);
        }
        ++clusterId;
    }
   //renderPointCloud(viewer,segmentCloud.first,"objectCloud",Color(1,0,0));
    //renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
}
//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	//srand(time(NULL));
	//float A,B,C;
	// TODO: Fill in this function
	while(maxIterations--){

		std::unordered_set<int> inliers;
		while (inliers.size()<3){
			inliers.insert(rand()%(cloud->points.size()));
		}
		float x1 ,x2, x3, y1, y2,y3, z1, z2, z3 ;
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;
		//int dist = 0;
		float A = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
		float B = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
		float C = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
		float D = -1*(A*x1+B*y1+C*z1);
		for (int index=0; index<cloud->points.size();index++){
			if (inliers.count(index)>0){
				continue;
			}
			pcl::PointXYZI point = cloud->points[index];
			float x0 = point.x;
			float y0 = point.y;
			float z0 = point.z;
			float dist = fabs(A*x0+B*y0+C*z0+D)/sqrt(A*A+B*B+C*C);
			if (dist<=distanceTol){
				inliers.insert(index);
			}
			
		}
		if (inliers.size()>inliersResult.size()){
			inliersResult = inliers;
		}
	}
	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers

	
	return inliersResult;

}

std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,  pcl::PointCloud<pcl::PointXYZI>::Ptr> SegmentPlane_UD(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    std::unordered_set<int> inliers = RansacPlane(cloud, maxIterations, distanceThreshold);
    pcl::PointCloud<pcl::PointXYZI>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZI>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZI point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult;
    segResult.first = cloudOutliers;
    segResult.second =  cloudInliers;
    return segResult;
}
/*std::vector<std::vector<float>> CreateVector(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    KdTree* tree = new KdTree;
    std::vector<float> = p;
    for (int i=0; i<cloud->points.size(); i++) {
        p.insert({cloud->points[i].x,cloud->points[i].y,cloud->points[i].z});
    	tree->insert({cloud->points[i].x,cloud->points[i].y,cloud->points[i].z},i); 
    }
}*/
void clusterHelper(int indice, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol){
	processed[indice]= true;
	cluster.push_back(indice);

	std::vector<int> nearest = tree->search(points[indice], distanceTol);

	for (int id : nearest){
		if (!processed[id])
			clusterHelper(id, points, cluster, processed, tree, distanceTol);
			
	}
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed (points.size(), false);
	int i =0;
	while (i<points.size()){
		if (processed[i]){
			i++;
			continue;
		}
		std::vector<int> cluster;
		clusterHelper(i, points, cluster, processed, tree, distanceTol);
        if (cluster.size()>100){
            clusters.push_back(cluster);
        }
		
		i++;

	} 
	return clusters;

}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    Eigen::Vector4f minPoint (-30, -6.5, -3, 1);
    Eigen::Vector4f maxPoint (30, 6.5, 10, 1);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.2, minPoint, maxPoint);
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = SegmentPlane_UD(filterCloud, 100, 0.2);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters;
    
    KdTree* tree = new KdTree;
    std::vector<std::vector<float>>  p;
    for (int i=0; i<segmentCloud.first->points.size(); i++) {
        p.push_back({segmentCloud.first->points[i].x,segmentCloud.first->points[i].y,segmentCloud.first->points[i].z});
    	tree->insert({segmentCloud.first->points[i].x,segmentCloud.first->points[i].y,segmentCloud.first->points[i].z},i); 
        }
    std::vector<std::vector<int>> clusters = euclideanCluster(p, tree, 0.53);
    int clusId = 0;
	//std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  	for(std::vector<int> clus : clusters)
  	{
  		//pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZI>);
  		for(int indice: clus){
            pcl::PointXYZI point;
  		    point.x = p[indice][0];
  		    point.y = p[indice][1];
  		    point.z = p[indice][2];
            point.intensity = 0.0;
  		    tempCloud->points.push_back(point);}
        cloudClusters.push_back(tempCloud);
  		//renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%3]);
  		++clusId;
  	}
    int clusterId = 0;
    bool render_clusters=true;
    bool render_box=true;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters){
        if (render_clusters){

            std::cout<<"Cluster Size ";
            pointProcessorI->numPoints(cluster);
            renderPointCloud(viewer, cluster, "objectCloud"+std::to_string(clusterId));
//        }
  //      if (render_box){
            Box box = pointProcessorI->BoundingBox(cluster);
            renderBox(viewer,box, clusterId);
            renderPointCloud(viewer, segmentCloud.second, "ground_plane"+std::to_string(clusterId));
        }
        ++clusterId;
    }
    
    
    
    
    //renderPointCloud(viewer, segmentCloud.first, "filtercloud");
}




int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto stream_iter = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud ;
    /*KdTree* tree = new KdTree;
    std::vector<std::vector<float>>  p;
    for (int i=0; i<inputCloud->points.size(); i++) {
        p.push_back({inputCloud->points[i].x,inputCloud->points[i].y,inputCloud->points[i].z});
    	tree->insert({inputCloud->points[i].x,inputCloud->points[i].y,inputCloud->points[i].z},i); 
    }
    for (int i=0; i<p.size(); i++) 
    	tree->insert(p[i],i);
    std::vector<std::vector<int>> clusters = euclideanCluster(p, tree, 4.0);*/
    while (!viewer->wasStopped ())
    {
        // clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // load pcd
        inputCloud = pointProcessorI->loadPcd((*stream_iter).string());
        

        // run obstacle detection
        cityBlock(viewer, pointProcessorI, inputCloud);

        stream_iter++;
        if (stream_iter == stream.end())
            stream_iter = stream.begin();

        viewer->spinOnce ();
    }

}