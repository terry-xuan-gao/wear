#pragma once

#include <iterator>
#include <algorithm>
#include <regex>
#include <filesystem>
#include <iostream>
#include <stdlib.h>

#include <cmath>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

#include "DataManager.h"

using namespace std;
using namespace cv;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::visualization;
using namespace std::filesystem;

class PointCloudProducer
{
public:
	PointCloudProducer();
	~PointCloudProducer();

	void getTaskName(string s);
	void generatePointCloud();
	void viewPointCloud();
	void singleImgProcess(string imgPath, int rad);

private:
	void getPinCenter(string s);
	void coordinateTransf(double x, double y, int index);
	
	vector<vector<double>> cylindricalCoordinates;
	string currentTaskName;
	PointCloud<PointXYZ>::Ptr cloud;
};

