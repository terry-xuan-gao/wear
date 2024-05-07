#pragma once

#include <iterator>
#include <algorithm>
#include <regex>
#include <filesystem>
#include <iostream>
#include <stdlib.h>
#include <unordered_map>
#include <windows.h>

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
	void savePointCloud();

	double getPinCenter(int imgNum);
	void tiltOptimize();

private:
	void calculatePinCenter(int imgNum);
	void coordinateTransf(double x, double y, int index);
	void coordinateTransfTiltOptimize(double x, double y, int index);
	void singleImgProcess(string imgPath, int index);
	void fitRotationAxis(Mat hierarchy, vector<vector<Point>> contours);
	
	vector<vector<double>> cylindricalCoordinates;
	string currentTaskName;
	PointCloud<PointXYZ>::Ptr cloud;

	const int COLS = 3072, ROWS = 2048;
	const int THRESHOLD = 8;
	const double PINPOSTION = 2350.0;
	double PINCENTER = 1060.21;
	const int SAMPLINGFREQ = 1;

	const int FITLEFTLINE = 2100;
	const int FITRIGHTLINE = 2300;

	vector<vector<Point>> envelopLinePoints;

	double xv = -558.169,   yv = 1136.51;
	double A0 = -0.0699067, B0 = -2,         C0 = 2234;
	double A1 = 2,          B1 = -0.0699067, C1 = -5627.02;
	double k1 = -0.165738,  b1 = 1044;
	double k2 = 0.0958313,  b2 = 1190;
};

