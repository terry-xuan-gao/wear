#pragma once

#include <iterator>
#include <algorithm>
#include <regex>
#include <filesystem>
#include <iostream>
#include <stdlib.h>
#include <unordered_map>
#include <windows.h>
#include <string>

#include <cmath>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>


#include <pcl/io/pcd_io.h>

#include <pcl/io/ply_io.h>

// poisson reconstruct
/*
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/poisson.h>
*/

#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>
#include <pcl/console/parse.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

#include <QFileDialog>

#include "DataManager.h"

//using namespace std;
//using namespace cv;
//using namespace pcl;
//using namespace pcl::io;
//using namespace pcl::visualization;
//using namespace std::filesystem;

class PointCloudProducer
{
public:
	PointCloudProducer();
	~PointCloudProducer();

	void getTaskName(string s);
	void generatePointCloud();
	void viewPointCloud();
	void viewPointCloud(std::string pcdFile);
	void savePointCloud();

	void reconstruction();


private:
	
	void coordinateTransf(double x, double y, int index);
	bool coordinateTransfTiltOptimize(double x, double y, int index);
	void singleImgProcess(string imgPath, int index);
	void fitRotationAxis(cv::Mat hierarchy, 
						 vector<vector<cv::Point>> contours);

	// Fitting trimmed B-splines to unordered point clouds
	void PointCloud2Vector3d(pcl::on_nurbs::vector_vec3d& data);
	void visualizeCurve(ON_NurbsCurve& curve,
						ON_NurbsSurface& surface,
						pcl::visualization::PCLVisualizer& viewer);

	double computePolygonArea(const std::vector<cv::Point>& points);
	
	std::vector<vector<double>> cylindricalCoordinates;
	std::string currentTaskName;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

	const int COLS = 3072, ROWS = 2048;
	const int THRESHOLD = 240;
	const double PINPOSTION = 2350.0;
	double PINCENTER = 1020.21;
	const int SAMPLINGFREQ = 1;

	const int FITLEFTLINE = 2100;
	const int FITRIGHTLINE = 2300;


	volatile double A0 = -0.0045,    B0 = -1,         C0 = 1020;
	volatile double A1 = 1,          B1 = -0.0045,    C1 = -2130;

	int toolShoulderX = 2135, toolShoulderY = 1044;
	int valueMapSize = 1500;

	volatile double volume = 0.0;

	double perfectValue = 2.50649e+08;
	std::vector<double> lValueMap;
	std::vector<double> rValueMap;
	double imageValue;
	double toolValue;
};

