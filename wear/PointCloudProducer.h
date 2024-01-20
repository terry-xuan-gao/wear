#pragma once

#include <iterator>
#include <algorithm>
#include <regex>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <filesystem>

#include "DataManager.h"

using namespace std;
using namespace cv;
using namespace std::filesystem;

class PointCloudProducer
{
public:
	PointCloudProducer();
	~PointCloudProducer();

	void getTaskName(string s);
	void generatePointCloud();

private:
	vector<vector<double>> cylindricalCoordinates;
	string currentTaskName;
};

