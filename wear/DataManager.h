#pragma once

#include <string>
#include <vector>

#include <iostream>
#include <fstream>
#include <sstream>

#include <QFile>
#include <QTextStream>
#include <QDebug>

using namespace std;

class DataManager
{
public:
	DataManager();
	~DataManager();

	vector<vector<string>> getTaskList();

private:	
	
	string dataPath = "..//data//data.csv";
	vector<vector<string>> taskList;


};

