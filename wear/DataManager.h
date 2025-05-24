#pragma once

#include <string>
#include <vector>
#include <unordered_map>

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <ctime>
#include <windows.h>

#include <QFile>
#include <QTextStream>
#include <QDebug>

using namespace std;

class DataManager
{
public:
	DataManager();
	~DataManager();

	static vector<vector<string>> loadTaskList();
	static vector<vector<string>> getTaskList();
	string getNewTask();

	unordered_map<string, double> loadValueList();
	double getValue(const std::string& name);
	double setValue(const std::string& name, double newValue);



private:	
	
	static string dataPath ;
	static vector<vector<string>> taskList;

	string filePath = "../data/value.csv";
	unordered_map<std::string, double> values;
};



