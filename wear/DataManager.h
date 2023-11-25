#pragma once

#include <string>
#include <vector>

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <ctime>

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

private:	
	
	static string dataPath ;
	
	static vector<vector<string>> taskList;

};



