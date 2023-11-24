#include "DataManager.h"

DataManager::DataManager()
{

}

DataManager::~DataManager()
{

}

vector<vector<string>> DataManager::getTaskList()
{
	std::ifstream csvFile(dataPath);

	if (!csvFile.is_open()) {
		qDebug() << "Error opening data.csv.";
	}
	else
	{
		std::string line;
		while (std::getline(csvFile, line) ){
		
			vector<string> t;

			std::istringstream lineStream(line);
			std::string token;

			while (std::getline(lineStream, token, ',')) 
				t.push_back(token);
			taskList.push_back(t);
		}

		csvFile.close();
	}

	return taskList;
}