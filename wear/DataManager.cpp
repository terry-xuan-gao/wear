#include "DataManager.h"

string DataManager::dataPath = "..//data//data.csv";
vector<vector<string>> DataManager::taskList;

DataManager::DataManager()
{

}

DataManager::~DataManager()
{

}

vector<vector<string>> DataManager::loadTaskList()
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

vector<vector<string>> DataManager::getTaskList()
{
	return taskList;
}

string DataManager::getNewTask()
{
	// 获取当前日期和时间
	time_t now = time(nullptr);
	struct tm timeinfo = *localtime(&now);

	// 创建一个字符串流来格式化日期和时间
	std::stringstream ss;

	// 格式化日期和时间
	ss << std::put_time(&timeinfo, "%Y%m%d%H%M%S");

	// 获取格式化后的字符串
	std::string date_time_str = ss.str();

	// 打开CSV文件进行写入
	std::ofstream csv_file(dataPath, std::ios::app);
	if (csv_file.is_open()) {
		// 假设已经有了一个CSV文件，现在要追加一行数据
		csv_file << date_time_str + ",0,0" << std::endl;

		vector<string> t;
		t.push_back(date_time_str);
		t.push_back("0");
		t.push_back("0");

		taskList.push_back(t);

		csv_file.close();
	}
	else
	{
		qDebug() << "Failed to open CSV file for writing.";
	}
		
	// 使用MultiByteToWideChar函数进行转换
	const std::string folderPath = "..\\img\\" + date_time_str;
	int cchWideChar = MultiByteToWideChar(CP_ACP, 0, folderPath.c_str(), -1, NULL, 0);
	LPWSTR folderPathWideString = new WCHAR[cchWideChar];
	MultiByteToWideChar(CP_ACP, 0, folderPath.c_str(), -1, folderPathWideString, cchWideChar);

	// 创建文件夹
	if (CreateDirectoryW(folderPathWideString, NULL) == 0)
	{
		qDebug() << "Failed to create directory: " << QString::fromStdString(folderPath);
		qDebug() << "Error: " << GetLastError() ;
	}

	return date_time_str;
}