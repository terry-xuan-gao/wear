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
	// ��ȡ��ǰ���ں�ʱ��
	time_t now = time(nullptr);
	struct tm timeinfo = *localtime(&now);

	// ����һ���ַ���������ʽ�����ں�ʱ��
	std::stringstream ss;

	// ��ʽ�����ں�ʱ��
	ss << std::put_time(&timeinfo, "%Y%m%d%H%M%S");

	// ��ȡ��ʽ������ַ���
	std::string date_time_str = ss.str();

	// ��CSV�ļ�����д��
	std::ofstream csv_file(dataPath, std::ios::app);
	if (csv_file.is_open()) {
		// �����Ѿ�����һ��CSV�ļ�������Ҫ׷��һ������
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
		
	// ʹ��MultiByteToWideChar��������ת��
	const std::string folderPath = "..\\img\\" + date_time_str;
	int cchWideChar = MultiByteToWideChar(CP_ACP, 0, folderPath.c_str(), -1, NULL, 0);
	LPWSTR folderPathWideString = new WCHAR[cchWideChar];
	MultiByteToWideChar(CP_ACP, 0, folderPath.c_str(), -1, folderPathWideString, cchWideChar);

	// �����ļ���
	if (CreateDirectoryW(folderPathWideString, NULL) == 0)
	{
		qDebug() << "Failed to create directory: " << QString::fromStdString(folderPath);
		qDebug() << "Error: " << GetLastError() ;
	}

	return date_time_str;
}