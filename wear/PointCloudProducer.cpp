#include "PointCloudProducer.h"

const int COLS = 3072;
const int ROWS = 2048;
const int THRESHOLD = 8;


void read_directory(const string& folderPath, vector<string>& filenames) 
{
    for (const auto& entry : directory_iterator(folderPath)) 
        if (is_regular_file(entry.path()) && !entry.path().filename().string().starts_with(".")) 
            filenames.push_back(entry.path().string());
        
    std::stable_sort(filenames.begin(), filenames.end(),
        [&filenames](const std::string& a, const std::string& b) 
        {
            std::size_t a_pos1 = a.find_last_of('-');
            std::size_t a_pos2 = a.find_last_of(".bmp");
            string a_num = a.substr(a_pos1 + 1, a_pos2 - a_pos1 - 4);

            std::size_t b_pos1 = b.find_last_of('-');
            std::size_t b_pos2 = b.find_last_of(".bmp");
            string b_num = b.substr(b_pos1 + 1, b_pos2 - b_pos1 - 4);

            return stoi(a_num) < stoi(b_num);
        });
}

PointCloudProducer::PointCloudProducer()
{

}

PointCloudProducer::~PointCloudProducer()
{

}

void PointCloudProducer::getTaskName(string s)
{
	qDebug() << "Get Task Name: " << QString::fromStdString(s);
	this->currentTaskName = s;
}

void PointCloudProducer::generatePointCloud()
{
	string taskPath = "..\\img\\" + this->currentTaskName;
	qDebug() << "Generate File Path: " << QString::fromStdString(taskPath);

	vector<string> filenames;
	read_directory(taskPath, filenames);

    Mat image;

    image = imread(filenames[0], CV_LOAD_IMAGE_GRAYSCALE);
    qDebug() << image.cols << " " << image.rows;

    Mat binaryImage(ROWS, COLS, CV_8UC1);
    threshold(image, binaryImage, THRESHOLD, 255, THRESH_BINARY);
    imwrite(taskPath + "\\binary-img-1.bmp", binaryImage);
    
    vector<vector<Point>> contours;
    Mat hierarchy;
    findContours(binaryImage, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);

    double total_y = 0.0;
    int total_points = 0;
    for (const auto& contour : contours) 
        for (const auto& point : contour) 
        {
            total_y += point.y;
            total_points++;
        }
    double average_y = total_y / total_points;

    line(image, Point(1, average_y), Point(3070, average_y), Scalar(255, 255, 255), 1, CV_AA);

    for (int i = 0; i < contours.size(); i++) 
        drawContours(image, contours, i, Scalar(255, 255, 255), 2, 8, vector<Vec4i>(), 0, Point());
    imwrite(taskPath + "\\contours-img-1.bmp", image);
}


