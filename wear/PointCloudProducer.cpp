#include "PointCloudProducer.h"

const int COLS = 3072;
const int ROWS = 2048;
const int THRESHOLD = 8;
const double PINPOSTION = 2350.0;
double PINCENTER = 1060.21;
const int SAMPLINGFREQ = 1;


void read_directory(const string& folderPath, vector<string>& filenames) 
{
    for (const auto& entry : directory_iterator(folderPath)) 
        //if (is_regular_file(entry.path()) && !entry.path().filename().string().starts_with(".")) 
		if (is_regular_file(entry.path()))
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
    : cloud(new PointCloud<PointXYZ>())
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

    //for(auto fn : filenames)
    //    qDebug() << QString::fromStdString(fn);

    //this->singleImgProcess(filenames[0], 0);
    
    for(int i = 0; i < 180; i ++)
        this->singleImgProcess(filenames[i], i);

    this->viewPointCloud();
}

void PointCloudProducer::viewPointCloud()
{
    qDebug() << "The point cloud has " << this->cloud->size() << " points.";
    PCLVisualizer::Ptr viewer(new PCLVisualizer("3D Viewer"));
    viewer->addPointCloud(cloud, "cloud");
    viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer->spin();
}

void PointCloudProducer::singleImgProcess(string imgPath, int index)
{
    //string taskPath = "..\\img\\" + this->currentTaskName;
    
    Mat image = imread(imgPath, CV_LOAD_IMAGE_GRAYSCALE);
    qDebug() <<  "Analyze" << QString::fromStdString(imgPath);

    Mat binaryImage(ROWS, COLS, CV_8UC1);
    threshold(image, binaryImage, THRESHOLD, 255, THRESH_BINARY);
    //imwrite(taskPath + "\\binary-img-1.bmp", binaryImage);

    vector<vector<Point>> contours;
    Mat hierarchy;
    findContours(binaryImage, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);

    //line(image, Point(1, average_y), Point(3070, average_y), Scalar(255, 255, 255), 1, CV_AA);
    /*for (int i = 0; i < contours.size(); i++)
        drawContours(image, contours, i, Scalar(255, 255, 255), 2, 8, vector<Vec4i>(), 0, Point());*/
    //imwrite(taskPath + "\\contours-img-1.bmp", image);

    for (const auto& contour : contours)
        for (const auto& point : contour)
            this->coordinateTransf((double)point.x, (double)point.y, index);

    qDebug() << "Complete analysis" << QString::fromStdString(imgPath);
}

void PointCloudProducer::coordinateTransf(double x, double y, int index)
{
    double r = abs(y - PINCENTER);
    double theta = index / SAMPLINGFREQ;
    if (y - PINCENTER >= 0)
        theta += 180;
    theta *= 3.1415926 / 180;
    double z = PINPOSTION - x;

    PointXYZ p;
    p.x = r * cos(theta);
    p.y = r * sin(theta);
    p.z = z;

    this->cloud->push_back(p);

    //qDebug() << "(" << x << ", " << y << ") -> (" << r << ", " << theta << ", " << z << ") -> (" << p.x << ", " << p.y << ", " << p.z << ")";
}

void PointCloudProducer::getPinCenter(string imgPath)
{
    Mat image = imread(imgPath, CV_LOAD_IMAGE_GRAYSCALE);
    
    Mat binaryImage(ROWS, COLS, CV_8UC1);
    threshold(image, binaryImage, THRESHOLD, 255, THRESH_BINARY);
    
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
    
    PINCENTER = total_y / total_points;
}
