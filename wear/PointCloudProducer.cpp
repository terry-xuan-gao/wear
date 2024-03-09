#include "PointCloudProducer.h"

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

void onMouseCallback(int event, int x, int y, int flags, void* userdata)
{
    vector<Point>* points = (vector<Point>*)userdata;
    switch (event) 
    {
    case CV_EVENT_LBUTTONDOWN:
        points->push_back(Point(x, y));
        qDebug() << "(" << y << ", " << x << ")";
        break;
    }
}

PointCloudProducer::PointCloudProducer()
    : cloud(new PointCloud<PointXYZ>())
{
    envelopLinePoints.resize(2);
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
    
    for(int i = 0; i < 180; i ++)
        this->singleImgProcess(filenames[i], i);

}

void PointCloudProducer::viewPointCloud()
{
    PCLVisualizer::Ptr viewer(new PCLVisualizer("3D Viewer"));
    viewer->addPointCloud(cloud, "cloud");
    viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer->spin();

    qDebug() << "The point cloud has " << this->cloud->size() << " points.";
}

void PointCloudProducer::savePointCloud()
{
    qDebug() << "Saving point cloud...";
    
    PCDWriter writer;
    string filename = "..\\data\\" + this->currentTaskName + ".pcd";
    writer.write<pcl::PointXYZ>(filename, *cloud);
    //saveVTKFile(filename, *cloud);

    qDebug() << "Point cloud saved to " << QString::fromStdString(filename);
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

double PointCloudProducer::getPinCenter(int imgNum)
{
    qDebug() << "Calculating pin center... ";
    this->calculatePinCenter(imgNum);
    return PINCENTER;
    qDebug() << "PINCENTER = " << PINCENTER;
}

void PointCloudProducer::calculatePinCenter(int imgNum)
{
    string taskPath = "..\\img\\" + this->currentTaskName;

    vector<string> filenames;
    read_directory(taskPath, filenames);

    double total_y_avg = 0.0;
    for (int i = 0; i < imgNum; i++)
    {
        qDebug() << i << " " << QString::fromStdString(filenames[i]);
        
        Mat image = imread(filenames[i], CV_LOAD_IMAGE_GRAYSCALE);
    
        Mat binaryImage(ROWS, COLS, CV_8UC1);
        threshold(image, binaryImage, THRESHOLD, 255, THRESH_BINARY);
    
        vector<vector<Point>> contours;
        Mat hierarchy;
        findContours(binaryImage, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);

        double total_y = 0.0;
        int total_points = 0;
        for (const auto& contour : contours)
            for (const auto& point : contour)
                if(point.x < PINPOSTION)
                {
                    total_y += point.y;
                    total_points++;
                }

        double y_avg = total_y / total_points;
        total_y_avg += y_avg;
    }
    PINCENTER = total_y_avg / imgNum;
}

void PointCloudProducer::tiltOptimize()
{
    
}

void PointCloudProducer::fitEnvelopOfPinEnvelop(int index)
{
    string taskPath = "..\\img\\" + this->currentTaskName;
    vector<string> filenames;
    read_directory(taskPath, filenames);

    qDebug() << QString::fromStdString(taskPath);

    Mat image = imread(filenames[0], CV_LOAD_IMAGE_GRAYSCALE);
    Mat binaryImage(ROWS, COLS, CV_8UC1);
    threshold(image, binaryImage, THRESHOLD, 255, THRESH_BINARY);

    vector<vector<Point>> contours;
    Mat hierarchy;
    findContours(binaryImage, contours, hierarchy, 
        RETR_EXTERNAL, CHAIN_APPROX_NONE);

    for (int i = 0; i < contours.size(); i++)
        if (contours[i].size() > 10)
            drawContours(image, contours, i, Scalar(255, 255, 255), 
                2, 8, vector<Vec4i>(), 0, Point());

    int width = image.cols / 4;
    int height = image.rows / 4;
    Mat resizedImage;
    resize(image, resizedImage, Size(width, height));
    imshow("Contours", resizedImage);
    setMouseCallback("Contours", onMouseCallback, &envelopLinePoints[index]);

    int key = waitKey(0);
    if (key == 27)
        destroyAllWindows();

    qDebug() << "Point Number: " << envelopLinePoints[index].size();
    for (int i = 0; i < envelopLinePoints[index].size(); i++)
        qDebug() << "Point  " << i << "  : (" 
            << envelopLinePoints[index][i].y << ", " << envelopLinePoints[index][i].x << ") " 
            << resizedImage.at<uchar>(envelopLinePoints[index][i].y, envelopLinePoints[index][i].x);

    for (int i = 0; i < envelopLinePoints[index].size(); i++)
        circle(resizedImage, envelopLinePoints[index][i], 5, cv::Scalar(255, 255, 255), 3, 8, 0);
    //imshow("Points", resizedImage);

    Vec4f line_para;
    fitLine(envelopLinePoints[index], line_para, cv::DIST_L2, 0, 1e-2, 1e-2);

    //获取点斜式的点和斜率
    cv::Point point0;
    point0.x = line_para[2];
    point0.y = line_para[3];

    double k = line_para[1] / line_para[0];

    Point point1, point2;
    point1.x = 0;
    point1.y = k * (0 - point0.x) + point0.y;
    point2.x = 773;
    point2.y = k * (773 - point0.x) + point0.y;

    line(resizedImage, point1, point2, cv::Scalar(255, 255, 255), 2, 8, 0);
    imshow("Points", resizedImage);
    key = waitKey(0);
    if (key == 27)
        destroyAllWindows();
}

