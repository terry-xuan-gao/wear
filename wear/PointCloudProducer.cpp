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
        points->push_back(Point(x*4, y*4));
        qDebug() << "(" << y*4 << ", " << x*4 << ")";
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
    string taskPath = "..\\img\\" + this->currentTaskName;
    vector<string> filenames;
    read_directory(taskPath, filenames);
    
    xv = - (b2 - b1) / (k2 - k1);
    yv = - k1 * (b2 - b1) / (k2 - k1) + b1;
    qDebug() << "yv =" << yv << "xv =" << xv;

    A0 = k1 + k2;
    B0 = -2;
    C0 = 2 * yv - (k1 + k2) * xv;
    qDebug() << "A0 =" << A0 << "B0 =" << B0 << "C0 =" << C0;

    A1 = 2;
    B1 = k1 + k2;
    C1 = -(A1 * 2850 + B1 * 1044); // Point x = 2850 y = 1044
    qDebug() << "A1 =" << A1 << "B1 =" << B1 << "C1 =" << C1;

    Mat image = imread(filenames[0], CV_LOAD_IMAGE_GRAYSCALE);
    Mat colorImage;
    cvtColor(image, colorImage, COLOR_GRAY2RGB);
    
    Scalar blue(255, 0, 0); 
    Scalar green(0, 255, 0); 
    Scalar red(0, 0, 255); 
    Scalar yellow(0, 255, 255);
    Scalar lightPurple(238, 130, 238);
    Scalar lightOrange(255, 165, 0);

    circle(colorImage, Point(xv, yv), 5, lightOrange, 12, 8, 0);

    Point point1, point2;
    point1.x = 10;
    point1.y = -(A0 * point1.x + C0) / B0;
    point2.x = 3050;
    point2.y = -(A0 * point2.x + C0) / B0;
    line(colorImage, point1, point2, red, 7, LINE_AA, 0);

    point1.y = 10;
    point1.x = -(B1 * point1.y + C1) / A1;
    point2.y = 2040;
    point2.x = -(B1 * point2.y + C1) / A1;
    line(colorImage, point1, point2, lightPurple, 7, LINE_AA, 0);

    point1.x = 0;
    point1.y = k1 * point1.x + b1;
    point2.x = 3050;
    point2.y = k1 * point2.x + b1;
    line(colorImage, point1, point2, green, 7, LINE_AA, 0);

    point1.x = 0;
    point1.y = k2 * point1.x + b2;
    point2.x = 3050;
    point2.y = k2 * point2.x + b2;
    line(colorImage, point1, point2, yellow, 7, LINE_AA, 0);

    int width = image.cols / 4;
    int height = image.rows / 4;
    Mat resizedImage;
    resize(colorImage, resizedImage, Size(width, height));
    imshow("Tilt Optimize", resizedImage);
    int key = waitKey(0);
    if (key == 27)
        destroyAllWindows();
}

void PointCloudProducer::fitPinEnvelop(int index)
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
            << envelopLinePoints[index][i].y << ", " 
            << envelopLinePoints[index][i].x << ") " 
            << image.at<uchar>(envelopLinePoints[index][i].y, 
                envelopLinePoints[index][i].x);

    Mat colorImage;
    cvtColor(image, colorImage, COLOR_GRAY2RGB);

    for (int i = 0; i < envelopLinePoints[index].size(); i++)
        circle(colorImage, envelopLinePoints[index][i],
            5, cv::Scalar(0, 255, 0), 12, 8, 0);

    Vec4f line_para;
    fitLine(envelopLinePoints[index], line_para, cv::DIST_L2, 0, 1e-2, 1e-2);

    cv::Point point0;
    point0.x = line_para[2];
    point0.y = line_para[3];

    double k = line_para[1] / line_para[0];

    Point point1, point2;
    point1.x = 0;
    point1.y = k * (0.0 - point0.x) + point0.y;
    point2.x = 3095;
    point2.y = k * (3095 - point0.x) + point0.y;

    switch (index)
    {
    case 0:
        k1 = k;
        b1 = point1.y;
    case 1:
        k2 = k;
        b2 = point1.y;
    }
    qDebug() << "k" << index+1 << " = " << k;
    qDebug() << "b" << index+1 << " = " << point1.y;

    line(colorImage, point1, point2, cv::Scalar(0, 0, 255), 7, LINE_AA, 0);

    resize(colorImage, resizedImage, Size(width, height));
    imshow("Points", resizedImage);
    key = waitKey(0);
    if (key == 27)
        destroyAllWindows();

}

