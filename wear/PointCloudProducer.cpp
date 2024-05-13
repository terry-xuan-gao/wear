#include "PointCloudProducer.h"

void read_directory(const string& folderPath, vector<string>& filenames) 
{
    for (const auto& entry : std::filesystem::directory_iterator(folderPath))
        //if (is_regular_file(entry.path()) && !entry.path().filename().string().starts_with(".")) 
		if (std::filesystem::is_regular_file(entry.path()))
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
    vector<cv::Point>* points = (vector<cv::Point>*)userdata;
    switch (event) 
    {
    case CV_EVENT_LBUTTONDOWN:
        points->push_back(cv::Point(x*4, y*4));
        qDebug() << "(" << y*4 << ", " << x*4 << ")";
        break;
    }
}

cv::Scalar blue(255, 0, 0);
cv::Scalar green(0, 255, 0);
cv::Scalar red(0, 0, 255);
cv::Scalar yellow(0, 255, 255);
cv::Scalar lightPurple(238, 130, 238);
cv::Scalar lightOrange(255, 165, 0);

PointCloudProducer::PointCloudProducer()
    : cloud(new pcl::PointCloud<pcl::PointXYZ>())
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
    
    for(int i = 0; i < 179; i ++)
        this->singleImgProcess(filenames[i], i);

}

void PointCloudProducer::viewPointCloud()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->addPointCloud(cloud, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer->spin();

    qDebug() << "The point cloud has " << this->cloud->size() << " points.";
}

void PointCloudProducer::savePointCloud()
{
    qDebug() << "Saving point cloud ...";
    
    pcl::PCDWriter writer;
    string filename = "..\\data\\" + this->currentTaskName + ".pcd";
    writer.write<pcl::PointXYZ>(filename, *cloud);
    //saveVTKFile(filename, *cloud);

    qDebug() << "Point cloud saved to " << QString::fromStdString(filename);
}

void PointCloudProducer::singleImgProcess(string imgPath, int index)
{
    cv::Mat image = cv::imread(imgPath, CV_LOAD_IMAGE_GRAYSCALE);
    qDebug() <<  "Analyze" << QString::fromStdString(imgPath);

    cv::Mat binaryImage(ROWS, COLS, CV_8UC1);
    threshold(image, binaryImage, THRESHOLD, 255, cv::THRESH_BINARY);
    //imwrite(taskPath + "\\binary-img-1.bmp", binaryImage);

    cv::Mat blurredImage;
    cv::GaussianBlur(image, blurredImage, cv::Size(5, 5), 0);

    cv::Mat _binaryImage(ROWS, COLS, CV_8UC1);
    threshold(blurredImage, _binaryImage, THRESHOLD, 255, cv::THRESH_BINARY);

    vector<vector<cv::Point>> _contours;
    cv::Mat _hierarchy;
    findContours(_binaryImage, _contours, _hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    this->fitRotationAxis(_binaryImage, _contours);

    for (const auto& contour : _contours)
        if (contour.size() > 8)
            for (const auto& point : contour)
                this->coordinateTransfTiltOptimize((double)point.x, (double)point.y, index);

    qDebug() << "Complete analysis" << QString::fromStdString(imgPath);
}

void PointCloudProducer::fitRotationAxis(cv::Mat binaryImage, 
                                         vector<vector<cv::Point>> contours)
{
    qDebug() << "fit the rotation axis";

    unordered_map<double, pair<double,int>> h;
    vector<cv::Point> axisPoints;
    cv::Mat colorImage;
    cvtColor(binaryImage, colorImage, cv::COLOR_GRAY2RGB);

    for (size_t i = 0; i < contours.size(); i++) 
        if (contours[i].size() > 200)
            drawContours(colorImage, contours, i, cv::Scalar(0, 255, 0), 2); 
    
    for (const auto& contour : contours)
        if (contour.size() > 200)
            for (const auto& point : contour)
            {
                // 1000
                //if (point.x % 5 == 0 && point.x > 1900 && point.x < 2200)
                if ((point.x > FITLEFTLINE && point.x < FITRIGHTLINE) || point.x > 2900 || point.x < 700)
                {
                    h[point.x].first += point.y;
                    h[point.x].second += 1;
                    circle(colorImage, point, 5, lightOrange, 4, 8, 0);
                }
            }
                

    for (auto& kv : h)
    {
        cv::Point pt(kv.first, kv.second.first / kv.second.second);

        if (abs(1024 - pt.y) > 50)
            continue;

        axisPoints.push_back(pt);
        circle(colorImage, pt, 5, red, 4, 8, 0);
    }

    cv::Vec4f line_para;
    fitLine(axisPoints, line_para, cv::DIST_L2, 0, 1e-2, 1e-2);

    cv::Point point0;
    point0.x = line_para[2];
    point0.y = line_para[3];

    double k = line_para[1] / line_para[0];

    cv::Point point1, point2;
    point1.x = 0;
    point1.y = k * (0.0 - point0.x) + point0.y;
    point2.x = 3095;
    point2.y = k * (3095 - point0.x) + point0.y;
    
    line(colorImage, point1, point2, cv::Scalar(0, 0, 255), 7, cv::LINE_AA, 0);


    A0 = k;
    B0 = -1;
    C0 = point0.y - k * point0.x;
    qDebug() << "A0 =" << A0 << "B0 =" << B0 << "C0 =" << C0;

    A1 = 1;
    B1 = k;
    C1 = -(A1 * 2350 + B1 * 1044); // Point x = 2400 y = 1044
    qDebug() << "A1 =" << A1 << "B1 =" << B1 << "C1 =" << C1;


    point1.y = 10;
    point1.x = -(B1 * point1.y + C1) / A1;
    point2.y = 2040;
    point2.x = -(B1 * point2.y + C1) / A1;
    line(colorImage, point1, point2, lightPurple, 7, cv::LINE_AA, 0);
    
    /*
    Mat resizedImage;
    int width = binaryImage.cols / 4;
    int height = binaryImage.rows / 4;
    resize(colorImage, resizedImage, Size(width, height));
    imshow("Fit Rotation Axis", resizedImage);
    int key = waitKey(0);
    if(key == 27)
        destroyAllWindows();
    */
}

void PointCloudProducer::coordinateTransfTiltOptimize(double x, double y, int index)
{
    double flag0 = A0 * x + B0 * y + C0;
    double flag1 = A1 * x + B1 * y + C1;

    if (flag1 > 0) return;
    
    double r = abs(flag0) / sqrt(pow(A0, 2) + pow(B0, 2));
    double theta = index / SAMPLINGFREQ;
    if (flag0 < 0)
        theta += 180;
    double z = flag1 / sqrt(pow(A1, 2) + pow(B1, 2));

    pcl::PointXYZ p;
    p.x = r * cos(theta);
    p.y = r * sin(theta);
    p.z = z;

    this->cloud->push_back(p);
}

void PointCloudProducer::reconstruction()
{
    qDebug() << "Poisson reconstuction ...";


    








}

void PointCloudProducer::coordinateTransf(double x, double y, int index)
{
    double r = abs(y - PINCENTER);
    double theta = index / SAMPLINGFREQ;
    if (y - PINCENTER >= 0)
        theta += 180;
    theta *= 3.1415926 / 180;
    double z = PINPOSTION - x;

    pcl::PointXYZ p;
    p.x = r * cos(theta);
    p.y = r * sin(theta);
    p.z = z;

    this->cloud->push_back(p);

    //qDebug() << "(" << x << ", " << y << ") -> (" << r << ", " << theta << ", " << z << ") -> (" << p.x << ", " << p.y << ", " << p.z << ")";
}

void PointCloudProducer::PointCloud2Vector3d(pcl::on_nurbs::vector_vec3d& data)
{

}

void PointCloudProducer::visualizeCurve(ON_NurbsCurve& curve,
                                        ON_NurbsSurface& surface,
                                        pcl::visualization::PCLVisualizer& viewer)
{

}

