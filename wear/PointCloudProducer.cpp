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
const double PI = 3.14159265358979;

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
    
    for(int i = 0; i < 180; i ++)
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
    
    theta = theta * PI / 180;

    pcl::PointXYZ p;
    p.x = r * cos(theta);
    p.y = r * sin(theta);
    p.z = z;

    this->cloud->push_back(p);
}

void PointCloudProducer::reconstruction()
{
    qDebug() << "Poisson reconstuction ...";

    pcl::visualization::PCLVisualizer viewer("B PCL");
    viewer.setBackgroundColor(255, 255, 255);
    
    //--------------���ص���-----------------------------
    pcl::on_nurbs::NurbsDataSurface data;
    
    PointCloud2Vector3d(data.interior);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler(cloud, 0, 255, 0);

    viewer.addPointCloud<pcl::PointXYZ>(cloud, handler, "cloud_cylinder");
    //printf("  %lu points in data set\n", cloud->size());
    qDebug() << cloud->size() << " points in data set";

    //-----------B���������ؽ�------------------------

    // -----B����������ϵĲ���-----------------------
    unsigned order(3);//B���������ģ�Ͷ���ʽ�Ľ���
    unsigned refinement(4);//����Ż��ĵ�������
    unsigned iterations(10);//�������Ż���ĵ�������
    unsigned mesh_resolution(128);//ÿ�����������ϵĲ�������������ڶ���ϵõ���B��������������ǻ�
    bool two_dim = true;

    pcl::on_nurbs::FittingSurface::Parameter params;
    params.interior_smoothness = 0.2;//�������汾���ƽ����
    params.interior_weight = 1.0;//����Ż�ʱ�õ���Ȩ��
    params.boundary_smoothness = 0.2;//����߽磨�ǲü��߽磩��ƽ����
    params.boundary_weight = 1.0;//�Ż�ʱ�ı߽�Ȩ��

    // --------��ʼ��B��������----------------------
    ///printf("  surface fitting ...\n");
    qDebug() << "  surface fitting ...\n";
    //����ֲ�����ϵ
    ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox(order, &data);
    pcl::on_nurbs::FittingSurface fit(&data, nurbs);
    fit.setQuiet(false); //�����Ƿ��ӡ������Ϣ

    // ----------���ӻ�����ģ��---------------------
    pcl::PolygonMesh mesh;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::Vertices> mesh_vertices;
    std::string mesh_id = "mesh_nurbs";
    pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh(fit.m_nurbs, mesh, mesh_resolution);
    viewer.addPolygonMesh(mesh, mesh_id);//���ӻ���ʼ����B��������
    //std::cout << "Before refine" << endl;
    qDebug() << "Before refine";
    viewer.spinOnce(3000);

    //----------- ���澫ϸ������---------------------
    for (unsigned i = 0; i < refinement; i++)//ÿ�ε���������ӿ��Ƶ���Ŀ
    {
        fit.refine(0);           //�����ڲ�������0����ӿ��Ƶ��Ż�
        if (two_dim)fit.refine(1);// �����ڲ�������1����ӿ��Ƶ��Ż�
        fit.assemble(params);
        fit.solve();
        pcl::on_nurbs::Triangulation::convertSurface2Vertices(fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
        viewer.updatePolygonMesh<pcl::PointXYZ>(mesh_cloud, mesh_vertices, mesh_id);
        viewer.spinOnce(3000);
        //std::cout << "refine: " << i << endl;
        qDebug() << "refine: " << i;
    }

    //----------�������Ż�ȷ���Ŀ��Ƶ����������ж�ε���������-----------
    for (unsigned i = 0; i < iterations; i++)
    {
        fit.assemble(params);
        fit.solve();
        pcl::on_nurbs::Triangulation::convertSurface2Vertices(fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
        viewer.updatePolygonMesh<pcl::PointXYZ>(mesh_cloud, mesh_vertices, mesh_id);
        viewer.spinOnce(3000);
        //std::cout << "iterations: " << i << endl;
        qDebug() << "iterations: " << i;
    }
     
    // ----------------------���B��������-------------------------

    pcl::on_nurbs::FittingCurve2dAPDM::FitParameter curve_params;
    curve_params.addCPsAccuracy = 5e-2;//������ֵ��������ߵ�֧����������ݵ�ľ�����ڸ���ֵ�������һ���Ƶ�
    curve_params.addCPsIteration = 3;  //�����п��Ƶ����ʱ���ڲ������Ż�����
    curve_params.maxCPs = 20;         //����������Ƶ����
    curve_params.accuracy = 1;      //���ߵ�ƽ����Ͼ���
    curve_params.iterations = 10;     //����������

    curve_params.param.closest_point_resolution = 0;//ÿһ��֧�����ڿ��Ƶ�ĸ���
    curve_params.param.closest_point_weight = 1.0;//������Ӧ��Ȩ��
    curve_params.param.closest_point_sigma2 = 0.1;//�����������ֵ�����ʱ������Զ�������ߵ����ľ���ֵ���ڸõ��ֵ
    curve_params.param.interior_sigma2 = 0.00001; //�ڵ���������ֵ�����ʱ������Զ�������ߵ��ڵ�ľ���ֵ���ڸõ��ֵ
    curve_params.param.smooth_concavity = 1.0;    //ƽ����͹�ԣ���ֵʹ�������ڻ��ⰼ��=0û�ã�<0���ڰ���>0���ⰼ��
    curve_params.param.smoothness = 1.0;          //ƽ�����Ȩ��

    // ����С�Ŀ��Ƶ������ʾ��һ��Բ����ʼ�����������
    //printf("  curve fitting ...\n");
    qDebug() << "  curve fitting ...\n";
    pcl::on_nurbs::NurbsDataCurve2d curve_data;
    curve_data.interior = data.interior_param;
    curve_data.interior_weight_function.push_back(true);//���ý��д�Ȩ�ص�B�����������
    ON_NurbsCurve curve_nurbs = pcl::on_nurbs::FittingCurve2dAPDM::initNurbsCurve2D(order, curve_data.interior);

    //����������ϲ����ӻ�
    pcl::on_nurbs::FittingCurve2dASDM curve_fit(&curve_data, curve_nurbs);
    // curve_fit.setQuiet (false); // �����Ƿ��ӡ������Ϣ
    curve_fit.fitting(curve_params);
    visualizeCurve(curve_fit.m_nurbs, fit.m_nurbs, viewer);

    //-------------------�ü�B��������-----------------------
    //printf("  triangulate trimmed surface ...\n");
    qDebug() << "  triangulate trimmed surface ...\n";
    viewer.removePolygonMesh(mesh_id);
    //��B��������������ǻ���������B�������߶������ⲿ�������ν��вü���
    //���ڲü�������������B���������ཻ�������߱�ʾ
    pcl::on_nurbs::Triangulation::convertTrimmedSurface2PolygonMesh(fit.m_nurbs, curve_fit.m_nurbs, mesh,
        mesh_resolution);
    viewer.addPolygonMesh(mesh, mesh_id);

    // ------------�������޼���B��������--------------------------
    if (fit.m_nurbs.IsValid())
    {
        ONX_Model model;
        ONX_Model_Object& surf = model.m_object_table.AppendNew();
        surf.m_object = new ON_NurbsSurface(fit.m_nurbs);
        surf.m_bDeleteObject = true;
        surf.m_attributes.m_layer_index = 1;
        surf.m_attributes.m_name = "surface";

        ONX_Model_Object& curv = model.m_object_table.AppendNew();
        curv.m_object = new ON_NurbsCurve(curve_fit.m_nurbs);
        curv.m_bDeleteObject = true;
        curv.m_attributes.m_layer_index = 2;
        curv.m_attributes.m_name = "trimming curve";

        model.Write("136.ply");
    }

    //printf("  ... done.\n");

    viewer.spin();
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
    for (unsigned i = 0; i < cloud->size(); i++)
    {
        pcl::PointXYZ& p = cloud->at(i);
        if (!pcl_isnan(p.x) && !pcl_isnan(p.y) && !pcl_isnan(p.z))
            data.push_back(Eigen::Vector3d(p.x, p.y, p.z));
    }
}

void PointCloudProducer::visualizeCurve(ON_NurbsCurve& curve,
                                        ON_NurbsSurface& surface,
                                        pcl::visualization::PCLVisualizer& viewer)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr curve_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::on_nurbs::Triangulation::convertCurve2PointCloud(curve, surface, curve_cloud, 4);
    for (std::size_t i = 0; i < curve_cloud->size() - 1; i++)
    {
        pcl::PointXYZRGB& p1 = curve_cloud->at(i);
        pcl::PointXYZRGB& p2 = curve_cloud->at(i + 1);
        std::ostringstream os;
        os << "line" << i;
        viewer.removeShape(os.str());
        viewer.addLine<pcl::PointXYZRGB>(p1, p2, 1.0, 0.0, 0.0, os.str());
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr curve_cps(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i = 0; i < curve.CVCount(); i++)
    {
        ON_3dPoint p1;
        curve.GetCV(i, p1);

        double pnt[3];
        surface.Evaluate(p1.x, p1.y, 0, 3, pnt);
        pcl::PointXYZRGB p2;
        p2.x = float(pnt[0]);
        p2.y = float(pnt[1]);
        p2.z = float(pnt[2]);

        p2.r = 255;
        p2.g = 0;
        p2.b = 0;

        curve_cps->push_back(p2);
    }
    viewer.removePointCloud("cloud_cps");
    viewer.addPointCloud(curve_cps, "cloud_cps");
}

