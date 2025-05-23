#include "Capture.h"

#define TRIGGER_SOURCE   7
#define TRIGGER_ON       1
#define TRIGGER_OFF      0
#define IMAGE_NAME_LEN   64

Capture::Capture(QWidget* parent)
    : QWidget(parent)
{
    this->resize(400, 300);
    this->layout = new QVBoxLayout();
    
    this->initDisplayLabel();
    this->initButtons();
    this->initStatusLabel();
    this->initProgressBar();

    this->myThread = new MyThread;

    this->statusLabel->setText("STATUS: prepare for capturing!");
}

Capture::~Capture()
{
    delete this->layout;
    delete this->statusLabel;

    delete this->enumButton;
    delete this->openButton;
    delete this->closeButton;
    delete this->continueModeSetButton;
    delete this->triggerModeSetButton;
    delete this->startGrabbingButton;
    delete this->stopGrabbingButton;
    
    delete this->saveButton;
    delete this->scanButton;

    delete this->imageDisplayLabel;

    delete this->m_stDevList;

    delete this->myImage;
    delete this->myThread;
}


void Capture::initDisplayLabel()
{
    this->imageDisplayLabel = new QLabel("image");

    imageDisplayLabel->setScaledContents(true);
    int fixedWidth = 480;
    int fixedHeight = 320;
    imageDisplayLabel->setMinimumSize(fixedWidth, fixedHeight);
    imageDisplayLabel->setMaximumSize(fixedWidth, fixedHeight);

    this->layout->addWidget(imageDisplayLabel, 0, Qt::AlignHCenter);

    QImage image("..\\ui\\capture.jpg");

    image = (image).scaled(300, 200, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
    QPixmap pixmap = QPixmap::fromImage(image);
    imageDisplayLabel->setPixmap(pixmap);
}

void Capture::initButtons()
{
    this->enumButton = new QPushButton("ENUM");

    this->openButton = new QPushButton("OPEN CAMERA");
    this->closeButton = new QPushButton("CLOSE CAMERA");

    this->continueModeSetButton = new QPushButton("CONTINUE MODE");
    this->triggerModeSetButton = new QPushButton("TRIGGER MODE");

    this->startGrabbingButton = new QPushButton("START GRABBING");
    this->stopGrabbingButton = new QPushButton("STOP GRABBING");
    
    this->saveButton = new QPushButton("SAVE ONE IMAGE");
    this->scanButton = new QPushButton("SCAN TOOL PIN");
    this->testButton = new QPushButton("TEST");
    this->softTriggerButton = new QPushButton("SOFT TRIGGER");


    connect(this->enumButton, &QPushButton::clicked,
        this, &Capture::enumButtonClicked);
    connect(this->openButton, &QPushButton::clicked,
        this, &Capture::openButtonClicked);
    connect(this->closeButton, &QPushButton::clicked,
        this, &Capture::closeButtonClicked);

    connect(this->continueModeSetButton, &QPushButton::clicked,
        this, &Capture::continueModeButtonClicked);

    connect(this->startGrabbingButton, &QPushButton::clicked,
        this, &Capture::startGrabbingButtonClicked);
    connect(this->stopGrabbingButton, &QPushButton::clicked,
        this, &Capture::stopGrabbingButtonClicked);

    connect(this->saveButton, &QPushButton::clicked,
        this, &Capture::saveButtonClicked);
    connect(this->scanButton, &QPushButton::clicked,
        this, &Capture::scanButtonClicked);
    connect(this->testButton, &QPushButton::clicked,
        this, &Capture::testButtonClicked);

    this->layout->addWidget(enumButton);

    QHBoxLayout* cameraLayout = new QHBoxLayout;
    cameraLayout->addWidget(openButton);
    cameraLayout->addWidget(closeButton);
    this->layout->addLayout(cameraLayout);

    QHBoxLayout* modeLayout = new QHBoxLayout;
    modeLayout->addWidget(continueModeSetButton);
    modeLayout->addWidget(triggerModeSetButton);
    this->layout->addLayout(modeLayout);

    QHBoxLayout* grabLayout = new QHBoxLayout;
    grabLayout->addWidget(startGrabbingButton);
    grabLayout->addWidget(stopGrabbingButton);
    this->layout->addLayout(grabLayout);

    QHBoxLayout* captureLayout = new QHBoxLayout;
    captureLayout->addWidget(saveButton);
    captureLayout->addWidget(scanButton);
    captureLayout->addWidget(testButton);
    captureLayout->addWidget(softTriggerButton);
    this->layout->addLayout(captureLayout);

    this->openButton->setEnabled(false);
    this->closeButton->setEnabled(false);
    this->continueModeSetButton->setEnabled(false);
    this->triggerModeSetButton->setEnabled(false);
    this->startGrabbingButton->setEnabled(false);
    this->stopGrabbingButton->setEnabled(false);

    this->saveButton->setEnabled(false);
    this->softTriggerButton->setEnabled(false);

    this->saveButton->setVisible(false);
    this->softTriggerButton->setVisible(false);

    this->setLayout(layout);
    
}

void Capture::initStatusLabel()
{
    this->statusLabel = new QLabel("STATUS: OK", this);
    this->statusLabel->setAlignment(Qt::AlignBottom | Qt::AlignLeft);
    this->layout->addWidget(this->statusLabel);
}

void Capture::initProgressBar()
{
    this->progressBar = new QProgressBar();
    this->layout->addWidget(this->progressBar);

    progressBar->setRange(0, 100);
    progressBar->setValue(0);
}

void Capture::enumButtonClicked()
{    
    progressBar->setValue(10);
    this->enumCamera();
    progressBar->setValue(90);
    if (devices_num > 0)
        this->openButton->setEnabled(true);
    progressBar->setValue(100);
}

void Capture::openButtonClicked()
{
    this->openButton->setEnabled(false);
    this->closeButton->setEnabled(true);
    //this->triggerModeSetButton->setEnabled(true);
    this->continueModeSetButton->setEnabled(true);

    this->openCamera();
}

void Capture::closeButtonClicked()
{
    this->openButton->setEnabled(true);
    this->closeButton->setEnabled(false);

    this->continueModeSetButton->setEnabled(false);
    this->triggerModeSetButton->setEnabled(false);
    this->startGrabbingButton->setEnabled(false);
    this->stopGrabbingButton->setEnabled(false);
    this->saveButton->setEnabled(false);

    this->closeCamera();
}

void Capture::continueModeButtonClicked()
{
    this->startGrabbingButton-> setEnabled(true);
    m_nTriggerMode = TRIGGER_ON;
    m_nAcquisitionMode = 2;
}

void Capture::startGrabbingButtonClicked()
{
    m_bContinueStarted = 1; // 为触发模式标记一下，切换触发模式时先执行停止采集图像函数

    this->startGrabbingButton->setEnabled(false);
    this->stopGrabbingButton->setEnabled(true);

    this->saveButton->setEnabled(true);
    this->scanButton->setEnabled(true);

    // 先判断什么模式，再判断是否正在采集
    if (m_nTriggerMode == TRIGGER_ON)
    {
        for (int i = 0; i < devices_num; i++)
        {
            //开启相机采集
            unsigned int nRet = m_pcMyCamera[i]->StartGrabbing();
            if (MV_OK != nRet)
            {
                QString hexStr = QString::number(nRet, 16);
                qDebug() << "StartGrabbing failed! nRet [" << hexStr << "]";
            }
            else {
                qDebug() << "Start Grabbing";
            }

            //myThread->getCameraPtr(m_pcMyCamera[i]); //线程获取相机指针
            //myThread->getImagePtr(myImage);          //线程获取图像指针
            //myThread->getCameraIndex(i);

            //if (!myThread->isRunning())
            //{
            //    myThread->start();
            //    m_pcMyCamera[i]->softTrigger();
            //    m_pcMyCamera[i]->ReadBuffer(*myImage);//读取Mat格式的图像
            //}
        }
    }
}

void Capture::stopGrabbingButtonClicked()
{
    this->startGrabbingButton->setEnabled(true);
    this->stopGrabbingButton->setEnabled(false);

    this->saveButton->setEnabled(false);
    this->scanButton->setEnabled(false);

    for (int i = 0; i < devices_num; i++)
    {
        m_pcMyCamera[i]->StopGrabbing();
        /*if (myThread->isRunning())
        {
            
            myThread->requestInterruption();
            myThread->wait();
        }*/
    }
}

void Capture::saveButtonClicked()
{
    this->saveImage();
}

void Capture::scanButtonClicked()
{
    string taskName = dataManager->getNewTask();
    qDebug() << QString::fromStdString(taskName);

    auto start = std::chrono::system_clock::now();
    this->scanToolPin(taskName,60);
    auto end = std::chrono::system_clock::now();
    std::time_t start_time = std::chrono::system_clock::to_time_t(start);
    std::time_t end_time = std::chrono::system_clock::to_time_t(end);
    std::chrono::duration<double> duration = end - start;
    qDebug() << " scanToolPin(): Start time: " << std::ctime(&start_time)
        << "End time: " << std::ctime(&end_time)
        << "Duration: " << duration.count() << " seconds.";
}

void Capture::testButtonClicked() {
    this->testExposureTime();
    //this->testAcquisitionFrameRate();
}

void Capture::softTriggerButtonClicked() {
    for (unsigned int i = 0; i < devices_num; i++)
    {

    }
}

void Capture::displayImage(QString displayPath)
{
    QImage image(displayPath);

    this->statusLabel->setText("STATUS: save image: " + displayPath);

    image = (image).scaled(300, 200, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);

    QPixmap pixmap = QPixmap::fromImage(image);
    imageDisplayLabel->setPixmap(pixmap);
}

void Capture::enumCamera()
{
    memset(m_stDevList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));    // 初始化设备信息列表  
    int nRet = MV_OK;
    nRet = CameraController::EnumDevices(MV_GIGE_DEVICE, m_stDevList);  // 枚举子网内所有设备,相机设备数量

    if (nRet != MV_OK)
        this->statusLabel->setText("STATUS: Ops, something wrong with EnumDevices().");
    
    devices_num = m_stDevList->nDeviceNum;
    if (devices_num > 0)
        this->statusLabel->setText("STATUS: Great! Number of camera: " + QString::number(devices_num));
    else
        this->statusLabel->setText("STATUS: Ops, there is no camera.");
}

void Capture::openCamera()
{
    this->statusLabel->setText("STATUS: try to open the camera.");
    
    int nRet = MV_OK;

    for (unsigned int i = 0; i < devices_num; i++)
    {
        m_pcMyCamera[i] = new CameraController();
        // 相机对象初始化
        m_pcMyCamera[i]->m_pBufForDriver = NULL;
        m_pcMyCamera[i]->m_pBufForSaveImage = NULL;
        m_pcMyCamera[i]->m_nBufSizeForDriver = 0;
        m_pcMyCamera[i]->m_nBufSizeForSaveImage = 0;
        m_pcMyCamera[i]->m_nTLayerType = m_stDevList->pDeviceInfo[i]->nTLayerType;

        nRet = m_pcMyCamera[i]->Open(m_stDevList->pDeviceInfo[i]); 

        // 设置采集模式 AcquisitionMode
        nRet = m_pcMyCamera[i]->SetEnumValue("AcquisitionMode", m_nAcquisitionMode);
        if (MV_OK != nRet)
        {
            QString hexStr = QString::number(nRet, 16);
            qDebug() << "Set AcquisitionMode failed! nRet [" << hexStr << "]";
        }
        else {
            qDebug() << "AcquisitionMode set to " << m_nAcquisitionMode;
        }

        // 设置触发模式
        //m_pcMyCamera[i]->setTriggerMode(TRIGGER_ON);
        nRet = m_pcMyCamera[i]->SetEnumValue("TriggerMode", 0);
        if (MV_OK != nRet)
        {
            QString hexStr = QString::number(nRet, 16);
            qDebug() << "Set TriggerMode failed! nRet [" << hexStr << "]";
        }
        else {
            qDebug() << "TriggerMode Off";
        }

        // 设置触发源为软触发
        m_pcMyCamera[i]->setTriggerSource(TRIGGER_SOURCE);
        if (MV_OK != nRet)
        {
            QString hexStr = QString::number(nRet, 16);
            qDebug() << "Set TriggerSource failed! nRet [" << hexStr << "]";
        }
        else {
            qDebug() << "TriggerSource Set to Software";
        }


        // 设置曝光时间
        unsigned int nExposureTime = 20000;
        nRet = m_pcMyCamera[i]->SetFloatValue("ExposureTime", nExposureTime);
        if (MV_OK != nRet) {
            qDebug() << "Set ExposureTime failed! nRet [" << nRet << "]";
        } else {
            qDebug() << "ExposureTime set to " << nExposureTime << " ms successfully.";
        }


        // 设置帧率控制
        unsigned int nFrameRate = 10;
        nRet = m_pcMyCamera[i]->SetFloatValue("AcquisitionFrameRate", nFrameRate);
        if (MV_OK != nRet) {
            qDebug() << "Set frame rate failed! nRet [" << nRet << "]";
        }else {
            qDebug() << "Frame rate set to " << nFrameRate << " fps successfully.";
        }
        nRet = m_pcMyCamera[i]->SetBoolValue("AcquisitionFrameRateEnable", true);
        if (MV_OK != nRet){
            qDebug() << "Enable frame rate control failed! nRet [" << nRet << "]";
        } else {
            qDebug() << "Frame rate control enabled successfully.";
        }


        unsigned int nOriginalWidth, nOriginalHeight;
        nRet = m_pcMyCamera[i]->GetIntValue("Width", &nOriginalWidth);
        nRet = m_pcMyCamera[i]->GetIntValue("Height", &nOriginalHeight);

    //    unsigned int nNewWidth = nOriginalWidth / 2;
    //    unsigned int nNewHeight = nOriginalHeight / 2;

    //    unsigned int nNewOffsetX = (nOriginalWidth - nNewWidth) / 2;
    //    unsigned int nNewOffsetY = (nOriginalHeight - nNewHeight) / 2;

    //    nRet = m_pcMyCamera[i]->SetIntValue( "Width", nNewWidth);
    //    if (MV_OK != nRet)
    //    {
    //        std::cout << "Set new width failed! nRet [" << nRet << "]" << std::endl;
    //    }
    //    nRet = m_pcMyCamera[i]->SetIntValue("Height", nNewHeight);
    //    if (MV_OK != nRet)
    //    {
    //        std::cout << "Set new height failed! nRet [" << nRet << "]" << std::endl;
    //    }
    //    nRet = m_pcMyCamera[i]->SetIntValue( "OffsetX", nNewOffsetX);
    //    if (MV_OK != nRet)
    //    {
    //        std::cout << "Set new offset X failed! nRet [" << nRet << "]" << std::endl;
    //    }
    //    nRet = m_pcMyCamera[i]->SetIntValue("OffsetY", nNewOffsetY);
    //    if (MV_OK != nRet)
    //    {
    //        std::cout << "Set new offset Y failed! nRet [" << nRet << "]" << std::endl;
    //    }

    }
    
    if (nRet == MV_OK)
        this->statusLabel->setText("STATUS: Great! Camera opens successfully!");
    else
        logCameraError(nRet);
}

void Capture::closeCamera()
{
    for (unsigned int i = 0; i < devices_num; i++)
    {
        m_pcMyCamera[i]->StopGrabbing();
        m_pcMyCamera[i]->Close();
    }
}

void Capture::saveImage()
{
    this->statusLabel->setText("STATUS: try to save image..");
    
    MV_FRAME_OUT_INFO_EX stImageInfo = { 0 };
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));

    unsigned int nDataLen = 0;
    int nRet = MV_OK;

    for (int i = 0; i < devices_num; i++)
    {
        // 仅在第一次保存图像时申请缓存，在 CloseDevice 时释放
        if (m_pcMyCamera[i]->m_pBufForDriver == NULL)
        {
            unsigned int nRecvBufSize = 0;
            unsigned int nRet = m_pcMyCamera[i]->GetIntValue("PayloadSize", &nRecvBufSize);

            m_pcMyCamera[i]->m_nBufSizeForDriver = nRecvBufSize;  // 一帧数据大小
            m_pcMyCamera[i]->m_pBufForDriver 
                = (unsigned char*)malloc(m_pcMyCamera[i]->m_nBufSizeForDriver);
        }

        this->statusLabel->setText("STATUS: 1");

        auto start = std::chrono::system_clock::now();
        nRet = m_pcMyCamera[i]->GetOneFrameTimeout(m_pcMyCamera[i]->m_pBufForDriver,
            &nDataLen, m_pcMyCamera[i]->m_nBufSizeForDriver, &stImageInfo, 10000);
        if (MV_OK != nRet)
        {
            qDebug() << "Get one frame failed! nRet [" << nRet << "]";
        }
        auto end = std::chrono::system_clock::now();

        std::chrono::duration<double> duration = end - start;
        auto start_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(start);
        auto end_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(end);

        auto start_epoch = start_ms.time_since_epoch();
        auto end_epoch = end_ms.time_since_epoch();

        auto start_millis = std::chrono::duration_cast<std::chrono::milliseconds>(start_epoch).count();
        auto end_millis = std::chrono::duration_cast<std::chrono::milliseconds>(end_epoch).count();

        
        qDebug() << "Image captured. Start time: " << start_millis
            << " ms, End time: " << end_millis
            << " ms.";
        qDebug() << "   Duration: " << duration.count() * 1000 << " ms.";

        if (nRet == MV_OK)
        {
            this->statusLabel->setText("STATUS: 2");

            // 仅在第一次保存图像时申请缓存，在 CloseDevice 时释放
            if (NULL == m_pcMyCamera[i]->m_pBufForSaveImage)
            {
                // BMP图片大小：width * height * 3 + 2048(预留BMP头大小)
                m_pcMyCamera[i]->m_nBufSizeForSaveImage 
                    = stImageInfo.nWidth * stImageInfo.nHeight * 3 + 2048;
                m_pcMyCamera[i]->m_pBufForSaveImage 
                    = (unsigned char*)malloc(m_pcMyCamera[i]->m_nBufSizeForSaveImage);
            }
          
            // ch:设置对应的相机参数 | en:Set camera parameter
            MV_SAVE_IMAGE_PARAM_EX stParam = { 0 };
            stParam.enImageType = MV_Image_Bmp; // ch:需要保存的图像类型 | en:Image format to save;
            stParam.enPixelType = stImageInfo.enPixelType;  // 相机对应的像素格式 | en:Pixel format
            stParam.nBufferSize = m_pcMyCamera[i]->m_nBufSizeForSaveImage;  // 存储节点的大小 | en:Buffer node size
            stParam.nWidth = stImageInfo.nWidth;         // 相机对应的宽 | en:Width
            stParam.nHeight = stImageInfo.nHeight;          // 相机对应的高 | en:Height
            stParam.nDataLen = stImageInfo.nFrameLen;
            stParam.pData = m_pcMyCamera[i]->m_pBufForDriver;
            stParam.pImageBuffer = m_pcMyCamera[i]->m_pBufForSaveImage;
            stParam.nJpgQuality = 90;       // ch:jpg编码，仅在保存Jpg图像时有效。保存BMP时SDK内忽略该参数
            
            nRet = m_pcMyCamera[i]->SaveImage(&stParam);

            this->statusLabel->setText("STATUS: 3");

            char chImageName[IMAGE_NAME_LEN] = { 0 };

            sprintf_s(chImageName, IMAGE_NAME_LEN, 
                "..\\img\\image-%d.bmp", stImageInfo.nFrameNum);

            FILE* fp = fopen(chImageName, "wb");
            fwrite(m_pcMyCamera[i]->m_pBufForSaveImage, 1, stParam.nImageLen, fp);
            fclose(fp);

            QString displayPath(chImageName);
            this->displayImage(displayPath);
            qDebug() << "displayPath: " << displayPath;
        }
        else 
        {
            this->logCameraError(nRet);
        }

    }
}

void Capture::scanToolPin(string taskName)
{
    this->statusLabel->setText("STATUS: try to scan Tool Pin..");

    MV_FRAME_OUT_INFO_EX stImageInfo = { 0 };
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));

    unsigned int nDataLen = 0;
    int nRet = MV_OK;

    for (int i = 0; i < devices_num; i++)
    {
        // 仅在第一次保存图像时申请缓存，在 CloseDevice 时释放
        if (m_pcMyCamera[i]->m_pBufForDriver == NULL)
        {
            unsigned int nRecvBufSize = 0;
            unsigned int nRet = m_pcMyCamera[i]->GetIntValue("PayloadSize", &nRecvBufSize);

            m_pcMyCamera[i]->m_nBufSizeForDriver = nRecvBufSize;  // 一帧数据大小
            m_pcMyCamera[i]->m_pBufForDriver
                = (unsigned char*)malloc(m_pcMyCamera[i]->m_nBufSizeForDriver);
        }

        for (int j = 0; j < 180; j++)
        {
            nRet = m_pcMyCamera[i]->GetOneFrameTimeout(m_pcMyCamera[i]->m_pBufForDriver,
                &nDataLen, m_pcMyCamera[i]->m_nBufSizeForDriver, &stImageInfo, 10000000);

            if (nRet == MV_OK)
            {
                // 仅在第一次保存图像时申请缓存，在 CloseDevice 时释放
                if (NULL == m_pcMyCamera[i]->m_pBufForSaveImage)
                {
                    // BMP图片大小：width * height * 3 + 2048(预留BMP头大小)
                    m_pcMyCamera[i]->m_nBufSizeForSaveImage
                        = stImageInfo.nWidth * stImageInfo.nHeight * 3 + 2048;
                    m_pcMyCamera[i]->m_pBufForSaveImage
                        = (unsigned char*)malloc(m_pcMyCamera[i]->m_nBufSizeForSaveImage);
                }

                // ch:设置对应的相机参数 | en:Set camera parameter
                MV_SAVE_IMAGE_PARAM_EX stParam = { 0 };
                stParam.enImageType = MV_Image_Bmp; // ch:需要保存的图像类型 | en:Image format to save;
                stParam.enPixelType = stImageInfo.enPixelType;  // 相机对应的像素格式 | en:Pixel format
                stParam.nBufferSize = m_pcMyCamera[i]->m_nBufSizeForSaveImage;  // 存储节点的大小 | en:Buffer node size
                stParam.nWidth = stImageInfo.nWidth;         // 相机对应的宽 | en:Width
                stParam.nHeight = stImageInfo.nHeight;          // 相机对应的高 | en:Height
                stParam.nDataLen = stImageInfo.nFrameLen;
                stParam.pData = m_pcMyCamera[i]->m_pBufForDriver;
                stParam.pImageBuffer = m_pcMyCamera[i]->m_pBufForSaveImage;
                stParam.nJpgQuality = 90;       // ch:jpg编码，仅在保存Jpg图像时有效。保存BMP时SDK内忽略该参数

                nRet = m_pcMyCamera[i]->SaveImage(&stParam);

                char chImageName[IMAGE_NAME_LEN] = { 0 };
                sprintf_s(chImageName, IMAGE_NAME_LEN,
                    "..\\img\\%s\\img-%d.bmp", taskName.c_str(), stImageInfo.nFrameNum);

                FILE* fp = fopen(chImageName, "wb");
                fwrite(m_pcMyCamera[i]->m_pBufForSaveImage, 1, stParam.nImageLen, fp);
                fclose(fp);

                //this->displayImage(displayPath);
                //qDebug() << "imagePath: " << displayPath;

                /*                
                typedef struct _MV_SAVE_IMG_TO_FILE_PARAM_
                {
                    enum MvGvspPixelType    enPixelType;                            ///< [IN]  \~chinese输入数据的像素格式      \~english The pixel format of the input data
                    unsigned char* pData;                                  ///< [IN]  \~chinese 输入数据缓存           \~english Input Data Buffer
                    unsigned int            nDataLen;                               ///< [IN]  \~chinese 输入数据长度           \~english Input Data length
                    unsigned short          nWidth;                                 ///< [IN]  \~chinese 图像宽                 \~english Image Width
                    unsigned short          nHeight;                                ///< [IN]  \~chinese 图像高                 \~english Image Height
                    enum MV_SAVE_IAMGE_TYPE enImageType;                            ///< [IN]  \~chinese 输入图片格式           \~english Input Image Format
                    unsigned int            nQuality;                               ///< [IN]  \~chinese JPG编码质量(50-99]，PNG编码质量[0-9]，其它格式无效 \~english JPG Encoding quality(50-99],PNG Encoding quality[0-9]，Other formats are invalid
                    char                    pImagePath[256];                        ///< [IN]  \~chinese 输入文件路径           \~english Input file path

                    int                     iMethodValue;                           ///< [IN]  \~chinese 插值方法 0-快速 1-均衡 2-最优（其它值默认为最优）  \~english Bayer interpolation method  0-Fast 1-Equilibrium 2-Optimal

                    unsigned int            nReserved[8];                           ///<       \~chinese 预留                   \~english Reserved

                }MV_SAVE_IMG_TO_FILE_PARAM;
                */

                /*
                MV_SAVE_IMG_TO_FILE_PARAM sfParam = { 0 };
                sfParam.enPixelType = stImageInfo.enPixelType;
                */
                

            }
            else
            {
                this->logCameraError(nRet);
            }
        }
        
        this->stopGrabbingButtonClicked();
    }
}

void Capture::scanToolPin(string taskName, const int images_num)
{
    this->statusLabel->setText("STATUS: try to scan Tool Pin..");

    MV_FRAME_OUT_INFO_EX stImageInfo = { 0 };
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));

    unsigned int nDataLen = 0;
    int nRet = MV_OK;

    // 定义一个数组来存储所有的图像参数
    MV_SAVE_IMAGE_PARAM_EX* imageParams = new MV_SAVE_IMAGE_PARAM_EX[devices_num * images_num];
    int paramIndex = 0;

    for (int i = 0; i < devices_num; i++)
    {
        // 仅在第一次保存图像时申请缓存，在 CloseDevice 时释放
        if (m_pcMyCamera[i]->m_pBufForDriver == NULL)
        {
            unsigned int nRecvBufSize = 0;
            unsigned int nRet = m_pcMyCamera[i]->GetIntValue("PayloadSize", &nRecvBufSize);

            m_pcMyCamera[i]->m_nBufSizeForDriver = nRecvBufSize;  // 一帧数据大小
            m_pcMyCamera[i]->m_pBufForDriver
                = (unsigned char*)malloc(m_pcMyCamera[i]->m_nBufSizeForDriver);
        }

        for (int j = 0; j < images_num; j++)
        {
            auto start = std::chrono::system_clock::now();
            nRet = m_pcMyCamera[i]->GetOneFrameTimeout(m_pcMyCamera[i]->m_pBufForDriver,
                &nDataLen, m_pcMyCamera[i]->m_nBufSizeForDriver, &stImageInfo, 10000000);
            auto end = std::chrono::system_clock::now();

            std::chrono::duration<double> duration = end - start;
            auto start_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(start);
            auto end_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(end);

            auto start_epoch = start_ms.time_since_epoch();
            auto end_epoch = end_ms.time_since_epoch();

            auto start_millis = std::chrono::duration_cast<std::chrono::milliseconds>(start_epoch).count();
            auto end_millis = std::chrono::duration_cast<std::chrono::milliseconds>(end_epoch).count();

            qDebug() << "Image " << paramIndex 
                << " captured. Start time: " << start_millis
                << " ms, End time: " << end_millis 
                << " ms, Duration: " << duration.count() * 1000 << " ms.";

            if (nRet == MV_OK)
            {
                // 仅在第一次保存图像时申请缓存，在 CloseDevice 时释放
                if (NULL == m_pcMyCamera[i]->m_pBufForSaveImage)
                {
                    // BMP图片大小：width * height * 3 + 2048(预留BMP头大小)
                    m_pcMyCamera[i]->m_nBufSizeForSaveImage
                        = stImageInfo.nWidth * stImageInfo.nHeight * 3 + 2048;
                    m_pcMyCamera[i]->m_pBufForSaveImage
                        = (unsigned char*)malloc(m_pcMyCamera[i]->m_nBufSizeForSaveImage);
                }

                
                MV_SAVE_IMAGE_PARAM_EX stParam = { 0 };
                stParam.enImageType = MV_Image_Bmp;
                stParam.enPixelType = stImageInfo.enPixelType;  
                stParam.nBufferSize = m_pcMyCamera[i]->m_nBufSizeForSaveImage; 
                stParam.nWidth = stImageInfo.nWidth;        
                stParam.nHeight = stImageInfo.nHeight;        
                stParam.nDataLen = stImageInfo.nFrameLen;
                stParam.pData = m_pcMyCamera[i]->m_pBufForDriver;
                // 为每张图片分配独立的内存空间
                stParam.pImageBuffer = new unsigned char[m_pcMyCamera[i]->m_nBufSizeForSaveImage];
                memcpy(stParam.pImageBuffer, m_pcMyCamera[i]->m_pBufForSaveImage, m_pcMyCamera[i]->m_nBufSizeForSaveImage);
                stParam.nJpgQuality = 90;       

                nRet = m_pcMyCamera[i]->SaveImage(&stParam);

                // 将参数保存到数组中
                imageParams[paramIndex++] = stParam;

                qDebug() << "Image " << paramIndex << " saved (" << stImageInfo.nFrameNum << ")";
            }
            else
            {
                this->logCameraError(nRet);
            }
        }

        this->stopGrabbingButtonClicked();
    }

    // 所有照片拍摄完成后，依次保存到文件
    for (int k = 0; k < paramIndex; k++)
    {
        char chImageName[IMAGE_NAME_LEN] = { 0 };
        sprintf_s(chImageName, IMAGE_NAME_LEN,
            "..\\img\\%s\\img-%d.bmp", taskName.c_str(), k + 1);

        FILE* fp = fopen(chImageName, "wb");
        if (fp != NULL)
        {
            fwrite(imageParams[k].pImageBuffer, 1, imageParams[k].nImageLen, fp);
            fclose(fp);

            qDebug() << "Image " << k << " saved in " << chImageName;
        }
    }

    // 释放内存
    for (int k = 0; k < paramIndex; k++)
    {
        delete[] imageParams[k].pImageBuffer;
    }
    delete[] imageParams;
}

void Capture::logCameraError(int nRet)
{
    switch ( nRet)
    {
    case MV_E_HANDLE:
        this->statusLabel->setText("STATUS: Ops, GetOneFrameTimeout() failed. nRet = MV_E_HANDLE");
    case MV_E_SUPPORT:
        this->statusLabel->setText("STATUS: Ops, GetOneFrameTimeout() failed. nRet = MV_E_SUPPORT");
    case MV_E_BUFOVER:
        this->statusLabel->setText("STATUS: Ops, GetOneFrameTimeout() failed. nRet = MV_E_BUFOVER ");
    case MV_E_CALLORDER:
        this->statusLabel->setText("STATUS: Ops, GetOneFrameTimeout() failed. nRet = MV_E_CALLORDER ");
    
    case MV_E_PARAMETER:
        this->statusLabel->setText("STATUS: Ops, GetOneFrameTimeout() failed. nRet = MV_E_PARAMETER ");
    case MV_E_RESOURCE:
        this->statusLabel->setText("STATUS: Ops, GetOneFrameTimeout() failed. nRet = MV_E_RESOURCE ");
    case MV_E_NODATA:
        this->statusLabel->setText("STATUS: Ops, GetOneFrameTimeout() failed. nRet = MV_E_NODATA  ");
    case MV_E_PRECONDITION:
        this->statusLabel->setText("STATUS: Ops, GetOneFrameTimeout() failed. nRet = MV_E_PRECONDITION  ");

    case MV_E_VERSION:
        this->statusLabel->setText("STATUS: Ops, GetOneFrameTimeout() failed. nRet = MV_E_VERSION  ");
    case MV_E_NOENOUGH_BUF:
        this->statusLabel->setText("STATUS: Ops, GetOneFrameTimeout() failed. nRet = MV_E_NOENOUGH_BUF  ");
    case MV_E_ABNORMAL_IMAGE:
        this->statusLabel->setText("STATUS: Ops, GetOneFrameTimeout() failed. nRet = MV_E_ABNORMAL_IMAGE   ");
    case MV_E_LOAD_LIBRARY:
        this->statusLabel->setText("STATUS: Ops, GetOneFrameTimeout() failed. nRet = MV_E_LOAD_LIBRARY   ");

    case MV_E_NOOUTBUF:
        this->statusLabel->setText("STATUS: Ops, GetOneFrameTimeout() failed. nRet = MV_E_NOOUTBUF   ");
    case MV_E_ENCRYPT:
        this->statusLabel->setText("STATUS: Ops, GetOneFrameTimeout() failed. nRet = MV_E_ENCRYPT  ");
    case MV_E_OPENFILE:
        this->statusLabel->setText("STATUS: Ops, GetOneFrameTimeout() failed. nRet = MV_E_OPENFILE    ");
    case MV_E_UNKNOW:
        this->statusLabel->setText("STATUS: Ops, GetOneFrameTimeout() failed. nRet = MV_E_UNKNOW    ");


    default:
        std::stringstream ss;
        ss << std::hex << nRet;
        
        std::string str = ss.str();
        std::cout << str << std::endl;
    }
}

void Capture::testExposureTime() {

    this->statusLabel->setText("STATUS: test exposure time ..");

    MV_FRAME_OUT_INFO_EX stImageInfo = { 0 };
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));

    unsigned int nDataLen = 0;
    int nRet = MV_OK;

    std::vector<int> exposureTimes = {50, 50, 50, 100, 100, 500, 1000, 5000, 5555};
    MVCC_FLOATVALUE exposureTime;



    for (int i = 0; i < devices_num; i++)
    {
        // 触发源设为"Counter0"
        //m_pcMyCamera[i]->setTriggerSource(4);
        //if (MV_OK != nRet) {
        //    QString hexStr = QString::number(nRet, 16);
        //    qDebug() << "Set TriggerSource failed! nRet [" << hexStr << "]";
        //}else {
        //    qDebug() << "TriggerSource Set to Counter0";
        //}

        

        // 仅在第一次保存图像时申请缓存，在 CloseDevice 时释放
        if (m_pcMyCamera[i]->m_pBufForDriver == NULL)
        {
            unsigned int nRecvBufSize = 0;
            unsigned int nRet = m_pcMyCamera[i]->GetIntValue("PayloadSize", &nRecvBufSize);

            m_pcMyCamera[i]->m_nBufSizeForDriver = nRecvBufSize;  // 一帧数据大小
            m_pcMyCamera[i]->m_pBufForDriver
                = (unsigned char*)malloc(m_pcMyCamera[i]->m_nBufSizeForDriver);
        }


        for (int j = 0; j < exposureTimes.size(); j++)
        {
            m_pcMyCamera[i]->SetFloatValue("ExposureTime", exposureTimes[j]);
            
            auto start = std::chrono::system_clock::now();
            nRet = m_pcMyCamera[i]->GetOneFrameTimeout(m_pcMyCamera[i]->m_pBufForDriver,
                &nDataLen, m_pcMyCamera[i]->m_nBufSizeForDriver, &stImageInfo, 10000);
            //nRet = m_pcMyCamera[i]->softTrigger();
            if (MV_OK != nRet)
            {
                qDebug() << "Get one frame failed! nRet [" << nRet << "]";
            }
            auto end = std::chrono::system_clock::now();

            std::chrono::duration<double> duration = end - start;
            auto start_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(start);
            auto end_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(end);

            auto start_epoch = start_ms.time_since_epoch();
            auto end_epoch = end_ms.time_since_epoch();

            auto start_millis = std::chrono::duration_cast<std::chrono::milliseconds>(start_epoch).count();
            auto end_millis = std::chrono::duration_cast<std::chrono::milliseconds>(end_epoch).count();

            nRet = m_pcMyCamera[i]->GetFloatValue("ExposureTime", &exposureTime);
            qDebug() << "Image " << j
                << " ( exposure time: " << exposureTime.fCurValue
                << ") captured. Start time: " << start_millis
                << " ms, End time: " << end_millis
                << " ms.";
            qDebug() << "   Duration: " << duration.count() * 1000 << " ms.";
                

            

            if (nRet == MV_OK)
            {
                // 仅在第一次保存图像时申请缓存，在 CloseDevice 时释放
                if (NULL == m_pcMyCamera[i]->m_pBufForSaveImage)
                {
                    // BMP图片大小：width * height * 3 + 2048(预留BMP头大小)
                    m_pcMyCamera[i]->m_nBufSizeForSaveImage
                        = stImageInfo.nWidth * stImageInfo.nHeight * 3 + 2048;
                    m_pcMyCamera[i]->m_pBufForSaveImage
                        = (unsigned char*)malloc(m_pcMyCamera[i]->m_nBufSizeForSaveImage);
                }


                MV_SAVE_IMAGE_PARAM_EX stParam = { 0 };
                stParam.enImageType = MV_Image_Bmp;
                stParam.enPixelType = stImageInfo.enPixelType;
                stParam.nBufferSize = m_pcMyCamera[i]->m_nBufSizeForSaveImage;
                stParam.nWidth = stImageInfo.nWidth;
                stParam.nHeight = stImageInfo.nHeight;
                stParam.nDataLen = stImageInfo.nFrameLen;
                stParam.pData = m_pcMyCamera[i]->m_pBufForDriver;
                // 为每张图片分配独立的内存空间
                stParam.pImageBuffer = new unsigned char[m_pcMyCamera[i]->m_nBufSizeForSaveImage];
                memcpy(stParam.pImageBuffer, m_pcMyCamera[i]->m_pBufForSaveImage, m_pcMyCamera[i]->m_nBufSizeForSaveImage);
                stParam.nJpgQuality = 90;

                nRet = m_pcMyCamera[i]->SaveImage(&stParam);

                //qDebug() << "Image " << j << " saved in imageParams[]";
            }
            else
            {
                this->logCameraError(nRet);
            }
        }

        this->stopGrabbingButtonClicked();
    }

}

void Capture::testAcquisitionFrameRate() {

    this->statusLabel->setText("STATUS: test acquisition frame rate ..");

    MV_FRAME_OUT_INFO_EX stImageInfo = { 0 };
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));

    unsigned int nDataLen = 0;
    int nRet = MV_OK;

    std::vector<float> frameRates = { 1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21};
    MVCC_FLOATVALUE frameRate;
    MVCC_FLOATVALUE exposureTime;

    for (int i = 0; i < devices_num; i++)
    {
        // 仅在第一次保存图像时申请缓存，在 CloseDevice 时释放
        if (m_pcMyCamera[i]->m_pBufForDriver == NULL)
        {
            unsigned int nRecvBufSize = 0;
            unsigned int nRet = m_pcMyCamera[i]->GetIntValue("PayloadSize", &nRecvBufSize);

            m_pcMyCamera[i]->m_nBufSizeForDriver = nRecvBufSize;  // 一帧数据大小
            m_pcMyCamera[i]->m_pBufForDriver
                = (unsigned char*)malloc(m_pcMyCamera[i]->m_nBufSizeForDriver);
        }


        for (int j = 0; j < frameRates.size(); j++)
        {
            nRet = m_pcMyCamera[i]->SetBoolValue("AcquisitionFrameRateEnable", true);
            if (MV_OK != nRet)
            {
                qDebug() << "Enable frame rate control failed! nRet [" << nRet << "]" ;
            } 
            else {
                qDebug() << "Frame rate control enabled successfully.";
            }
            
            nRet = m_pcMyCamera[i]->SetFloatValue("AcquisitionFrameRate", frameRates[j]);
            if (MV_OK != nRet)
            {
                qDebug() << "Set frame rate failed! nRet [" << nRet << "]" ;
            }
            else
            {
                qDebug() << "Frame rate set to " << frameRates[j] << " fps successfully." ;
            }

            auto start = std::chrono::system_clock::now();
            nRet = m_pcMyCamera[i]->GetOneFrameTimeout(m_pcMyCamera[i]->m_pBufForDriver,
                &nDataLen, m_pcMyCamera[i]->m_nBufSizeForDriver, &stImageInfo, 10000000);
            if (MV_OK != nRet)
            {
                qDebug() << "Get one frame failed! nRet [" << nRet << "]";
            }
            auto end = std::chrono::system_clock::now();

            std::chrono::duration<double> duration = end - start;
            auto start_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(start);
            auto end_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(end);

            auto start_epoch = start_ms.time_since_epoch();
            auto end_epoch = end_ms.time_since_epoch();

            auto start_millis = std::chrono::duration_cast<std::chrono::milliseconds>(start_epoch).count();
            auto end_millis = std::chrono::duration_cast<std::chrono::milliseconds>(end_epoch).count();

            nRet = m_pcMyCamera[i]->GetFloatValue("AcquisitionFrameRate", &frameRate);
            nRet = m_pcMyCamera[i]->GetFloatValue("ExposureTime", &exposureTime);
            qDebug() << "Image " << j
                << " ( frame rate: " << frameRate.fCurValue
                << " , exposure time: " << exposureTime.fCurValue
                << " ) captured. Start time: " << start_millis
                << " ms, End time: " << end_millis
                << " ms.";
            qDebug() << "   Duration: " << duration.count() * 1000 << " ms.";




            if (nRet == MV_OK)
            {
                // 仅在第一次保存图像时申请缓存，在 CloseDevice 时释放
                if (NULL == m_pcMyCamera[i]->m_pBufForSaveImage)
                {
                    // BMP图片大小：width * height * 3 + 2048(预留BMP头大小)
                    m_pcMyCamera[i]->m_nBufSizeForSaveImage
                        = stImageInfo.nWidth * stImageInfo.nHeight * 3 + 2048;
                    m_pcMyCamera[i]->m_pBufForSaveImage
                        = (unsigned char*)malloc(m_pcMyCamera[i]->m_nBufSizeForSaveImage);
                }


                MV_SAVE_IMAGE_PARAM_EX stParam = { 0 };
                stParam.enImageType = MV_Image_Bmp;
                stParam.enPixelType = stImageInfo.enPixelType;
                stParam.nBufferSize = m_pcMyCamera[i]->m_nBufSizeForSaveImage;
                stParam.nWidth = stImageInfo.nWidth;
                stParam.nHeight = stImageInfo.nHeight;
                stParam.nDataLen = stImageInfo.nFrameLen;
                stParam.pData = m_pcMyCamera[i]->m_pBufForDriver;
                // 为每张图片分配独立的内存空间
                stParam.pImageBuffer = new unsigned char[m_pcMyCamera[i]->m_nBufSizeForSaveImage];
                memcpy(stParam.pImageBuffer, m_pcMyCamera[i]->m_pBufForSaveImage, m_pcMyCamera[i]->m_nBufSizeForSaveImage);
                stParam.nJpgQuality = 90;

                nRet = m_pcMyCamera[i]->SaveImage(&stParam);

                //qDebug() << "Image " << j << " saved in imageParams[]";
            }
            else
            {
                this->logCameraError(nRet);
            }
        }

        this->stopGrabbingButtonClicked();
    }

}