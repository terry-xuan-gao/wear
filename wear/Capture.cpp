#include "Capture.h"

#define TRIGGER_SOURCE   7
#define TRIGGER_ON       1
#define TRIGGER_OFF      0
#define IMAGE_NAME_LEN   64

Capture::Capture(QWidget* parent)
    : QWidget(parent)
{
    this->resize(400, 300);

    this->initStatusLabel();
    this->initButtons();

    this->myThread = new MyThread;
}

Capture::~Capture()
{

}

void Capture::initButtons()
{
    this->startButton = new QPushButton("START");
    this->ansysButton = new QPushButton("ANSYS");

    this->enumButton = new QPushButton("ENUM");

    this->openButton = new QPushButton("OPEN CAMERA");
    this->closeButton = new QPushButton("CLOSE CAMERA");

    this->continueModeSetButton = new QPushButton("CONTINUE MODE");
    this->triggerModeSetButton = new QPushButton("TRIGGER MODE");

    this->startGrabbingButton = new QPushButton("START GRABBING");
    this->stopGrabbingButton = new QPushButton("STOP GRABBING");
    this->saveButton = new QPushButton("SAVE IMAGE");

    connect(this->startButton, &QPushButton::clicked,
        this, &Capture::startButtonClicked);
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

    layout = new QVBoxLayout();
    layout->addWidget(startButton);
    layout->addWidget(ansysButton);
    this->setLayout(layout);
}

void Capture::initStatusLabel()
{
    this->statusLabel = new QLabel("STATUS: OK", this);
    this->statusLabel->setGeometry(0, this->height() - 30, this->width(), 30);
    this->statusLabel->setAlignment(Qt::AlignBottom | Qt::AlignLeft);
    this->statusLabel->show();
}

void Capture::startButtonClicked()
{
    this->statusLabel->setText("STATUS: prepare for capturing!");
    this->ansysButton->setVisible(false);

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
    
    this->layout->addWidget(saveButton);

    this->openButton->setEnabled(false);
    this->closeButton->setEnabled(false);
    this->continueModeSetButton->setEnabled(false);
    this->triggerModeSetButton->setEnabled(false);
    this->startGrabbingButton->setEnabled(false);
    this->stopGrabbingButton->setEnabled(false);
    this->saveButton->setEnabled(false);

}

void Capture::enumButtonClicked()
{
    this->enumCamera();
    if (devices_num > 0)
        this->openButton->setEnabled(true);
}

void Capture::openButtonClicked()
{
    this->openButton->setEnabled(false);
    this->closeButton->setEnabled(true);
    this->triggerModeSetButton->setEnabled(true);
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
}

void Capture::startGrabbingButtonClicked()
{
    m_bContinueStarted = 1; // 为触发模式标记一下，切换触发模式时先执行停止采集图像函数

    this->startGrabbingButton->setEnabled(false);
    this->stopGrabbingButton->setEnabled(true);

    this->saveButton->setEnabled(true);

    // 先判断什么模式，再判断是否正在采集
    if (m_nTriggerMode == TRIGGER_ON)
    {
        for (int i = 0; i < devices_num; i++)
        {
            //开启相机采集
            m_pcMyCamera[i]->StartGrabbing();

            myThread->getCameraPtr(m_pcMyCamera[i]); //线程获取相机指针
            myThread->getImagePtr(myImage);          //线程获取图像指针
            myThread->getCameraIndex(i);

            if (!myThread->isRunning())
            {
                myThread->start();
                m_pcMyCamera[i]->softTrigger();
                m_pcMyCamera[i]->ReadBuffer(*myImage);//读取Mat格式的图像
            }
        }
    }
}

void Capture::stopGrabbingButtonClicked()
{
    this->startGrabbingButton->setEnabled(true);
    this->stopGrabbingButton->setEnabled(false);

    for (int i = 0; i < devices_num; i++)
    {
        if (myThread->isRunning())
        {
            m_pcMyCamera[i]->StopGrabbing();
            myThread->requestInterruption();
            myThread->wait();
        }
    }
}

void Capture::saveButtonClicked()
{
    this->saveImage();
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

int Capture::openCamera()
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

        nRet = m_pcMyCamera[i]->Open(m_stDevList->pDeviceInfo[i]); //打开相机
        //设置触发模式
        m_pcMyCamera[i]->setTriggerMode(TRIGGER_ON);
        //设置触发源为软触发
        m_pcMyCamera[i]->setTriggerSource(TRIGGER_SOURCE);
    }
    
    if (nRet == MV_OK)
        this->statusLabel->setText("STATUS: Great! Camera opens successfully!");
    else
        logCameraError(nRet);

    return nRet;
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

        nRet = m_pcMyCamera[i]->GetOneFrameTimeout(m_pcMyCamera[i]->m_pBufForDriver, 
            &nDataLen, m_pcMyCamera[i]->m_nBufSizeForDriver, &stImageInfo, 10000000);

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
            sprintf_s(chImageName, IMAGE_NAME_LEN, "image_%d.bmp", stImageInfo.nHostTimeStamp);

            FILE* fp = fopen(chImageName, "wb");
            fwrite(m_pcMyCamera[i]->m_pBufForSaveImage, 1, stParam.nImageLen, fp);
            this->statusLabel->setText("STATUS: save image: " + QString::number(stImageInfo.nHostTimeStamp));
            fclose(fp);
        }
        else 
        {
            this->logCameraError(nRet);
        }

    }
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

