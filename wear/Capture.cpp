#include "Capture.h"

#define TRIGGER_SOURCE   7
#define TRIGGER_ON       1
#define TRIGGER_OFF      0
#define IMAGE_NAME_LEN   64

Capture::Capture(QWidget* parent)
    : QWidget(parent)
{
    this->resize(800, 600);

    this->initStatusLabel();
    this->initButtons();
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
    this->startGrabbingButton = new QPushButton("START GRABBING");
    this->stopGrabbingButton = new QPushButton("STOP GRABBING");
    this->saveButton = new QPushButton("SAVE IMAGE");

    connect(this->startButton, &QPushButton::clicked,
        this, &Capture::startButtonClicked);
    connect(this->enumButton, &QPushButton::clicked,
        this, &Capture::enumCamera);
    connect(this->openButton, &QPushButton::clicked,
        this, &Capture::openCamera);


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

void Capture::captureTask()
{
    this->statusLabel->setText("STATUS: try to connect camera...");
    
    int nRet = MV_OK;
    
    this->enumCamera();


    nRet = this->openCamera();
    if (nRet != MV_OK)
        return;

    nRet = m_pcMyCamera[0]->StartGrabbing();
    if (nRet != MV_OK)
        this->logCameraError(nRet);

    this->saveImage();

    nRet = m_pcMyCamera[0]->StopGrabbing();
    this->closeCamera();
}

void Capture::startButtonClicked()
{
    this->statusLabel->setText("STATUS: prepare for capturing!");
    this->ansysButton->setVisible(false);

    this->layout->addWidget(enumButton);
    this->layout->addWidget(openButton);
    this->layout->addWidget(startGrabbingButton);
    this->layout->addWidget(stopGrabbingButton);
    this->layout->addWidget(saveButton);
    
    //this->captureTask();
}

void Capture::enumCamera()
{
    memset(m_stDevList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));    // ��ʼ���豸��Ϣ�б�  
    int nRet = MV_OK;
    nRet = CameraController::EnumDevices(MV_GIGE_DEVICE, m_stDevList);  // ö�������������豸,����豸����

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
        // ��������ʼ��
        m_pcMyCamera[i]->m_pBufForDriver = NULL;
        m_pcMyCamera[i]->m_pBufForSaveImage = NULL;
        m_pcMyCamera[i]->m_nBufSizeForDriver = 0;
        m_pcMyCamera[i]->m_nBufSizeForSaveImage = 0;
        m_pcMyCamera[i]->m_nTLayerType = m_stDevList->pDeviceInfo[i]->nTLayerType;

        nRet = m_pcMyCamera[i]->Open(m_stDevList->pDeviceInfo[i]); //�����
        //���ô���ģʽ
        m_pcMyCamera[i]->setTriggerMode(TRIGGER_ON);
        //���ô���ԴΪ����
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
        // ���ڵ�һ�α���ͼ��ʱ���뻺�棬�� CloseDevice ʱ�ͷ�
        if (m_pcMyCamera[i]->m_pBufForDriver == NULL)
        {
            unsigned int nRecvBufSize = 0;
            unsigned int nRet = m_pcMyCamera[i]->GetIntValue("PayloadSize", &nRecvBufSize);

            m_pcMyCamera[i]->m_nBufSizeForDriver = nRecvBufSize;  // һ֡���ݴ�С
            m_pcMyCamera[i]->m_pBufForDriver 
                = (unsigned char*)malloc(m_pcMyCamera[i]->m_nBufSizeForDriver);
        }

        this->statusLabel->setText("STATUS: 1");

        nRet = m_pcMyCamera[i]->GetOneFrameTimeout(m_pcMyCamera[i]->m_pBufForDriver, 
            &nDataLen, m_pcMyCamera[i]->m_nBufSizeForDriver, &stImageInfo, 10000000);

    
        if (nRet == MV_OK)
        {
            this->statusLabel->setText("STATUS: 2");

            // ���ڵ�һ�α���ͼ��ʱ���뻺�棬�� CloseDevice ʱ�ͷ�
            if (NULL == m_pcMyCamera[i]->m_pBufForSaveImage)
            {
                // BMPͼƬ��С��width * height * 3 + 2048(Ԥ��BMPͷ��С)
                m_pcMyCamera[i]->m_nBufSizeForSaveImage 
                    = stImageInfo.nWidth * stImageInfo.nHeight * 3 + 2048;
                m_pcMyCamera[i]->m_pBufForSaveImage 
                    = (unsigned char*)malloc(m_pcMyCamera[i]->m_nBufSizeForSaveImage);
            }
          
            // ch:���ö�Ӧ��������� | en:Set camera parameter
            MV_SAVE_IMAGE_PARAM_EX stParam = { 0 };
            stParam.enImageType = MV_Image_Bmp; // ch:��Ҫ�����ͼ������ | en:Image format to save;
            stParam.enPixelType = stImageInfo.enPixelType;  // �����Ӧ�����ظ�ʽ | en:Pixel format
            stParam.nBufferSize = m_pcMyCamera[i]->m_nBufSizeForSaveImage;  // �洢�ڵ�Ĵ�С | en:Buffer node size
            stParam.nWidth = stImageInfo.nWidth;         // �����Ӧ�Ŀ� | en:Width
            stParam.nHeight = stImageInfo.nHeight;          // �����Ӧ�ĸ� | en:Height
            stParam.nDataLen = stImageInfo.nFrameLen;
            stParam.pData = m_pcMyCamera[i]->m_pBufForDriver;
            stParam.pImageBuffer = m_pcMyCamera[i]->m_pBufForSaveImage;
            stParam.nJpgQuality = 90;       // ch:jpg���룬���ڱ���Jpgͼ��ʱ��Ч������BMPʱSDK�ں��Ըò���
            
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

