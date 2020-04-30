#ifndef PX2CAMLIB_H
#define PX2CAMLIB_H

#include <iostream>
#include <thread>

// Core
#include <dw/core/Context.h>
#include <dw/core/Logger.h>
#include <dw/core/VersionCurrent.h>
#include <dw/core/NvMedia.h>

// HAL
#include <dw/sensors/Sensors.h>
#include <dw/sensors/SensorSerializer.h>
#include <dw/sensors/camera/Camera.h>

// Image
#include <dw/image/ImageStreamer.h>

// Renderer
#include <dw/renderer/Renderer.h>

// DW Framework
#include <framework/ProgramArguments.hpp>
#include <framework/WindowGLFW.hpp>
#include <framework/Checks.hpp>
#include <dw/renderer/RenderEngine.h>

#include <dw/isp/SoftISP.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "common_cv.h"

#include "img_dev.h"

#define CAM_IMG_WIDTH 1920
#define CAM_IMG_HEIGHT 1208
#define CUDA_PITCH 7680
#define CAM_IMG_SIZE CAM_IMG_WIDTH*CAM_IMG_HEIGHT

using namespace std;

typedef enum { MASTER_TEGRA = 0,
               SLAVE_TEGRA = 1
}dwTegraMode;

typedef enum {GMSL_CAM_YUV = 0,
              GMSL_CAM_RAW = 1,
              H264_FILE = 2,
              RAW_FILE = 3
}dwCamInputMode;

typedef struct {
    dwCamInputMode camInputMode;
    string filePath = "";
}camInputParameters;


typedef struct {
    float resizeRatio = 1.f;
    int roiX = 0;
    int roiY = 0;
    int roiW = CAM_IMG_WIDTH;
    int roiH = CAM_IMG_HEIGHT;
}imgCropParameters;

typedef struct {
    bool onDisplay = true;
    string windowTitle = "";
    int windowWidth = 1280;
    int windowHeight = 720;
}displayParameters;

typedef struct{
    uint64_t timestamp_us = 0;
    float* trtImg;
}trtImgData;

typedef struct{
    uint64_t timestamp_us = 0;
    cv::Mat matImg;
}matImgData;

// combine by camera sensor, which might have camera siblings
struct Camera {
    dwSensorHandle_t sensor;
    uint32_t numSiblings;
    uint32_t width;
    uint32_t height;
    dwImageStreamerHandle_t streamer; // different streamers to support different resolutions
    dwCameraProperties cameraProperties;
    std::vector<dwImageCUDA*> camImgCuda_PerCams;
    std::vector<dwImageCUDA*> camImgCudaRaw_PerCams;
    std::vector<dwImageCUDA*> camImgCudaRCB_PerCams;

    std::vector<dwCameraFrameHandle_t> frame_PerCams;
    std::vector<dwImageHandle_t> frameCuda_PerCams;
    std::vector<dwImageHandle_t> rawImageHandle_PerCams;
    std::vector<dwSoftISPHandle_t> ISP_PerCams;
    std::vector<uint8_t*> pitchedImgCudaRGBA_PerCams;
    std::vector<uint8_t*> gpuMatData_PerCams;
    std::vector<cv::cuda::GpuMat> gpuMat_PerCams;
    std::vector<uint8_t*> gpuResizedMatData_PerCams;
    std::vector<cv::cuda::GpuMat> gpuResizedMat_PerCams;
    std::vector<uint8_t*> cpuMatData_PerCams;
    std::vector<cv::Mat> cpuMat_PerCams;
    std::vector<cudaStream_t> cudaStream_PerCams;
    std::vector<sensor_msgs::Image> toPublishImage_PerCams;
    std::vector<ros::Publisher> imagePublisher_PerCams;
};


class px2Cam
{
public:
    px2Cam();
    ~px2Cam();

public:
    bool SetRosParams(ros::NodeHandle* nodeH,
                      std::vector<std::string> publishImageTopicNames,
                      float pulbishImageResizeRatio=0.5f,
                      std::string cameraType_ab = "",
                      std::string cameraType_cd = "",
                      std::string cameraType_ef = "",
                      std::string selectorMask = "")
    {
        mNodeH = nodeH;
        mPublishImageTopicNames = publishImageTopicNames;
        mPublishImageSize.width = (int)(pulbishImageResizeRatio*CAM_IMG_WIDTH);
        mPublishImageSize.height = (int)(pulbishImageResizeRatio*CAM_IMG_HEIGHT);
        mCameraType_ab = cameraType_ab;
        mCameraType_cd = cameraType_cd;
        mCameraType_ef = cameraType_ef;
        mSelectorMask = selectorMask;
    }


    bool Init();

    bool UpdateAllCamImgs();
    void PublishAllImages();
    
    // For multi-threading
    static void* PublishTreadCallback(void* thiz_void, uint portIdx, uint siblingIdx)
    {
        px2Cam* thiz = (px2Cam*)thiz_void;
        thiz->PublishSingleCameraImg(portIdx, siblingIdx);
    }


    bool UpdateSingleCameraImg(uint portIdx, uint siblingIdx);
    void PublishSingleCameraImg(uint portIdx, uint siblingIdx);
    cudaStream_t* GetCudaStreamForCamera(uint portIdx, uint siblingIdx);
    dwImageCUDA* GetDwSingleCameraImageCuda(uint portIdx, uint siblingIdx);

    void RenderCamImg();
    void DrawBoundingBoxes(vector<cv::Rect>  bbRectList, vector<float32_t*> bbColorList, float32_t lineWidth);
    void DrawBoundingBoxesWithLabels(vector<cv::Rect>  bbRectList, vector<float32_t*> bbColorList, vector<const char*> bbLabelList, float32_t lineWidth);
    void DrawBoundingBoxesWithLabelsPerClass(vector<vector<dwRectf> >  bbRectList, vector<const float32_t*> bbColorList, vector<vector<const char*> > bbLabelList, float32_t lineWidth);
    void DrawPoints(vector<cv::Point> ptList, float32_t ptSize, float32_t* ptColor);
    void DrawPolyLine(vector<cv::Point> ptList, float32_t lineWidth, float32_t* lineColor);
    void DrawPolyLineDw(vector<dwVector2f> ptList, float32_t lineWidth, dwVector4f lineColor);
    void DrawText(const char* text, cv::Point textPos, float32_t* textColor);

    void UpdateRendering();

    dwContextHandle_t GetDwContext();
    trtImgData GetTrtImgData();
    matImgData GetCroppedMatImgData();
    matImgData GetOriMatImgData();
    dwImageCUDA* GetDwImageCuda();

    void CoordTrans_Resize2Ori(int xIn, int yIn, int& xOut, int& yOut);
    void CoordTrans_ResizeAndCrop2Ori(float xIn, float yIn, float &xOut, float &yOut);

    

public:
    ProgramArguments mArguments;

    uint32_t mNumCameras=0;

    std::vector<Camera> mCameraInfoList;

protected:
    void InitGL();
    bool InitSDK();
    bool InitRenderer();
    bool InitSAL();
    bool InitSensors();
    bool InitPipeline();

    void ReleaseModules();


private:
    ros::NodeHandle* mNodeH;
    std::vector<std::string> mPublishImageTopicNames;
    std::string mCameraType_ab;
    std::string mCameraType_cd;
    std::string mCameraType_ef;
    std::string mSelectorMask;
    cv::Size mPublishImageSize;


    dwContextHandle_t mContext = DW_NULL_HANDLE;
    dwSALHandle_t mSAL = DW_NULL_HANDLE;
    dwRendererHandle_t mRenderer = DW_NULL_HANDLE;
    std::vector<dwSensorHandle_t> mCameraList;
    std::vector<dwImageProperties> mCamImgPropList;
    std::vector<dwCameraProperties> mCamPropList;
    dwImageStreamerHandle_t mStreamerCUDA2GL = DW_NULL_HANDLE;
    dwCameraFrameHandle_t mFrameHandle = DW_NULL_HANDLE;
    dwImageHandle_t mFrameCUDAHandle = DW_NULL_HANDLE;
    dwImageHandle_t mFrameGLHandle = DW_NULL_HANDLE;
    dwSensorSerializerHandle_t mSerializer = DW_NULL_HANDLE;

    bool mRecordCamera = false;
    bool mResizeEnable = false;
    dwTegraMode mTegraMode = MASTER_TEGRA;

    float mResizeRatio = 1.f;
    int mResizeWidth = CAM_IMG_WIDTH;
    int mResizeHeight = CAM_IMG_HEIGHT;
    int mROIx;
    int mROIy;
    int mROIw;
    int mROIh;

    WindowBase* mWindow = nullptr;
    dwRenderEngineHandle_t mRenderEngine = DW_NULL_HANDLE;

    uint32_t sibling = 0;
    dwTime_t timeout_us = 40000;

    dwImageCUDA* mCamImgCuda;
    dwImageCUDA* mCamImgCudaRaw;
    dwImageCUDA* mCamImgCudaRCB;

    uint8_t* mPitchedImgCudaRGBA;
    uint8_t* mGpuMat_data;
    cv::cuda::GpuMat mGpuMat;
    uint8_t* mGpuMatResized_data;
    cv::cuda::GpuMat mGpuMatResized;
    float* mTrtImg;
    uint8_t* mGpuMatResizedAndCropped_data;
    cv::cuda::GpuMat mGpuMatResizedAndCropped;
    cv::Mat mMatResizedAndCropped;
    cv::Mat mMatOri;

    uint64_t mCamTimestamp = 0;
    trtImgData mCurTrtImgData;
    matImgData mCurCroppedMatImgData;
    matImgData mCurOriMatImgData;

    displayParameters mDispParams;
    dwImageGL* mImgGl;

    camInputParameters mCamInputParams;

    // For Raw
    dwSoftISPHandle_t mISP = DW_NULL_HANDLE;
    uint32_t mISPoutput;
    dwImageHandle_t mRawImageHandle = DW_NULL_HANDLE;
    dwImageHandle_t mRCBImageHandle = DW_NULL_HANDLE;
    dwImageProperties mRCBImgProp{};

};



#endif // PX2CAMLIB_H

