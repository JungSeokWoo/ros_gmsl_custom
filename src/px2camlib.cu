#include "px2camlib.h"

// Convert Cam Img to gpuMat
__global__
void PitchedRGBA2GpuMat(uint8_t* pitchedImgRGBA, uint8_t* imgGpuMat, int width, int height, int cudaPitch)
{
    int xIndex_3ch = blockIdx.x*blockDim.x + threadIdx.x;
    int yIndex = blockIdx.y*blockDim.y + threadIdx.y;

    if((xIndex_3ch < 3*width) && (yIndex < height))
    {
        int xIndex = xIndex_3ch%width;
        int c = xIndex_3ch/width;

        int j = yIndex*width + xIndex;
        imgGpuMat[j*3 + 2 - c] = pitchedImgRGBA[cudaPitch*yIndex + c + xIndex*4];
    }
}

// Crop and convert gpuMat original image to Tensor RT and gpuMat
__global__
void GpuMat2Img(uint8_t* imgGpuMatOri, float* imgTrt, uint8_t* imgGpuMat,
                int width, int height,
                int roiX, int roiY, int roiW, int roiH)
{
    int xIndex_3ch = blockIdx.x * blockDim.x + threadIdx.x;
    int yIndex = blockIdx.y * blockDim.y + threadIdx.y;

    if((xIndex_3ch < 3*roiW) && (yIndex < roiH))
    {
        int xIndex = xIndex_3ch%roiW;
        int c = xIndex_3ch/roiW;

        int j = (yIndex)*roiW + (xIndex);
        int j_ori = (yIndex + roiY)*width + (xIndex + roiX);

        imgTrt[c*roiH*roiW + j] = (float)imgGpuMatOri[j_ori*3 + 2 - c]/255.f;
        imgGpuMat[j*3 + 2 - c] = imgGpuMatOri[j_ori*3 + 2 - c];
    }
}

px2Cam::px2Cam()
{
    mArguments = ProgramArguments(
    {           ProgramArguments::Option_t("type-ab", "ar0231-rccb-ae-sf3324"),
                ProgramArguments::Option_t("type-cd", "ar0231-rccb-ae-sf3324"),
                ProgramArguments::Option_t("selector-mask", "11110011"),
                ProgramArguments::Option_t("custom-board", "0"),
                ProgramArguments::Option_t("write-file", ""),
                ProgramArguments::Option_t("serializer-type", "h264"),
                ProgramArguments::Option_t("serializer-bitrate", "8000000"),
                ProgramArguments::Option_t("serializer-framerate", "30"),
                ProgramArguments::Option_t("fifo-size", "3"),
                ProgramArguments::Option_t("cross-csi-sync", "0"),
                ProgramArguments::Option_t("slave", "0")
    });
}

px2Cam::~px2Cam()
{
    ReleaseModules();
}

void px2Cam::ReleaseModules()
{
    if(mStreamerCUDA2GL)
    {
        dwImageStreamer_release(&mStreamerCUDA2GL);
    }

    // if(mCamera)
    // {
    //     dwSensor_stop(mCamera);
    //     dwSAL_releaseSensor(&mCamera);
    // }

    if(mRenderEngine)
    {
        dwRenderEngine_release(&mRenderEngine);
    }

    if(mRenderer)
    {
        dwRenderer_release(&mRenderer);
    }

    dwSAL_release(&mSAL);
    dwRelease(&mContext);
}

bool px2Cam::Init()
{
     // Initialize Modules
    bool status;
    // InitGL();

    status = InitSDK();
    if(!status)
        return status;

    // status = InitRenderer();
    // if(!status)
    //     return status;

    status = InitSAL();
    if(!status)
        return status;

    status = InitSensors();
    if(!status)
        return status;

    status = InitPipeline();
    if(!status)
        return status;

    return true;
}

void px2Cam::CoordTrans_Resize2Ori(int xIn, int yIn, int& xOut, int& yOut)
{
    xOut = (int)(xIn/mResizeRatio);
    yOut = (int)(yIn/mResizeRatio);
}

void px2Cam::CoordTrans_ResizeAndCrop2Ori(float xIn, float yIn, float &xOut, float &yOut)
{
    xOut = (float)((xIn + mROIx)/mResizeRatio);
    yOut = (float)((yIn + mROIy)/mResizeRatio);
}

void px2Cam::InitGL()
{
    if(!mWindow)
    {
        mWindow = new WindowGLFW(mDispParams.windowTitle.c_str(), mDispParams.windowWidth, mDispParams.windowHeight, !mDispParams.onDisplay);
    }
    mWindow->makeCurrent();
}

bool px2Cam::InitSDK()
{
    dwStatus status;

    dwContextParameters sdkParams = {};

    // sdkParams.eglDisplay = mWindow->getEGLDisplay();

    status = dwInitialize(&mContext, DW_VERSION, &sdkParams);

    if(status == DW_SUCCESS)
    {
        cout << "[DW_INIT_STEP_1] Driveworks init success" << endl;
    }
    else
    {
        cout << "[DW_INIT_STEP_1] Driveworks init fail" << endl;
        return false;
    }

    return true;
}

bool px2Cam::InitRenderer()
{
    dwStatus status;

    status = dwRenderer_initialize(&mRenderer, mContext);

    if(status == DW_SUCCESS)
    {
        cout << "[DW_INIT_STEP_2] Renderer init success" << endl;
    }
    else
    {
        cout << "[DW_INIT_STEP_2] Renderer init fail" << endl;
        return false;
    }

    dwRenderEngineParams renderEngineParams{};
    CHECK_DW_ERROR(dwRenderEngine_initDefaultParams(&renderEngineParams, mWindow->width(), mWindow->height()));

    renderEngineParams.defaultTile.lineWidth = 0.2f;
    renderEngineParams.defaultTile.font = DW_RENDER_ENGINE_FONT_VERDANA_20;

    CHECK_DW_ERROR(dwRenderEngine_initialize(&mRenderEngine, &renderEngineParams, mContext));

    return true;
}

bool px2Cam::InitSAL()
{
    dwStatus status;

    status = dwSAL_initialize(&mSAL, mContext);

    if(status == DW_SUCCESS)
    {
        cout << "[DW_INIT_STEP_3] SAL init success" << endl;
    }
    else
    {
        cout << "[DW_INIT_STEP_3] SAL init fail" << endl;
        return false;
    }

    return true;
}

bool px2Cam::InitSensors()
{
    dwStatus status;

    // std::string selector = mArguments.get("selector-mask");
    std::string selector = mSelectorMask;

    // identify active ports
    int idx = 0;
    int cnt[3] = {0, 0, 0};
    std::string port[3] = {"ab", "cd", "ef"};

    for(size_t i = 0; i < selector.length() && i < 12;i++, idx++)
    {
        const char s = selector[i];
        if (s == '1')
        {
            cnt[idx/4]++;
        }
    }
    for (size_t portIdx=0U; portIdx < 3; portIdx++)
    {
        if(cnt[portIdx] > 0)
        {
            std::string params;
            params += std::string("csi-port=") + port[portIdx];
            // params += ",camera-type=" + mArguments.get((std::string("type-") + port[portIdx]).c_str());
            switch(portIdx)
            {
                case 0:
                    params += ",camera-type=" + mCameraType_ab;       
                    break;
                case 1:
                    params += ",camera-type=" + mCameraType_cd;       
                    break;
                case 2:
                    params += ",camera-type=" + mCameraType_ef;       
                    break;
            }
            params += ",camera-count=" + std::to_string( cnt[portIdx] );

            if (selector.size() >= portIdx*4)
            {
                params += ",camera-mask="+ selector.substr(portIdx*4, std::min(selector.size() - portIdx*4, size_t{4}));
            }

            params += ",slave=" + mArguments.get("slave");
            params += ",cross-csi-sync=" + mArguments.get("cross-csi-sync");
            params += ",fifo-size=" + mArguments.get("fifo-size");
            params += std::string("output-format=raw");

            dwSensorHandle_t cameraHandle = DW_NULL_HANDLE;
            dwSensorParams cameraParams;
            cameraParams.parameters = params.c_str();
            cameraParams.protocol = "camera.gmsl";

            status = dwSAL_createSensor(&cameraHandle, cameraParams, mSAL);
        
            if (status == DW_SUCCESS) {
                Camera cam;
                cam.sensor = cameraHandle;

                dwImageProperties cameraImageProperties;
                dwSensorCamera_getImageProperties(&cameraImageProperties,DW_CAMERA_OUTPUT_CUDA_RAW_UINT16, cameraHandle);

                dwCameraProperties cameraProperties;
                dwSensorCamera_getSensorProperties(&cameraProperties, cameraHandle);
                
                cam.cameraProperties = cameraProperties;
                cam.width = cameraImageProperties.width;
                cam.height = cameraImageProperties.height;
                cam.numSiblings = cameraProperties.siblings;

                mCameraInfoList.push_back(cam);

                mNumCameras += cam.numSiblings;
            }
        
        }
    }
    
    return true;
}

bool px2Cam::InitPipeline()
{
    dwStatus status;

    for(uint portIdx=0U; portIdx < mCameraInfoList.size(); portIdx++)
    {
        status = dwSensor_start(mCameraInfoList[portIdx].sensor);
        // std::cout << dwGetStatusName(status) << std::endl;
    }

    std::cout << "port nums: " << mCameraInfoList.size() << std::endl;
    std::cout << "mNumCameras : " << mNumCameras << std::endl;

    std::string portName[3] = {"ab", "cd", "ef"};
    for(uint portIdx=0U; portIdx < mCameraInfoList.size(); portIdx++)
    {
        for(uint camIdx=0U; camIdx < mCameraInfoList[portIdx].numSiblings; camIdx++)
        {
            dwCameraFrameHandle_t frame = DW_NULL_HANDLE;
            status = DW_NOT_READY;
            do {
                status = dwSensorCamera_readFrame(&frame, camIdx, 300000, mCameraInfoList[portIdx].sensor);
            }while (status == DW_NOT_READY);
            std::cout << "Port name: " <<portName[portIdx] << " | Sibling Index: " << camIdx << " | ==> Camera frame test: " <<  dwGetStatusName(status) << std::endl;
        }
    }

    // something wrong happened, aborting
    if (status != DW_SUCCESS) {
        throw std::runtime_error("Cameras did not start correctly");
    }

    mISPoutput = DW_SOFTISP_PROCESS_TYPE_DEMOSAIC | DW_SOFTISP_PROCESS_TYPE_TONEMAP;

    uint topicIdx = 0;
    for(uint portIdx=0U; portIdx < mCameraInfoList.size(); portIdx++)
    {
        for(uint camIdx=0U; camIdx < mCameraInfoList[portIdx].numSiblings; camIdx++)
        {
            dwSoftISPHandle_t ISPhandle;

            dwSoftISPParams softISPParams;
            CHECK_DW_ERROR(dwSoftISP_initParamsFromCamera(&softISPParams, &mCameraInfoList[portIdx].cameraProperties));
            CHECK_DW_ERROR(dwSoftISP_initialize(&ISPhandle, &softISPParams, mContext));
            CHECK_DW_ERROR(dwSoftISP_setDemosaicMethod(DW_SOFTISP_DEMOSAIC_METHOD_INTERPOLATION, ISPhandle));

            // allocate memory for a demosaic image and bind it to the ISP
            dwImageProperties rcbImgProp;
            dwImageHandle_t rawImageHandle = DW_NULL_HANDLE;
            dwImageHandle_t rcbImageHandle = DW_NULL_HANDLE;
            dwImageCUDA* camImgCudaRCB;
            CHECK_DW_ERROR(dwSoftISP_getDemosaicImageProperties(&rcbImgProp, ISPhandle));
            CHECK_DW_ERROR(dwImage_create(&rcbImageHandle, rcbImgProp, mContext));
            CHECK_DW_ERROR(dwImage_getCUDA(&camImgCudaRCB, rcbImageHandle));
            CHECK_DW_ERROR(dwSoftISP_bindOutputDemosaic(camImgCudaRCB, ISPhandle));

            dwImageProperties glImgProps{};

            glImgProps.format = DW_IMAGE_FORMAT_RGBA_UINT8;
            glImgProps.type = DW_IMAGE_CUDA;

            glImgProps.width = rcbImgProp.width;
            glImgProps.height = rcbImgProp.height;

            dwCameraFrameHandle_t frame = DW_NULL_HANDLE;
            dwImageHandle_t frameCUDA = DW_NULL_HANDLE;
            dwImageHandle_t rawImagehandle = DW_NULL_HANDLE;
            dwImageCUDA* camImgCuda;
            dwImageCUDA* camImgCudaRaw;
            CHECK_DW_ERROR(dwImage_create(&frameCUDA, glImgProps, mContext));
            CHECK_DW_ERROR(dwImage_getCUDA(&camImgCuda, frameCUDA));
            CHECK_DW_ERROR(dwSoftISP_bindOutputTonemap(camImgCuda, ISPhandle));
        
            mCameraInfoList[portIdx].frame_PerCams.push_back(frame);
            mCameraInfoList[portIdx].frameCuda_PerCams.push_back(frameCUDA);
            mCameraInfoList[portIdx].rawImageHandle_PerCams.push_back(rawImagehandle);
            mCameraInfoList[portIdx].camImgCuda_PerCams.push_back(camImgCuda);
            mCameraInfoList[portIdx].camImgCudaRaw_PerCams.push_back(camImgCudaRaw);
            mCameraInfoList[portIdx].camImgCudaRCB_PerCams.push_back(camImgCudaRCB);
            mCameraInfoList[portIdx].ISP_PerCams.push_back(ISPhandle);

            cudaStream_t cudaStream;
            cudaStreamCreate(&cudaStream);
            mCameraInfoList[portIdx].cudaStream_PerCams.push_back(cudaStream);

            // Allocation Img Data memory
            uint8_t* pitchedImgCudaRGBA;
            cudaMalloc(&pitchedImgCudaRGBA, CUDA_PITCH*CAM_IMG_HEIGHT*sizeof(uint8_t));
            cudaMemset(pitchedImgCudaRGBA, 0, CUDA_PITCH*CAM_IMG_HEIGHT*sizeof(uint8_t));

            uint8_t* gpuMatData;
            cudaMalloc(&gpuMatData, CAM_IMG_WIDTH*CAM_IMG_HEIGHT*3*sizeof(uint8_t));
            cudaMemset(gpuMatData, 0, CAM_IMG_WIDTH*CAM_IMG_HEIGHT*3*sizeof(uint8_t));
            cv::cuda::GpuMat gpuMat = cv::cuda::GpuMat(CAM_IMG_HEIGHT, CAM_IMG_WIDTH, CV_8UC3, gpuMatData);

            mCameraInfoList[portIdx].pitchedImgCudaRGBA_PerCams.push_back(pitchedImgCudaRGBA);
            mCameraInfoList[portIdx].gpuMatData_PerCams.push_back(gpuMatData);
            mCameraInfoList[portIdx].gpuMat_PerCams.push_back(gpuMat);

            uint8_t* gpuResizedMatData;
            cudaMalloc(&gpuResizedMatData, mPublishImageSize.width*mPublishImageSize.height*3*sizeof(uint8_t));
            cudaMemset(gpuResizedMatData, 0, mPublishImageSize.width*mPublishImageSize.height*3*sizeof(uint8_t));
            cv::cuda::GpuMat gpuResizedMat = cv::cuda::GpuMat(mPublishImageSize, CV_8UC3, gpuResizedMatData);
            mCameraInfoList[portIdx].gpuResizedMatData_PerCams.push_back(gpuResizedMatData);
            mCameraInfoList[portIdx].gpuResizedMat_PerCams.push_back(gpuResizedMat);

            sensor_msgs::Image toPublishImage;
            toPublishImage.encoding = sensor_msgs::image_encodings::BGR8;
            toPublishImage.is_bigendian = 0;
            toPublishImage.step = mPublishImageSize.width*3;
            toPublishImage.width = mPublishImageSize.width;
            toPublishImage.height = mPublishImageSize.height;
            toPublishImage.data.resize(mPublishImageSize.width*mPublishImageSize.height*3);
            mCameraInfoList[portIdx].toPublishImage_PerCams.push_back(toPublishImage);

            uint8_t* cpuMatData = &mCameraInfoList[portIdx].toPublishImage_PerCams[camIdx].data[0];
            
            cv::Mat cpuMat = cv::Mat(mPublishImageSize, CV_8UC3, cpuMatData);
            mCameraInfoList[portIdx].cpuMatData_PerCams.push_back(cpuMatData);
            mCameraInfoList[portIdx].cpuMat_PerCams.push_back(cpuMat);

            ros::Publisher imagePublisher = mNodeH->advertise<sensor_msgs::Image>(mPublishImageTopicNames[topicIdx], 1); ;
            mCameraInfoList[portIdx].imagePublisher_PerCams.push_back(imagePublisher);
            topicIdx++;
        }
    }

    return true;
}

bool px2Cam::UpdateSingleCameraImg(uint portIdx, uint siblingIdx)
{
    dwStatus status;
    uint camIdx = siblingIdx;

    cudaStreamSynchronize(mCameraInfoList[portIdx].cudaStream_PerCams[camIdx]);

    status = dwSensorCamera_readFrame(&mCameraInfoList[portIdx].frame_PerCams[camIdx], camIdx, 300000, mCameraInfoList[portIdx].sensor);

    if (status == DW_END_OF_STREAM)
    {
        cout << "Camera reached end of stream." << endl;
        return false;
    }
    else if((status == DW_NOT_READY) || (status == DW_TIME_OUT)){
        while((status == DW_NOT_READY) || (status == DW_TIME_OUT))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            status = dwSensorCamera_readFrame(&mCameraInfoList[portIdx].frame_PerCams[camIdx], camIdx, 300000, mCameraInfoList[portIdx].sensor);
            printf("."); fflush(stdout);
        }
    }
    else if(status == DW_SUCCESS)
    {
    //    cout << "[DW_PROC_STEP_1] Read frame success" << portIdx << "," << camIdx << endl;
    }
    else
    {
        cout << "[DW_PROC_STEP_1] Read frame fail : " <<  dwGetStatusName(status) << endl;
    }

    status = dwSensorCamera_getImage(&(mCameraInfoList[portIdx].rawImageHandle_PerCams[camIdx]), DW_CAMERA_OUTPUT_CUDA_RAW_UINT16, mCameraInfoList[portIdx].frame_PerCams[camIdx]);

    if(status == DW_SUCCESS)
    {
    //    cout << "[DW_PROC_STEP_2] Get Raw Image Handle success" << endl;
    }
    else
    {
        cout << "[DW_PROC_STEP_2] Get Raw Image Handle fail : " << dwGetStatusName(status) << endl;
    }

    status = dwImage_getCUDA(&mCameraInfoList[portIdx].camImgCudaRaw_PerCams[camIdx], mCameraInfoList[portIdx].rawImageHandle_PerCams[camIdx]);

    if(status == DW_SUCCESS)
    {
        // cout << "[DW_PROC_STEP_3] Get Raw CUDA Image success" << endl;
    }
    else
    {
        cout << "[DW_PROC_STEP_3] Get Raw CUDA Image failed : " << dwGetStatusName(status) << endl;
    }


    CHECK_DW_ERROR(dwSoftISP_bindInputRaw(mCameraInfoList[portIdx].camImgCudaRaw_PerCams[camIdx], mCameraInfoList[portIdx].ISP_PerCams[camIdx]));
    CHECK_DW_ERROR(dwSoftISP_setProcessType(mISPoutput, mCameraInfoList[portIdx].ISP_PerCams[camIdx]));
    CHECK_DW_ERROR(dwSoftISP_processDeviceAsync(mCameraInfoList[portIdx].ISP_PerCams[camIdx]));

    dwSensorCamera_returnFrame(&mCameraInfoList[portIdx].frame_PerCams[camIdx]);

    // Set timestamp
    uint64_t camTimestamp_us =  mCameraInfoList[portIdx].camImgCuda_PerCams[camIdx]->timestamp_us;
    mCameraInfoList[portIdx].toPublishImage_PerCams[camIdx].header.stamp.sec = camTimestamp_us/1000000;
    mCameraInfoList[portIdx].toPublishImage_PerCams[camIdx].header.stamp.nsec = (camTimestamp_us%1000000)*1000;

    const dim3 block(32,32);
    const dim3 grid((CAM_IMG_WIDTH*3 + block.x - 1)/block.x, (CAM_IMG_HEIGHT + block.y -1)/block.y);

    PitchedRGBA2GpuMat <<< grid, block, 0, mCameraInfoList[portIdx].cudaStream_PerCams[camIdx] >>> 
    ((uint8_t*)mCameraInfoList[portIdx].camImgCuda_PerCams[camIdx]->dptr[0], mCameraInfoList[portIdx].gpuMatData_PerCams[camIdx], CAM_IMG_WIDTH, CAM_IMG_HEIGHT, CUDA_PITCH);
    
    cv::cuda::StreamAccessor streamAccessor_for_cv;
    cv::cuda::Stream stream_for_cv = streamAccessor_for_cv.wrapStream(mCameraInfoList[portIdx].cudaStream_PerCams[camIdx]);
    cv::cuda::resize(mCameraInfoList[portIdx].gpuMat_PerCams[camIdx], mCameraInfoList[portIdx].gpuResizedMat_PerCams[camIdx], mPublishImageSize, 0, 0, cv::INTER_LINEAR, 
                        stream_for_cv);

    cudaMemcpyAsync(mCameraInfoList[portIdx].cpuMatData_PerCams[camIdx], mCameraInfoList[portIdx].gpuResizedMatData_PerCams[camIdx], mPublishImageSize.width*mPublishImageSize.height*3*sizeof(uint8_t), cudaMemcpyDeviceToHost, mCameraInfoList[portIdx].cudaStream_PerCams[camIdx]);

    cudaMemcpyAsync(mCameraInfoList[portIdx].cpuMatData_PerCams[camIdx], mCameraInfoList[portIdx].gpuMatData_PerCams[camIdx], mPublishImageSize.width*mPublishImageSize.height*3*sizeof(uint8_t), cudaMemcpyDeviceToHost, mCameraInfoList[portIdx].cudaStream_PerCams[camIdx]);


    return true;
}

bool px2Cam::UpdateAllCamImgs()
{
    dwStatus status;
    
    for(uint portIdx=0U; portIdx < mCameraInfoList.size(); portIdx++)
    {
        for(uint camIdx=0U; camIdx < mCameraInfoList[portIdx].numSiblings; camIdx++)
        {
            cudaStreamSynchronize(mCameraInfoList[portIdx].cudaStream_PerCams[camIdx]);

            status = dwSensorCamera_readFrame(&mCameraInfoList[portIdx].frame_PerCams[camIdx], camIdx, 300000, mCameraInfoList[portIdx].sensor);

            if (status == DW_END_OF_STREAM)
            {
                cout << "Camera reached end of stream." << endl;
                return false;
            }
            else if((status == DW_NOT_READY) || (status == DW_TIME_OUT)){
                while((status == DW_NOT_READY) || (status == DW_TIME_OUT))
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    status = dwSensorCamera_readFrame(&mCameraInfoList[portIdx].frame_PerCams[camIdx], camIdx, 300000, mCameraInfoList[portIdx].sensor);
                    printf("."); fflush(stdout);
                }
            }
            else if(status == DW_SUCCESS)
            {
            //    cout << "[DW_PROC_STEP_1] Read frame success" << portIdx << "," << camIdx << endl;
            }
            else
            {
                cout << "[DW_PROC_STEP_1] Read frame fail : " <<  dwGetStatusName(status) << endl;
            }

            dwImageHandle_t rawImageHandle = DW_NULL_HANDLE;
            status = dwSensorCamera_getImage(&rawImageHandle, DW_CAMERA_OUTPUT_CUDA_RAW_UINT16, mCameraInfoList[portIdx].frame_PerCams[camIdx]);

            if(status == DW_SUCCESS)
            {
            //    cout << "[DW_PROC_STEP_2] Get Raw Image Handle success" << endl;
            }
            else
            {
                cout << "[DW_PROC_STEP_2] Get Raw Image Handle fail : " << dwGetStatusName(status) << endl;
            }

            status = dwImage_getCUDA(&mCameraInfoList[portIdx].camImgCudaRaw_PerCams[camIdx], rawImageHandle);

            if(status == DW_SUCCESS)
            {
                // cout << "[DW_PROC_STEP_3] Get Raw CUDA Image success" << endl;
            }
            else
            {
                cout << "[DW_PROC_STEP_3] Get Raw CUDA Image failed : " << dwGetStatusName(status) << endl;
            }
        

            CHECK_DW_ERROR(dwSoftISP_bindInputRaw(mCameraInfoList[portIdx].camImgCudaRaw_PerCams[camIdx], mCameraInfoList[portIdx].ISP_PerCams[camIdx]));
            CHECK_DW_ERROR(dwSoftISP_setProcessType(mISPoutput, mCameraInfoList[portIdx].ISP_PerCams[camIdx]));
            CHECK_DW_ERROR(dwSoftISP_processDeviceAsync(mCameraInfoList[portIdx].ISP_PerCams[camIdx]));

            dwSensorCamera_returnFrame(&mCameraInfoList[portIdx].frame_PerCams[camIdx]);

            // Set timestamp
            uint64_t camTimestamp_us =  mCameraInfoList[portIdx].camImgCuda_PerCams[camIdx]->timestamp_us;
            mCameraInfoList[portIdx].toPublishImage_PerCams[camIdx].header.stamp.sec = camTimestamp_us/1000000;
            mCameraInfoList[portIdx].toPublishImage_PerCams[camIdx].header.stamp.nsec = (camTimestamp_us%1000000)*1000;

            const dim3 block(32,32);
            const dim3 grid((CAM_IMG_WIDTH*3 + block.x - 1)/block.x, (CAM_IMG_HEIGHT + block.y -1)/block.y);

            PitchedRGBA2GpuMat <<< grid, block, 0, mCameraInfoList[portIdx].cudaStream_PerCams[camIdx] >>> 
            ((uint8_t*)mCameraInfoList[portIdx].camImgCuda_PerCams[camIdx]->dptr[0], mCameraInfoList[portIdx].gpuMatData_PerCams[camIdx], CAM_IMG_WIDTH, CAM_IMG_HEIGHT, CUDA_PITCH);
            
            cv::cuda::StreamAccessor streamAccessor_for_cv;
            cv::cuda::Stream stream_for_cv = streamAccessor_for_cv.wrapStream(mCameraInfoList[portIdx].cudaStream_PerCams[camIdx]);
            cv::cuda::resize(mCameraInfoList[portIdx].gpuMat_PerCams[camIdx], mCameraInfoList[portIdx].gpuResizedMat_PerCams[camIdx], mPublishImageSize, 0, 0, cv::INTER_LINEAR, 
                             stream_for_cv);

            cudaMemcpyAsync(mCameraInfoList[portIdx].cpuMatData_PerCams[camIdx], mCameraInfoList[portIdx].gpuResizedMatData_PerCams[camIdx], mPublishImageSize.width*mPublishImageSize.height*3*sizeof(uint8_t), cudaMemcpyDeviceToHost, mCameraInfoList[portIdx].cudaStream_PerCams[camIdx]);
        }
    }


    return true;
}

void px2Cam::PublishSingleCameraImg(uint portIdx, uint siblingIdx)
{
    uint camIdx = siblingIdx;
    
    cudaStreamSynchronize(mCameraInfoList[portIdx].cudaStream_PerCams[camIdx]);
    mCameraInfoList[portIdx].imagePublisher_PerCams[camIdx].publish(mCameraInfoList[portIdx].toPublishImage_PerCams[camIdx]);
}

void px2Cam::PublishAllImages()
{
    for(uint portIdx=0U; portIdx < mCameraInfoList.size(); portIdx++)
    {
        for(uint camIdx=0U; camIdx < mCameraInfoList[portIdx].numSiblings; camIdx++)
        {
            cudaStreamSynchronize(mCameraInfoList[portIdx].cudaStream_PerCams[camIdx]);
            mCameraInfoList[portIdx].imagePublisher_PerCams[camIdx].publish(mCameraInfoList[portIdx].toPublishImage_PerCams[camIdx]);
        }
    }
}

cudaStream_t* px2Cam::GetCudaStreamForCamera(uint portIdx, uint siblingIdx)
{
    return  &(mCameraInfoList[portIdx].cudaStream_PerCams[siblingIdx]);
}

dwImageCUDA* px2Cam::GetDwSingleCameraImageCuda(uint portIdx, uint siblingIdx)
{
    return  mCameraInfoList[portIdx].camImgCuda_PerCams[siblingIdx];
}

void px2Cam::RenderCamImg()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    dwTime_t timeout = 132000;

    // stream that image to the GL domain
    CHECK_DW_ERROR(dwImageStreamer_producerSend(mFrameCUDAHandle, mStreamerCUDA2GL));

    CHECK_DW_ERROR(dwImageStreamer_consumerReceive(&mFrameGLHandle, timeout, mStreamerCUDA2GL));

    CHECK_DW_ERROR(dwImage_getGL(&mImgGl, mFrameGLHandle));

    // render received texture
    dwVector2f range{};
    range.x = mImgGl->prop.width;
    range.y = mImgGl->prop.height;
    CHECK_DW_ERROR(dwRenderEngine_setCoordinateRange2D(range, mRenderEngine));
    CHECK_DW_ERROR(dwRenderEngine_renderImage2D(mImgGl, {0.0f, 0.0f, range.x, range.y}, mRenderEngine));

    // returned the consumed image
    CHECK_DW_ERROR(dwImageStreamer_consumerReturn(&mFrameGLHandle, mStreamerCUDA2GL));

    // notify the producer that the work is done
    CHECK_DW_ERROR(dwImageStreamer_producerReturn(nullptr, timeout, mStreamerCUDA2GL));
}

void px2Cam::DrawBoundingBoxes(vector<cv::Rect>  bbRectList, vector<float32_t*> bbColorList, float32_t lineWidth)
{
    CHECK_DW_ERROR(dwRenderEngine_setLineWidth(lineWidth, mRenderEngine));
    for(uint bbInd = 0; bbInd < bbRectList.size(); bbInd++)
    {
        float32_t* bBoxColor = bbColorList[bbInd];
        dwRenderEngineColorRGBA bBoxColorDw;
        bBoxColorDw.x = bBoxColor[0];
        bBoxColorDw.y = bBoxColor[1];
        bBoxColorDw.z = bBoxColor[2];
        bBoxColorDw.w = bBoxColor[3];
        CHECK_DW_ERROR(dwRenderEngine_setColor(bBoxColorDw, mRenderEngine));

        cv::Rect bBoxRect = bbRectList[bbInd];
        dwRectf bBoxRectDw;
        bBoxRectDw.x = bBoxRect.x;
        bBoxRectDw.y = bBoxRect.y;
        bBoxRectDw.width = bBoxRect.width;
        bBoxRectDw.height = bBoxRect.height;

        dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_BOXES_2D, &bBoxRectDw, sizeof(dwRectf), 0, 1, mRenderEngine);
    }
}

void px2Cam::DrawBoundingBoxesWithLabels(vector<cv::Rect>  bbRectList, vector<float32_t*> bbColorList, vector<const char*> bbLabelList, float32_t lineWidth)
{
    CHECK_DW_ERROR(dwRenderEngine_setLineWidth(lineWidth, mRenderEngine));
    for(uint bbInd = 0; bbInd < bbRectList.size(); bbInd++)
    {
        float32_t* bBoxColor = bbColorList[bbInd];
        dwRenderEngineColorRGBA bBoxColorDw;
        bBoxColorDw.x = bBoxColor[0];
        bBoxColorDw.y = bBoxColor[1];
        bBoxColorDw.z = bBoxColor[2];
        bBoxColorDw.w = bBoxColor[3];
        CHECK_DW_ERROR(dwRenderEngine_setColor(bBoxColorDw, mRenderEngine));

        cv::Rect bBoxRect = bbRectList[bbInd];
        dwRectf bBoxRectDw;
        bBoxRectDw.x = bBoxRect.x;
        bBoxRectDw.y = bBoxRect.y;
        bBoxRectDw.width = bBoxRect.width;
        bBoxRectDw.height = bBoxRect.height;

        const char* bbLabel = bbLabelList[bbInd];

        dwRenderEngine_renderWithLabel(DW_RENDER_ENGINE_PRIMITIVE_TYPE_BOXES_2D, &bBoxRectDw, sizeof(dwRectf), 0, bbLabel, 1, mRenderEngine);
    }
}

void px2Cam::DrawBoundingBoxesWithLabelsPerClass(vector<vector<dwRectf> >  bbRectList, vector<const float32_t*> bbColorList, vector<vector<const char*> > bbLabelList, float32_t lineWidth)
{
    CHECK_DW_ERROR(dwRenderEngine_setLineWidth(lineWidth, mRenderEngine));
    for(uint classIdx = 0; classIdx < bbRectList.size(); classIdx++)
    {
        const float32_t* bBoxColor = bbColorList[classIdx];
        dwRenderEngineColorRGBA bBoxColorDw;
        bBoxColorDw.x = bBoxColor[0];
        bBoxColorDw.y = bBoxColor[1];
        bBoxColorDw.z = bBoxColor[2];
        bBoxColorDw.w = bBoxColor[3];
        CHECK_DW_ERROR(dwRenderEngine_setColor(bBoxColorDw, mRenderEngine));

        if (bbRectList[classIdx].size() == 0)
            continue;

        CHECK_DW_ERROR(dwRenderEngine_renderWithLabels(DW_RENDER_ENGINE_PRIMITIVE_TYPE_BOXES_2D, &bbRectList[classIdx][0], sizeof(dwRectf), 0, &bbLabelList[classIdx][0], bbRectList[classIdx].size(), mRenderEngine));
    }
}

void px2Cam::DrawPoints(vector<cv::Point> ptList, float32_t ptSize, float32_t* ptColor)
{
    CHECK_DW_ERROR(dwRenderEngine_setPointSize(ptSize, mRenderEngine));
    vector<dwVector2f> ptDwList;
    for(uint ptInd = 0; ptInd < ptList.size(); ptInd++)
    {
        cv::Point pt = ptList[ptInd];
        dwVector2f ptDw;
        ptDw.x = pt.x;
        ptDw.y = pt.y;
        ptDwList.push_back(ptDw);
    }

    dwRenderEngineColorRGBA ptColorDw;
    ptColorDw.x = ptColor[0];
    ptColorDw.y = ptColor[1];
    ptColorDw.z = ptColor[2];
    ptColorDw.w = ptColor[3];

    CHECK_DW_ERROR(dwRenderEngine_setColor(ptColorDw, mRenderEngine));

    dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_POINTS_2D, &ptDwList[0], sizeof(dwVector2f), 0, ptDwList.size(), mRenderEngine);
}

void px2Cam::DrawPolyLine(vector<cv::Point> ptList, float32_t lineWidth, float32_t* lineColor)
{
    CHECK_DW_ERROR(dwRenderEngine_setLineWidth(lineWidth, mRenderEngine));
    vector<dwVector2f> ptDwList;
    for(uint ptInd = 0; ptInd < ptList.size(); ptInd++)
    {
        cv::Point pt = ptList[ptInd];
        dwVector2f ptDw;
        ptDw.x = pt.x;
        ptDw.y = pt.y;
        ptDwList.push_back(ptDw);
    }

    dwRenderEngineColorRGBA lineColorDw;
    lineColorDw.x = lineColor[0];
    lineColorDw.y = lineColor[1];
    lineColorDw.z = lineColor[2];
    lineColorDw.w = lineColor[3];

    CHECK_DW_ERROR(dwRenderEngine_setColor(lineColorDw, mRenderEngine));

    dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINESTRIP_2D, &ptDwList[0], sizeof(dwVector2f), 0, ptDwList.size(), mRenderEngine);
}

void px2Cam::DrawPolyLineDw(vector<dwVector2f> ptList, float32_t lineWidth, dwVector4f lineColor)
{
    CHECK_DW_ERROR(dwRenderEngine_setLineWidth(lineWidth, mRenderEngine));

    CHECK_DW_ERROR(dwRenderEngine_setColor(lineColor, mRenderEngine));

    dwRenderEngine_render(DW_RENDER_ENGINE_PRIMITIVE_TYPE_LINESTRIP_2D, &ptList[0], sizeof(dwVector2f), 0, ptList.size(), mRenderEngine);
}

void px2Cam::DrawText(const char* text, cv::Point textPos, float32_t* textColor)
{
    dwVector2f textPosDw;
    textPosDw.x = textPos.x;
    textPosDw.y = textPos.y;


    dwRenderEngineColorRGBA textColorDw;
    textColorDw.x = textColor[0];
    textColorDw.y = textColor[1];
    textColorDw.z = textColor[2];
    textColorDw.w = textColor[3];

    CHECK_DW_ERROR(dwRenderEngine_setColor(textColorDw, mRenderEngine));

    dwRenderEngine_renderText2D(text, textPosDw, mRenderEngine);
}

void px2Cam::UpdateRendering()
{
    mWindow->swapBuffers();
}

dwContextHandle_t px2Cam::GetDwContext()
{
    return mContext;
}

trtImgData px2Cam::GetTrtImgData()
{
    mCurTrtImgData.timestamp_us = mCamTimestamp;
    mCurTrtImgData.trtImg = mTrtImg;
    return mCurTrtImgData;
}

matImgData px2Cam::GetCroppedMatImgData()
{
    mGpuMatResizedAndCropped.download(mMatResizedAndCropped);
    mCurCroppedMatImgData.timestamp_us = mCamTimestamp;
    mCurCroppedMatImgData.matImg = mMatResizedAndCropped;
    return mCurCroppedMatImgData;
}

matImgData px2Cam::GetOriMatImgData()
{
    mGpuMat.download(mMatOri);
    mCurOriMatImgData.timestamp_us = mCamTimestamp;
    mCurOriMatImgData.matImg = mMatOri;
    return mCurOriMatImgData;
}

dwImageCUDA* px2Cam::GetDwImageCuda()
{
    return mCamImgCuda;
}
