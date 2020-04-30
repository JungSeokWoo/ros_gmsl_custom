#include <iostream>
#include <thread>

#include "px2camlib.h"
#include "px2od.h"

class threadWorker{
public:
    threadWorker() {}
    ~threadWorker() {}

    void Init(px2Cam* px2CamObj, uint portIdx, uint siblingIdx)
    {
        mPx2CamObj = px2CamObj;
        mPortIdx = portIdx;
        mSiblingIdx = siblingIdx;

        mPx2ODObj = new px2OD(px2CamObj->GetDwContext(), px2CamObj->GetCudaStreamForCamera(mPortIdx, mSiblingIdx));
        mPx2ODObj->Init();
    }

    void OneShotIter()
    {
        std::cout << mPortIdx << mSiblingIdx << std::endl;
        mPx2CamObj->UpdateSingleCameraImg(mPortIdx, mSiblingIdx);
        mPx2CamObj->PublishSingleCameraImg(mPortIdx, mSiblingIdx);

        dwImageCUDA* dnnInputImg = mPx2CamObj->GetDwSingleCameraImageCuda(mPortIdx, mSiblingIdx);

        vector<vector<dwRectf> > outputODRectPerClass;
        vector<const float32_t*> outputODRectColorPerClass;
        vector<vector<const char*> > outputODLabelPerClass;
        vector<vector<float32_t> > outputODConfidencePerClass;
        vector<vector<int> > outputODIDPerClass;

        mPx2ODObj->DetectObjects(dnnInputImg,
                                outputODRectPerClass,
                                outputODRectColorPerClass,
                                outputODLabelPerClass,
                                outputODConfidencePerClass,
                                outputODIDPerClass);
    }

    void ThreadLoop()
    {
        do{
            std::cout << mPortIdx << mSiblingIdx << std::endl;

            mPx2CamObj->UpdateSingleCameraImg(mPortIdx, mSiblingIdx);
            mPx2CamObj->PublishSingleCameraImg(mPortIdx, mSiblingIdx);

            dwImageCUDA* dnnInputImg = mPx2CamObj->GetDwSingleCameraImageCuda(mPortIdx, mSiblingIdx);

            vector<vector<dwRectf> > outputODRectPerClass;
            vector<const float32_t*> outputODRectColorPerClass;
            vector<vector<const char*> > outputODLabelPerClass;
            vector<vector<float32_t> > outputODConfidencePerClass;
            vector<vector<int> > outputODIDPerClass;

            mPx2ODObj->DetectObjects(dnnInputImg,
                                    outputODRectPerClass,
                                    outputODRectColorPerClass,
                                    outputODLabelPerClass,
                                    outputODConfidencePerClass,
                                    outputODIDPerClass);
        }while(true);
    }

    void OnlyDetect()
    {
        dwImageCUDA* dnnInputImg = mPx2CamObj->GetDwSingleCameraImageCuda(mPortIdx, mSiblingIdx);

        vector<vector<dwRectf> > outputODRectPerClass;
        vector<const float32_t*> outputODRectColorPerClass;
        vector<vector<const char*> > outputODLabelPerClass;
        vector<vector<float32_t> > outputODConfidencePerClass;
        vector<vector<int> > outputODIDPerClass;

        mPx2ODObj->DetectObjects(dnnInputImg,
                                outputODRectPerClass,
                                outputODRectColorPerClass,
                                outputODLabelPerClass,
                                outputODConfidencePerClass,
                                outputODIDPerClass);
    }


private:
    px2Cam* mPx2CamObj;
    px2OD* mPx2ODObj;

    uint mPortIdx;
    uint mSiblingIdx;

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "n_cameras_driveNet");
    ros::NodeHandle nh;

    px2Cam px2CamObj;

    std::vector<std::string> imgTopics;
    imgTopics.push_back("cam1");
    imgTopics.push_back("cam2");
    imgTopics.push_back("cam3");
    imgTopics.push_back("cam4");
    imgTopics.push_back("cam5");
    imgTopics.push_back("cam6");
    float publishImageResizeRatio = 0.5;

    std::string cameraType_ab = "ar0231-rccb-ae-sf3324";
    std::string cameraType_cd = "ar0231-rccb-ae-sf3324";
    std::string cameraType_ef = "";
    std::string selectorMask = "11110011";

    px2CamObj.SetRosParams(&nh,
                           imgTopics,
                           publishImageResizeRatio,
                           cameraType_ab,
                           cameraType_cd,
                           cameraType_ef,
                           selectorMask);

    
    // Camera library initialize
    if(!px2CamObj.Init())
        return -1;

    std::vector<threadWorker> workerList;

    for(uint portIdx=0U; portIdx < px2CamObj.mCameraInfoList.size(); portIdx++)
    {
        for(uint siblingIdx=0U; siblingIdx < px2CamObj.mCameraInfoList[portIdx].numSiblings; siblingIdx++)
        {
            threadWorker worker;
            worker.Init(&px2CamObj, portIdx, siblingIdx);
            workerList.push_back(worker);
        }
    }

    while(true)
    {
        std::vector<std::thread> threads;
        threads.resize(imgTopics.size() + workerList.size());

        px2CamObj.UpdateAllCamImgs();

        int tid=0;
        for(uint portIdx=0U; portIdx < px2CamObj.mCameraInfoList.size(); portIdx++)
        {
            for(uint siblingIdx=0U; siblingIdx < px2CamObj.mCameraInfoList[portIdx].numSiblings; siblingIdx++)
            {
                threads[tid] =  std::thread(&px2Cam::PublishSingleCameraImg, &px2CamObj, portIdx, siblingIdx);
                tid++;
            }
        }

        for(uint workerIdx=0U; workerIdx < workerList.size(); workerIdx++)
        {   
            threads[tid] =  std::thread(&threadWorker::OnlyDetect, &workerList[workerIdx]);
            tid++;
        }

        for(uint i=0U; i < threads.size(); i++)
        {
            threads[i].join();
        }
    }








    // std::vector<threadWorker> workerList;

    // for(uint portIdx=0U; portIdx < px2CamObj.mCameraInfoList.size(); portIdx++)
    // {
    //     for(uint siblingIdx=0U; siblingIdx < px2CamObj.mCameraInfoList[portIdx].numSiblings; siblingIdx++)
    //     {
    //         threadWorker worker;
    //         worker.Init(&px2CamObj, portIdx, siblingIdx);
    //         workerList.push_back(worker);
    //     }
    // }

    // std::vector<std::thread> threadList;
    // threadList.resize(workerList.size());
    
    // while(true)
    // {
    //     for(uint workerIdx=0U; workerIdx < 2; workerIdx++)
    //     // for(uint workerIdx=0U; workerIdx < workerList.size(); workerIdx++)
    //     {   
    //         threadList[workerIdx] = std::thread(&threadWorker::OneShotIter, &workerList[workerIdx]);
    //         // threadList[workerIdx].detach();
    //     }

    //      for(uint workerIdx=0U; workerIdx < workerList.size(); workerIdx++)
    //     {   
    //         threadList[workerIdx].join();
    //     }
    // }

    
    return 0;
}