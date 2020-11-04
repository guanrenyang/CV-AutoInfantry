//
// Created by xinyang on 2020/10/10.
//

#define MINDVISION_LOG_LEVEL    MINDVISION_LOG_LEVEL_WARNING

#include "MindVision.hpp"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <chrono>

/*** 图像采集回调函数 ***/
void local_callback(CameraHandle hCamera, BYTE *pFrameBuffer, tSdkFrameHead *pFrameHead, PVOID pContext) {
    auto *ptr = static_cast<MindVision *>(pContext);
    auto p = std::chrono::high_resolution_clock::now().time_since_epoch();
    MINDVISION_INFO("camera {} callback begin at {}", ptr->camera_name,
                    std::chrono::duration_cast<std::chrono::milliseconds>(p).count());
    MINDVISION_CHECK_API_THROW(CameraImageProcess(hCamera, pFrameBuffer, ptr->camera_buffer, pFrameHead));
    cv::Mat tmp;
    if (pFrameHead->uiMediaType == CAMERA_MEDIA_TYPE_MONO) {
        tmp = cv::Mat(pFrameHead->iHeight, pFrameHead->iWidth, CV_8UC1, ptr->camera_buffer);
    } else {
        tmp = cv::Mat(pFrameHead->iHeight, pFrameHead->iWidth, CV_8UC3, ptr->camera_buffer);
        cv::cvtColor(tmp, tmp, cv::COLOR_RGB2BGR);
    }
    MINDVISION_CHECK_API_THROW(CameraReleaseImageBuffer(hCamera, pFrameBuffer));
    {
        std::unique_lock lock(ptr->mtx);
        ptr->cache = std::move(tmp).clone();
        ptr->available = true;
        ptr->cv.notify_one();
    }
    p = std::chrono::high_resolution_clock::now().time_since_epoch();
    MINDVISION_INFO("camera {} callback finish at {}", ptr->camera_name,
                    std::chrono::duration_cast<std::chrono::milliseconds>(p).count());
}

MindVision::MindVision(std::string_view cfg) {
    CameraSdkInit(1);
    load_config(cfg);
}

MindVision::~MindVision() {
    close();
}

void MindVision::load_config(std::string_view cfg) {
    if (cfg.empty()) return;
    cv::FileStorage fs(cfg.begin(), cv::FileStorage::READ);

    MINDVISION_ASSERT(fs.isOpened());
    MINDVISION_ASSERT(!fs["camera_name"].empty());
    MINDVISION_ASSERT(!fs["camera_config"].empty());

    fs["camera_name"] >> camera_name;
    fs["camera_config"] >> camera_config;
}

bool MindVision::open() {
    if (isOpened()) return true;
    else close();

    bool success = false;

    // 退出函数时，如果没有成功打开两个相机，则自动关闭未关闭的相机
    AutoExecute auto_closer([&]() { if (!success) close(); });

    // 枚举所有相机
    tSdkCameraDevInfo camera_info_list[5];
    int camera_number = 5;
    MINDVISION_CHECK_API_THROW(CameraEnumerateDevice(camera_info_list, &camera_number));
    if (camera_number <= 0) {
        MINDVISION_WARNING("do not find any camera");
        return false;
    }
    MINDVISION_INFO("found {} cameras", camera_number);

    // 根据相机名称查找对应相机
    for (int i = 0; i < camera_number; i++) {
        if (h_camera <= 0) {
            MINDVISION_CHECK_API_WARNING(CameraInit(camera_info_list + i, -1, -1, &h_camera));
        }
    }
    if (h_camera <= 0) {
        MINDVISION_WARNING("cannot open camera '{}'", camera_name);
        return false;
    }

    // 获取相机特性描述结构体
    tSdkCameraCapbility cap;
    MINDVISION_CHECK_API_THROW(CameraGetCapability(h_camera, &cap));
    if (cap.sIspCapacity.bMonoSensor) {
        MINDVISION_INFO("camera '{}' is mono.", camera_name);
    } else {
        MINDVISION_INFO("camera '{}' is color.", camera_name);
    }

    // 申请buffer空间
    size_t buffer_size = cap.sResolutionRange.iHeightMax * cap.sResolutionRange.iWidthMax
                         * (cap.sIspCapacity.bMonoSensor ? 1 : 3);
    camera_buffer = new uint8_t[buffer_size];

    // 读取相机参数
    MINDVISION_CHECK_API_THROW(CameraReadParameterFromFile(h_camera, (char *) camera_config.c_str()));

    // 设置回调函数
    CameraSetCallbackFunction(h_camera, local_callback, this, nullptr);

    // 开始
    CameraPlay(h_camera);

    return success = true;
}

bool MindVision::isOpened() const {
    return h_camera > 0;
}

void MindVision::close() {
    if (h_camera > 0) {
        MINDVISION_CHECK_API_WARNING(CameraUnInit(h_camera));
        h_camera = 0;
    }
    if (camera_buffer != nullptr) {
        delete[]camera_buffer;
        camera_buffer = nullptr;
    }
}

void MindVision::trigger() const {
    MINDVISION_ASSERT(isOpened());
    auto p = std::chrono::high_resolution_clock::now().time_since_epoch();
    CameraSoftTrigger(h_camera);
    MINDVISION_INFO("camera {} tigger finish at {}", camera_name,
                    std::chrono::duration_cast<std::chrono::milliseconds>(p).count());
}

void MindVision::read(cv::Mat &src, size_t timeout_ms) const {
    MINDVISION_ASSERT(isOpened());
    std::unique_lock lock(mtx);
    if (!cv.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this]() { return available; })) {
        MINDVISION_THROW("camera {} read timeout", camera_name);
    }
    src = std::move(cache);
    available = false;
    auto p = std::chrono::high_resolution_clock::now().time_since_epoch();
    MINDVISION_INFO("camera {} read finish at {}", camera_name,
                    std::chrono::duration_cast<std::chrono::milliseconds>(p).count());
}

CameraHandle MindVision::getCameraHandle() const {
    return h_camera;
}
