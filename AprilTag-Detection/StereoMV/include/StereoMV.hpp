//
// Created by xinyang on 2020/10/10.
//

#ifndef _STEREOMV_HPP_
#define _STEREOMV_HPP_

#include "MindVision.hpp"

template<size_t CAMERA_SIZE = 2>
class StereoMV {
    static_assert(1 <= CAMERA_SIZE && CAMERA_SIZE <= 4, "only support 1~4 cameras.");
public:
    explicit StereoMV(const std::array<std::string_view, CAMERA_SIZE> &cfgs = {}) {
        load_config(cfgs);
    }

    StereoMV(const StereoMV &) = delete;

    StereoMV &operator=(const StereoMV &) = delete;

    void load_config(const std::array<std::string_view, CAMERA_SIZE> &cfgs) {
        for (int i = 0; i < CAMERA_SIZE; i++) {
            cameras[i].load_config(cfgs[i].begin());
        }
    }

    bool open() {
        bool success = true;
        for (int i = 0; i < CAMERA_SIZE; i++) {
            success &= cameras[i].open();
            if (!success) break;
            // 默认软触发模式
            success &= (CameraSetTriggerMode(cameras[i].getCameraHandle(), 1) == CAMERA_STATUS_SUCCESS);
            if (!success) break;
        }

        if (!success) close();
        return success;
    }

    bool isOpened() const {
        bool opened = true;
        for (int i = 0; i < CAMERA_SIZE; i++) {
            opened &= cameras[i].isOpened();
        }
        return opened;
    }

    void close() {
        for (int i = 0; i < CAMERA_SIZE; i++) {
            cameras[i].close();
        }
    }

    void trigger() {
        for (int i = 0; i < CAMERA_SIZE; i++) {
            cameras[i].trigger();
        }
    }

    void read(const std::array<cv::Mat *, CAMERA_SIZE> &images) {
        for (int i = 0; i < CAMERA_SIZE; i++) {
            cameras[i].read(*images[i]);
        }
    }

    void getCameraHandle(size_t id) const {
        return cameras[id].getCameraHandle();
    }

private:
    MindVision cameras[CAMERA_SIZE];
};

#endif /* _STEREOMV_HPP_ */
