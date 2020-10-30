//
// Created by xinyang on 2020/10/10.
//

#ifndef _MINDVISION_HPP_
#define _MINDVISION_HPP_

#include "SDK/CameraApi.h"
#include <opencv2/core.hpp>
#include <condition_variable>
#include <mutex>
#include <fmt/format.h>

#define MINDVISION_LOG_LEVEL_NONE       0
#define MINDVISION_LOG_LEVEL_ERROR      1
#define MINDVISION_LOG_LEVEL_WARNING    2
#define MINDVISION_LOG_LEVEL_INFO       3

#ifndef MINDVISION_LOG_LEVEL
#define MINDVISION_LOG_LEVEL    MINDVISION_LOG_LEVEL_WARNING
#endif

/*** 异常抛出 ***/
#define MINDVISION_THROW_IMPL(file, line, str, ...) \
        throw MindVision_Exception(fmt::format("[ERROR {}:{}] " str, file, line, ##__VA_ARGS__))
/*** 异常抛出包装器 ***/
#define MINDVISION_THROW(str, ...) MINDVISION_THROW_IMPL(__FILE__, __LINE__, str, ##__VA_ARGS__)

/*** 打印LOG ***/
#define MINDVISION_LOG_IMPL(ostream, type, file, line, str, ...) \
        ostream << fmt::format("[{} {}:{}] " str, #type, file, line, ##__VA_ARGS__) << std::endl
/*** 打印LOG包装器 ***/
#define MINDVISION_LOG(ostream, type, str, ...) MINDVISION_LOG_IMPL(ostream, type, __FILE__, __LINE__, str, ##__VA_ARGS__)

/*** 打印ERROR级别的LOG ***/
#if MINDVISION_LOG_LEVEL >= MINDVISION_LOG_LEVEL_ERROR
#define MINDVISION_ERROR(str, ...) MINDVISION_LOG(std::cerr, ERROR, str, ##__VA_ARGS__)
#else
#define MINDVISION_ERROR(str, ...) ((void)0)
#endif
/*** 打印WARNING级别的LOG ***/
#if MINDVISION_LOG_LEVEL >= MINDVISION_LOG_LEVEL_WARNING
#define MINDVISION_WARNING(str, ...) MINDVISION_LOG(std::cerr, WARNING, str, ##__VA_ARGS__)
#else
#define MINDVISION_WARNING(str, ...) ((void)0)
#endif
/*** 打印INFO级别的LOG ***/
#if MINDVISION_LOG_LEVEL >= MINDVISION_LOG_LEVEL_INFO
#define MINDVISION_INFO(str, ...) MINDVISION_LOG(std::cout, INFO, str, ##__VA_ARGS__)
#else
#define MINDVISION_INFO(str, ...) ((void)0)
#endif

/*** 断言，失败抛出异常 ***/
#define MINDVISION_ASSERT(expr) ((static_cast<bool>(expr)) ? \
                                ((void)0) : \
                                (MINDVISION_THROW("assert fail: " #expr)))
/*** 检查API返回值 ***/
#define MINDVISION_CHECK_API(type, expr) do{                 \
        CameraSdkStatus status = expr;                      \
        if(status != CAMERA_STATUS_SUCCESS){                  \
            MINDVISION_##type(#expr " return ({})", status); \
        }                                                   \
}while(0)
/*** 检查API返回值，错误则抛出异常 ***/
#define MINDVISION_CHECK_API_THROW(expr)    MINDVISION_CHECK_API(THROW, expr)
/*** 检查API返回值，错误则打印ERROR日志 ***/
#define MINDVISION_CHECK_API_ERROR(expr)    MINDVISION_CHECK_API(ERROR, expr)
/*** 检查API返回值，错误则打印WARNING日志 ***/
#define MINDVISION_CHECK_API_WARNING(expr)  MINDVISION_CHECK_API(WARNING, expr)

/*** 当析构时自动执行某个函数 ***/
class AutoExecute {
public:
    explicit AutoExecute(std::function<void()> f) : func(std::move(f)) {}

    ~AutoExecute() { func(); }

private:
    std::function<void()> func;
};


struct MindVision_Exception : public std::runtime_error {
    using std::runtime_error::runtime_error;
};

class MindVision {
    friend void local_callback(CameraHandle hCamera, BYTE *pFrameBuffer, tSdkFrameHead *pFrameHead, PVOID pContext);

public:
    explicit MindVision(std::string_view cfg = "");

    MindVision(const MindVision &) = delete;

    MindVision &operator=(const MindVision &other) = delete;

    ~MindVision();

    void load_config(std::string_view cfg);

    bool open();

    bool isOpened() const;

    void close();

    void trigger() const;

    void read(cv::Mat &src, size_t timeout_ms = 1000) const;

    CameraHandle getCameraHandle() const;

private:
    std::string camera_name;
    std::string camera_config;
    uint8_t *camera_buffer{nullptr};
    CameraHandle h_camera{0};

    mutable std::mutex mtx;
    mutable std::condition_variable cv;
    mutable cv::Mat cache;
    mutable bool available{false};
};

#endif /* _MINDVISION_HPP_ */
