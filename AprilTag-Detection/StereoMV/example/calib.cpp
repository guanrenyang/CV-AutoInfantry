//
// Created by xinyang on 2020/10/12.
//

#include <iostream>
#include <fmt/ostream.h>
#include <fmt/format.h>
#include <opencv2/opencv.hpp>
#include "StereoMV.hpp"

int main(int argc, char *argv[]) {
    int w, h;
    double mm;
    if (argc != 4) {
        std::cout << fmt::format("usage: {} <chess board width> <chess board height> <block size(mm)>", argv[0])
                  << std::endl;
        return -1;
    } else {
        std::istringstream(argv[1]) >> h;
        std::istringstream(argv[2]) >> w;
        std::istringstream(argv[3]) >> mm;
        std::cout << fmt::format("begin calibration with chess board size=[{}x{}], block size={}mm", w, h, mm)
                  << std::endl;
    }

    // 世界坐标点
    std::vector<cv::Point3f> obj(w * h);
    for (int j = 0; j < h; j++) {
        for (int i = 0; i < w; i++) {
            obj[j * w + i] = cv::Point3f(mm * i, mm * j, 0);
        }
    }
    // 世界坐标点序列
    std::vector<std::vector<cv::Point3f>> obj_seq;
    // 棋盘角点序列
    std::vector<std::vector<cv::Point2f>> corners_seq[2];
    // 指定亚像素计算迭代标注
    cv::TermCriteria criteria = cv::TermCriteria(
            cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,
            40, 0.01);

    StereoMV<2> stereo({EXAMPLE_PATH"/config/config-0.yml", EXAMPLE_PATH"/config/config-1.yml"});
    assert(stereo.open());

    int k = 0;
    cv::Mat src[2], gray[2], im2show[2];
    while (k != 'q') {
        // 读取图像
        stereo.trigger();
        stereo.read({src + 0, src + 1});
        // 显示原图
        cv::resize(src[0], im2show[0], {}, 0.5, 0.5);
        cv::resize(src[1], im2show[1], {}, 0.5, 0.5);
        cv::imshow("raw-0", im2show[0]);
        cv::imshow("raw-1", im2show[1]);
        k = cv::waitKey(1);
        if (k != 's') continue;

        // 进行角点检测
        std::vector<cv::Point2f> corners[2];
        cv::cvtColor(src[0], gray[0], cv::COLOR_BGR2GRAY);
        cv::cvtColor(src[1], gray[1], cv::COLOR_BGR2GRAY);
        if (!cv::findChessboardCorners(gray[0], {w, h}, corners[0])) continue;
        if (!cv::findChessboardCorners(gray[1], {w, h}, corners[1])) continue;
        cv::cornerSubPix(gray[0], corners[0], {5, 5}, {-1, -1}, criteria);
        cv::cornerSubPix(gray[1], corners[1], {5, 5}, {-1, -1}, criteria);
        // 显示检测结果
        cv::drawChessboardCorners(src[0], {w, h}, corners[0], true);
        cv::drawChessboardCorners(src[1], {w, h}, corners[1], true);
        cv::resize(src[0], im2show[0], {}, 0.5, 0.5);
        cv::resize(src[1], im2show[1], {}, 0.5, 0.5);
        cv::imshow("found-0", im2show[0]);
        cv::imshow("found-1", im2show[1]);
        if (cv::waitKey(0) != 's') continue;

        corners_seq[0].emplace_back(std::move(corners[0]));
        corners_seq[1].emplace_back(std::move(corners[1]));
        obj_seq.emplace_back(obj);
    }
    cv::destroyAllWindows();
    std::cout << fmt::format("collect {} images, calibrating...", obj_seq.size()) << std::endl;
    auto[iw, ih] = src[0].size();
    std::cout << fmt::format("image size: [{}x{}]", iw, ih) << std::endl;

    cv::Mat K[2], C[2];
    cv::Mat R, T, E, F;

    double err = cv::stereoCalibrate(obj_seq, corners_seq[0], corners_seq[1],
                                     K[0], C[0], K[1], C[1], {iw, ih}, R, T, E, F,
                                     cv::CALIB_FIX_PRINCIPAL_POINT | cv::CALIB_FIX_ASPECT_RATIO);
    std::cout << fmt::format("re-projection error: {}", err) << std::endl;
    std::cout << fmt::format("K[0]:\n {}", K[0]) << std::endl;
    std::cout << fmt::format("C[0]:\n {}", C[0]) << std::endl;
    std::cout << fmt::format("K[1]:\n {}", K[1]) << std::endl;
    std::cout << fmt::format("C[1]:\n {}", C[1]) << std::endl;
    std::cout << fmt::format("R:\n {}", R) << std::endl;
    std::cout << fmt::format("t:\n {}", T) << std::endl;

    cv::FileStorage fs("param.yml", cv::FileStorage::WRITE);
    fs << "K_0" << K[0];
    fs << "C_0" << C[0];
    fs << "K_1" << K[1];
    fs << "C_1" << C[1];
    fs << "R" << R;
    fs << "t" << T;
}

