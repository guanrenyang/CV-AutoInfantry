#include <iostream>
#include <boost/timer/timer.hpp>
#include <opencv2/opencv.hpp>
#include "StereoMV.hpp"

int main() {
    StereoMV<2> stereo({EXAMPLE_PATH"/config/config-0.yml", EXAMPLE_PATH"/config/config-1.yml"});
    assert(stereo.open());

    cv::VideoWriter v0("./0.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 50, {1280, 1024}, true);
    cv::VideoWriter v1("./1.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 50, {1280, 1024}, true);

    do {
        cv::Mat src0, src1;
        {
            boost::timer::auto_cpu_timer timer("trigger&read: %ws\n");
            stereo.trigger();
            stereo.read({&src0, &src1});
        }

        cv::imshow("0", src0);
        cv::imshow("1", src1);
        v0.write(src0);
        v1.write(src1);
    } while (cv::waitKey(1) != 'q');
    v0.release();
    v1.release();
    return 0;
}
