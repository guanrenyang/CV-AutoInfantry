#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/eigen.hpp>
#include <boost/timer/timer.hpp>
#include "StereoMV.hpp"
#include "Astar.h"
#include "Astar.cpp"
#include <serial/serial.h>

Eigen::Vector3d Right(0,0,1),rightDown(1,0,1),Down(1,0,0),leftDown(1,0,-1),Left(0,0,-1),leftUp(-1,0,-1),Up(-1,0,0),rightUp(-1,0,1);
vector<vector<int>> Map(60,vector<int>(60,0));
Eigen::Vector3d track(Eigen::Vector3d worldCoordinate)//由于精度为0.05cm,故构建地图时单位长度为0.05cm,地图为3m*3m,对应到图中为60*60
{
    const double tagZ=0;
    const double tagX=30;
    //将tag坐标系转化到世界坐标系,tag-z平行于world-z,tag-x平行于world-z,tag-O的世界坐标为(tagX,0.tagZ)
    //坐标变换式,方向余弦阵的普世解法忘记了,以下变换如果有错误可以推倒一下
    double tempZ= worldCoordinate[3]+tagZ;
    double tempX= -worldCoordinate[1]+tagX;

    int Z=int(tempZ*100/5);
    int X=int(tempX*100/5);

    //定义Astar类的对象
    Astar astar;
    astar.InitAstar(Map);

    //定义起始位置和终止位置
    //测试数据:空图的左下角到右上角
    Note start(Z,X);
    Note end(60,0);

    //Astar寻迹
    vector<Note *> path = astar.GetPath(start, end );
    if(path.empty())
    {//给电控调试使用的语句
        /*ser.write("no path")*/
    }
    else
    {
        /*
         * ser.write(p->x);
         * ser.write(p->y);
         * */

        for(auto &p : path) {
            if (p->parent == NULL)
                continue;
            if (p->parent->x == p->x - 1 && p->parent->y == p->y - 1)//右下走
            {
                return rightDown;
            } else if (p->parent->x == p->x - 1 && p->parent->y == p->y)//向下走
            {
                return Down;
            } else if (p->parent->x == p->x - 1 && p->parent->y == p->y + 1)//左下走
            {
                return leftDown;
            } else if (p->parent->x == p->x && p->parent->y == p->y + 1)//向左走
            {
                return Left;
            } else if (p->parent->x == p->x + 1 && p->parent->y == p->y + 1)//左上走
            {
                return leftUp;
            } else if (p->parent->x == p->x + 1 && p->parent->y == p->y)//向上走
            {
                return Up;
            } else if (p->parent->x == p->x + 1 && p->parent->y == p->y - 1)//右上走
            {
                return rightUp;
            } else if (p->parent->x == p->x && p->parent->y == p->y - 1)//右走
            {
                return Right;
            }
        }
    }
}
//turn left >0;turn right <0
double computeAngle(Eigen::Vector3d carDirection,Eigen::Vector3d moveDirection)
{
    double carY=-carDirection[0];
    double carX=carDirection[3];
    double carAngle = atan2(carY,carX);

    double moveY=-moveDirection[0];
    double moveX=moveDirection[3];
    double moveAngle = atan2(moveY,moveX);

    return ((moveAngle-carAngle)*180.0/3.1415);
}
int main() {
    MindVision cap("../config/config-0.yml");
    assert(cap.open());

    cv::Mat K, C;
    cv::FileStorage fs("../config/config-0.yml", cv::FileStorage::READ);
    fs["K"] >> K;
    fs["C"] >> C;

    cv::Ptr<cv::aruco::Dictionary> dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_25h9);

    serial::Serial s("/COM3", 115200 );
    s.open();

    int cnt = 0;
    int k = 0;
    while (k != 'q') {
        cv::Mat img, raw;
        cap.read(img);
        raw = img.clone();

        {
            boost::timer::auto_cpu_timer timer("detect&estimate: %ws\n");
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            cv::aruco::detectMarkers(img, dict, corners, ids);
            if (!ids.empty()) {
                cv::aruco::drawDetectedMarkers(img, corners, ids);
                std::vector<cv::Vec3d> rvecs, tvecs;
                cv::aruco::estimatePoseSingleMarkers(corners, 0.385, K, C, rvecs, tvecs);
                for (int i = 0; i < ids.size(); i++) {
                    cv::Mat R;
                    cv::Rodrigues(rvecs[i], R);
                    Eigen::Vector3d t_cw;
                    Eigen::Matrix3d R_cw;
                    cv::cv2eigen(tvecs[i], t_cw);
                    cv::cv2eigen(R, R_cw);
                    Eigen::Matrix4d T_cw = Eigen::Matrix4d::Ones();
                    T_cw.block<3, 3>(0, 0) = R_cw;
                    T_cw.block<3, 1>(0, 3) = t_cw;
                    Eigen::Vector4d P_c{0, 0, 0, 1};
                    Eigen::Vector4d P_l = T_cw.inverse() * P_c;

                    //将方向偏角矩阵转化为word coordinate
                    Eigen::Vector3d p_c1{0, 0, 1};
                    Eigen::Vector3d p_l1 = R_cw.transpose() * p_c1;
                    Eigen::Matrix3d A_wl;//tag zuo biao xi dao shi jie zuo biao xi de bian huan zhen
                    Eigen::Vector3d p_w1=A_wl * p_l1;
                    Eigen::Vector3d p_w = P_l.block<3, 1>(0, 0) / P_l(3, 0);
                    
                    Eigen::Vector3d direction=track(p_w);
                    double angle=computeAngle(p_w1,direction);
                    string st=to_string(angle);
                    s.write(st);

                    std::cout << ids[i] << ": " << p_w.norm() << " | " << p_w.transpose() << std::endl;
                    cv::aruco::drawAxis(img, K, C, rvecs[i], tvecs[i], 0.1);
                }
            }
        }

        cv::imshow("img", img);
        k = cv::waitKey(10);
        if (k == 's') {
            cv::imwrite(std::to_string(cnt++) + ".png", raw);
        }
    }

    return 0;
}
