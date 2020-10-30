#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/eigen.hpp>
#include <boost/timer/timer.hpp>
#include "StereoMV.hpp"
#include "Astar.h"
#include "Astar.cpp"
const int Right=3,rightDown=4,down=5,leftDown=6,Left=7,leftUp=8,up=9,rightUp=10;
vector<vector<int>> Map(60,vector<int>(60,0));
void track(Eigen::Vector3d worldCoordinate)//由于精度为0.05cm,故构建地图时单位长度为0.05cm,地图为3m*3m,对应到图中为60*60
{
    const int tagX=0;
    const int tagY=30;
    //将tag坐标系转化到世界坐标系,tag-z平行于world-x,tag-x平行于world-y,tag-O的世界坐标为(tagX,tagY)
    //坐标变换式,方向余弦阵的普世解法忘记了,以下变换如果有错误可以推倒一下
    int x=worldCoordinate[3]+tagX;
    int y=-worldCoordinate[1]+tagY;
    //定义Astar类的对象
    Astar astar;
    astar.InitAstar(Map);

    //定义起始位置和终止位置
    //测试数据:空图的左下角到右上角
    Note start(0,60);
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
        for(auto &p : path)
        {
            if(p->parent==NULL)
                continue;
            if(p->parent->x==p->x-1&&p->parent->y==p->y-1)//右下走
                Map[p->parent->x][p->parent->y] = rightDown;
            else if(p->parent->x==p->x-1&&p->parent->y==p->y)//向下走
                Map[p->parent->x][p->parent->y] = down;
            else if(p->parent->x==p->x-1&&p->parent->y==p->y+1)//左下走
                Map[p->parent->x][p->parent->y] = leftDown;
            else if(p->parent->x==p->x&&p->parent->y==p->y+1)//向左走
                Map[p->parent->x][p->parent->y] = Left;
            else if(p->parent->x==p->x+1&&p->parent->y==p->y+1)//左上走
                Map[p->parent->x][p->parent->y] = leftUp;
            else if(p->parent->x==p->x+1&&p->parent->y==p->y)//向上走
                Map[p->parent->x][p->parent->y] = up;
            else if(p->parent->x==p->x+1&&p->parent->y==p->y-1)//右上走
                Map[p->parent->x][p->parent->y] = rightUp;
            else if(p->parent->x==p->x&&p->parent->y==p->y-1)//右走
                Map[p->parent->x][p->parent->y] = Right;

            /*ser.write(Map[p->parent->x][p->parent->y]);
            break;
             */

        }
    }

}

int main() {
    MindVision cap("../config/config-0.yml");
    assert(cap.open());

    cv::Mat K, C;
    cv::FileStorage fs("../config/config-0.yml", cv::FileStorage::READ);
    fs["K"] >> K;
    fs["C"] >> C;

    cv::Ptr<cv::aruco::Dictionary> dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_25h9);

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
                    Eigen::Vector4d P_w = T_cw.inverse() * P_c;
                    //将方向偏角矩阵转化为word coordinate
                    Eigen::Vector3d p_c1{0, 0, 1};
                    Eigen::Vector3d p_w1 = R_cw.transpose() * p_c1;


                    Eigen::Vector3d p_w = P_w.block<3, 1>(0, 0) / P_w(3, 0);
                    /*
                     * ser.write(p_w1);
                     */
                    track(p_w);


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
