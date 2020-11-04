//
// Created by guanrenyang on 2020/10/13.
//
#include "Astar.h"
#include<cmath>
#include"Astar.h"
#include<opencv2/opencv.hpp>
using namespace std;

inline int multipTen(int n)
{
    return (n<<3)+(n<<1);
}
void Astar::InitAstar(vector<vector<int>>  &_map)
{
    map = _map;
    writer.open("Astar.avi", cv::VideoWriter::fourcc('M','J','P','G'), 10, Size(675, 675), true);
    img.create(map.size(), map[0].size(), CV_8UC3);
    for (int i = 0; i < img.rows; i++)//障碍物设置为黑色，路径设置为红色
        for (int j = 0; j < img.cols; j++)
        {
            if(map[i][j]==0)
                img.at<Vec3b>(i, j) = Vec3b(255,255,255);
            else
                img.at<Vec3b>(i, j) = Vec3b(0,0,0);
        }
}

bool Astar::isInList(const vector<Note *> &list, const Note *note) const
{
    for (auto p : list)
        if (p->x == note->x && p->y == note->y)
            return true;
    return false;
}

bool Astar::isReachable(const Note *currentNote, const Note *targetNote) const
{
    //如果点超出地图、不是八个方向、是障碍物、或者在闭列表中，返回false。反之。
    if (targetNote->x < 0 || targetNote->x > (int)(map.size() - 1)
        || targetNote->y < 0 || targetNote->y > (int)(map[0].size() - 1)
        || map[targetNote->x][targetNote->y] == 1
        || isInList(closeList, targetNote)
        //下面这部分的逻辑结构可以优化，我想得绕进去了
        ||!((targetNote->x==currentNote->x-1&&targetNote->y==currentNote->y-1)//左上角
        ||(targetNote->x==currentNote->x-1&&targetNote->y==currentNote->y)//上
        ||(targetNote->x==currentNote->x-1&&targetNote->y==currentNote->y+1)//右上角
        ||(targetNote->x==currentNote->x&&targetNote->y==currentNote->y+1)//右
        ||(targetNote->x==currentNote->x+1&&targetNote->y==currentNote->y+1)//右下角
        ||(targetNote->x==currentNote->x+1&&targetNote->y==currentNote->y)//下
        ||(targetNote->x==currentNote->x+1&&targetNote->y==currentNote->y-1)//左下角
        ||(targetNote->x==currentNote->x&&targetNote->y==currentNote->y-1)//左
        ))
        return false;
    else
        return true;
}

vector<Note *> Astar::getSurroundNotes(const Note *currentNote) const//得到当前结点可到达结点的vector
{
    vector<Note *> surroundNotes;
    for (int x = currentNote->x - 1; x <= currentNote->x + 1; ++x)
        for (int y = currentNote->y - 1; y <= currentNote->y + 1; ++y)
            if (isReachable(currentNote, new Note(x, y)))
                surroundNotes.push_back(new Note(x, y));
    return surroundNotes;
}

Note *Astar::getLeastFNote()//找openlist中的最小F值结点
{
    if (!openList.empty())
    {
        auto minFNote = openList.front();
        for (auto &note : openList)
            if (note->F < minFNote->F)
                minFNote = note;
        return minFNote;
    }
    return NULL;
}
int Astar::calcG( Note *note)
{

    int parentG;
    if(note->parent == NULL) //如果是初始节点，则其父节点是空
    {
        parentG = 0;
        return (parentG+10);
    }
    else if(
            (note->x==note->parent->x-1&&note->y==note->parent->y-1)//左上
            ||(note->x==note->parent->x-1&&note->y==note->parent->y+1)//右上
            ||(note->x==note->parent->x+1&&note->y==note->parent->y-1)//左下
            ||(note->x==note->parent->x+1&&note->y==note->parent->y+1)//右下
            )
        return (parentG+14);
    else
        return (parentG+10);
}
int Astar::calcH(Note *note, Note *end)
{
    return multipTen(abs(end->x - note->x)+ abs(end->y - note->y));
}
int Astar::calcF(Note *note)
{
    return note->G + note->H;
}
void Astar::deleteNote(vector<Note *> &list, Note *note)
{
    int pos=0;
    for (auto i = 0; i != list.size(); ++i)
    {
        if (list[i]->x == note->x && list[i]->y == note->y)
            break;
        ++pos;
    }
    list.erase(list.begin()+pos);
}

Note *Astar::findPath(Note &startNote, Note &endNote)
{
    //标记开始坐标和终止坐标
    img.at<Vec3b>(startNote.x, startNote.y) = Vec3b(0, 0, 255);
    img.at<Vec3b>(endNote.x, endNote.y) = Vec3b(0, 0, 255);

    openList.push_back(new Note(startNote.x, startNote.y)); //起点放入开集
    while (!openList.empty())
    {
        /*currentNote即探测到的局部最优结点*/
        auto currentNote = getLeastFNote(); //找到F值最小的点
        deleteNote(openList, currentNote);   //从开集中删除
        closeList.push_back(currentNote);   //放到关闭集

        img.at<Vec3b>(currentNote->x, currentNote->y) = Vec3b(0, 0, 255);//将局部最优结点标记为黄色
        //处理图片
        resize(img, resize_img, Size(675, 675), 0, 0, 3);
        writer << resize_img;
        imshow("find path", resize_img);
        waitKey(120);

        auto surroundPoints = getSurroundNotes(currentNote);//寻找周围点,surroudPoints为currentnode周围可到达结点组成的vector
        for (auto &target : surroundPoints)
        {
            //对某一个格子，如果它不在开启列表中，加入到开启列表，设置当前格为其父节点，计算FGH
            if (!isInList(openList, target))
            {
                target->parent = currentNote;
                target->G = calcG(target);
                target->H = calcH(target, &endNote);
                target->F = calcF(target);
                openList.push_back(target);
                img.at<Vec3b>(target->x,target->y) = Vec3b(0, 255, 255);
            }
            //对某一个格子，它在开启列表中，计算G值, 如果比原来的大, 就什么都不做,（说明沿着currentnode前的某一结点走到target 比从currentNode走到target更优）
            // 否则设置它的父节点为当前点,并更新G和F
            else
            {
                int tempG = calcG(target);
                if (tempG < target->G)
                {
                    target->parent = currentNote;
                    target->G = tempG;
                    target->F = calcF(target);
                }
            }
            //如果终点出现在开集中，表明找到了路径，并返回。
            if (isInList(openList, &endNote))
                return target; //返回列表里的节点指针
        }
        //绘图，把当前探测到底结点标记为绿色
        img.at<Vec3b>(currentNote->x, currentNote->y) = Vec3b(0,255, 0);
        resize(img, resize_img, Size(675, 675), 0, 0, 3);
        writer << resize_img;
        imshow("find path", resize_img);
        waitKey(20);

    }
    return NULL;
}

vector<Note *> Astar::GetPath(Note &starNote, Note &endNote)
{
    Note *result = findPath(starNote, endNote);
    vector<Note *> path;
    //返回路径，如果没找到路径，返回空
    while (result)
    {
        //将找到的路径上的结点标记为蓝色
        img.at<Vec3b>(result->x, result->y) = Vec3b(255, 0, 0);
        resize(img, resize_img, Size(675, 675), 0, 0, 3);
        writer << resize_img;
        imshow("find path", resize_img);
        waitKey(30);

        path.insert(path.begin(), result);//一直在vector头插入，最终path为从起点到终点的路径

        result = result->parent;
    }
    writer.release();
    return path;
}