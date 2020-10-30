//
// Created by guanrenyang on 2020/10/13.
//

#ifndef OCV_ASTAR_H
#define OCV_ASTAR_H

#endif //OCV_ASTAR_H
#pragma once
#include<vector>
#include<opencv2/opencv.hpp>
#include<bits/stdc++.h>
using namespace  std;
using namespace cv;
struct Note
{
    int x,y;
    int F,G,H;
    Note *parent;
    Note(int _x, int _y) :x(_x), y(_y), F(0), G(0), H(0), parent(NULL) {}  //变量初始化
};

class Astar
{
public:
    Mat img,resize_img;
    void InitAstar(vector<vector<int>> &_map);            //初始化图
    vector<Note *> GetPath(Note &starNote, Note &endNote);//获得最短的路径

private:
    vector<vector<int>> map;     //存放地图
    VideoWriter writer;
    vector<Note *> openList;     //开集
    vector<Note *> closeList;    //闭集

    Note *findPath(Note &startNote, Note &endNote);//找最短的路径
    vector<Note *> getSurroundNotes(const Note *currentNote) const;//遍历当前点的周围点
    bool isReachable(const Note *currentNote, const Note *targetNote) const; //判断某点是否可以用于下一步判断
    bool isInList(const vector<Note *> &list, const Note *note) const; //判断开/闭列表中是否包含某点
    void deleteNote(vector<Note *> &list,Note *note); //删除点
    Note *getLeastFNote(); //从开列表中返回F值最小的节点
    int calcG(Note *note);//计算FGH值
    int calcH(Note *note, Note *end);
    int calcF(Note *note);
};