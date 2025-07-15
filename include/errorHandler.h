#pragma once
#include "config.h"
#include "mainSequence.h"

// 声明外部全局变量 - 任务码数组
extern int taskcode[2][3];

// 声明外部全局变量 - 色环识别结果
extern int circleX, circleY;
extern bool circleFound;

/*
@brief 错误处理函数，如果未识别到目标，则进行前后左右移动再进行识别，防止卡死
@param errorCode 错误代码
*/
void errorHandle(int errorCode);
