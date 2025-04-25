#pragma once
#include "config.h"
/*
@brief 错误处理函数，如果未识别到目标，则进行前后左右移动再进行识别，防止卡死
@param errorCode 错误代码
*/
void errorHandle(int errorCode);
