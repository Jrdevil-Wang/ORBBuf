#ifndef IMAGE_CONTENT_ALGORITHM_H
#define IMAGE_CONTENT_ALGORITHM_H

#include <sys/time.h>
#include <stdlib.h>
#include <time.h>
#include <sensor_msgs/Image.h>

// 用于初始化随机数种子，可达微秒级系统时间
void InitRandom();

// 图片对比函数，暂时用随机数实现
double CompareImageContent(sensor_msgs::ImagePtr image_1, sensor_msgs::ImagePtr image_2);

#endif