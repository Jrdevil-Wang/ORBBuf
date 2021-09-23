#include <image_content_algorithm.h>

void InitRandom() {
    struct timeval tv;
    struct timezone tz;
    gettimeofday(&tv, &tz);

    srand((unsigned int)(tv.tv_sec*1000000 +tv.tv_usec));
}

double CompareImageContent(sensor_msgs::ImagePtr image_1, sensor_msgs::ImagePtr image_2) {
    // 暂时用随机数代替
    InitRandom();

    return rand() % 1000 / 1000.0;
}