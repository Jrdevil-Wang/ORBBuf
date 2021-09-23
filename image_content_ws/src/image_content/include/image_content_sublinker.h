#ifndef IMAGE_CONTENT_SUBLINKER_H
#define IMAGE_CONTENT_SUBLINKER_H

#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/publication.h>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <iostream>
#include <pthread.h>
#include <stdlib.h>

#include <boost/thread/mutex.hpp>

#include <queue>
#include <vector>

struct ImageAndContent {
    sensor_msgs::ImagePtr image;
    double val;

    ImageAndContent(sensor_msgs::ImagePtr img, double v) {
        image = img;
        val = v;
    }
};

class ImageContentSubLinker {
public:
    ImageContentSubLinker() : is_writting_(false), last_frame_(new sensor_msgs::Image()) {}
    ~ImageContentSubLinker() {}

    ImageContentSubLinker(const ImageContentSubLinker& A);

    void enqueueMessage(sensor_msgs::ImagePtr message);

    void setMaxQueue(int max_queue);

    void setImageTransportPub(image_transport::Publisher* pub);

    void onMessageWritten();

    void startWriteMessage();

    int getOutboxSize() { return outbox_.size(); }
private:
    std::vector<ImageAndContent*> outbox_;
    boost::mutex out_mutex_;
    
    int max_queue_;
    image_transport::Publisher* pub_;

    bool is_writting_;

    // 保存上一个到来的图片，用于计算队列首个元素的相似度
    // 可以考虑保存在队列的0号里，之后再说
    sensor_msgs::ImagePtr last_frame_;
};

#endif