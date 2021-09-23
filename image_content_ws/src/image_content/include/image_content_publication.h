#ifndef IMAGE_CONTENT_PUBLICATION_H
#define IMAGE_CONTENT_PUBLICATION_H

#include <iostream>

#include <ros/ros.h>
#include <ros/forwards.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <ros/publication.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <string.h>
#include <pthread.h>
#include <stdlib.h>
#include <vector>

#include <image_content_sublinker.h>

class ImageContentPublication {
public:
    ImageContentPublication() {}
    ~ImageContentPublication() {}

    ImageContentPublication(image_transport::ImageTransport& im_pub, const std::string& topic_name, uint32_t queue_size);

    void publish(sensor_msgs::ImagePtr message);

    void addSubLinker(const image_transport::SingleSubscriberPublisher& sub_link);

    void removeSubLinker(const image_transport::SingleSubscriberPublisher& sub_link);
private:
    image_transport::Publisher im_pub_;
    int max_queue_;

    std::vector<ros::SubscriberLink*> sub_linkers_;
    std::vector<ImageContentSubLinker*> im_sub_linkers_;
    boost::mutex im_sub_linkers_mutex_;
};

#endif