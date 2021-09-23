#include <image_content_publication.h>
#include <boost/foreach.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <ros/transport_subscriber_link.h>
#include <image_transport/single_subscriber_publisher.h>
#include <algorithm>

ImageContentPublication::ImageContentPublication(image_transport::ImageTransport& im_pub, const std::string& topic_name, uint32_t queue_size) {

    image_transport::SubscriberStatusCallback add_callback = boost::bind(&ImageContentPublication::addSubLinker, this, _1);
    image_transport::SubscriberStatusCallback remove_callback = boost::bind(&ImageContentPublication::removeSubLinker, this, _1);

    im_pub_ = im_pub.advertise(topic_name, 1, add_callback, remove_callback);
    max_queue_ = queue_size;
}

void ImageContentPublication::publish(sensor_msgs::ImagePtr message) {
    boost::mutex::scoped_lock lock(im_sub_linkers_mutex_);
    std::cout << "im_sub_linkers.size = " << im_sub_linkers_.size() << std::endl;
    BOOST_FOREACH(ImageContentSubLinker* sub, im_sub_linkers_) {
        sub->enqueueMessage(message);
    }
}

void ImageContentPublication::addSubLinker(const image_transport::SingleSubscriberPublisher& sub_link) {
    std::cout << "Add Sub Linker." << std::endl;

    if (sub_link.link_ == NULL) return;
    boost::mutex::scoped_lock lock(im_sub_linkers_mutex_);

    ImageContentSubLinker* im_sub = new ImageContentSubLinker();

    im_sub->setMaxQueue(max_queue_);
    im_sub->setImageTransportPub(&im_pub_);

    im_sub_linkers_.push_back(im_sub);
    sub_linkers_.push_back(sub_link.link_);
    sub_link.link_->setUserCb(boost::bind(&ImageContentSubLinker::onMessageWritten, im_sub));
}

void ImageContentPublication::removeSubLinker(const image_transport::SingleSubscriberPublisher& sub_link) {
    std::cout << "Remove Sub Linker." << std::endl;
    
    boost::mutex::scoped_lock lock(im_sub_linkers_mutex_);
 
    std::vector<ros::SubscriberLink*>::iterator it = std::find(sub_linkers_.begin(), sub_linkers_.end(), sub_link.link_);
    if (it != sub_linkers_.end()) {
        int dist = it - sub_linkers_.begin();
        sub_linkers_.erase(it);
        im_sub_linkers_.erase(im_sub_linkers_.begin() + dist);
    }
}