/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "image_transport/single_subscriber_publisher.h"
#include "image_transport/publisher.h"
#include <ros/subscriber_link.h>
#include <ros/transport_subscriber_link.h>

namespace image_transport {

SingleSubscriberPublisher::SingleSubscriberPublisher(const std::string& caller_id, const std::string& topic,
                                                     const GetNumSubscribersFn& num_subscribers_fn,
                                                     const PublishFn& publish_fn)
  : caller_id_(caller_id), topic_(topic),
    num_subscribers_fn_(num_subscribers_fn),
    publish_fn_(publish_fn),
    link_(NULL)
{
}

// Changed by Johnson
// 
// 2020.02.15 17:51
// 实现重载的构造函数
SingleSubscriberPublisher::SingleSubscriberPublisher(const std::string& caller_id, const std::string& topic,
                                                    const GetNumSubscribersFn& num_subscribers_fn,
                                                    const PublishFn& publish_fn,
                                                    ros::SubscriberLink* link) 
 : caller_id_(caller_id), topic_(topic),
   num_subscribers_fn_(num_subscribers_fn),
   publish_fn_(publish_fn),
   link_(link)
{
  // Debug
  // std::cout << link_ << std::endl;
}
// end

std::string SingleSubscriberPublisher::getSubscriberName() const
{
  return caller_id_;
}

std::string SingleSubscriberPublisher::getTopic() const
{
  return topic_;
}

uint32_t SingleSubscriberPublisher::getNumSubscribers() const
{
  return num_subscribers_fn_();
}

void SingleSubscriberPublisher::publish(const sensor_msgs::Image& message) const
{
  publish_fn_(message);
}

void SingleSubscriberPublisher::publish(const sensor_msgs::ImageConstPtr& message) const
{
  publish_fn_(*message);
}

} //namespace image_transport
