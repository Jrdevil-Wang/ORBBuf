
/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/transport_subscriber_link.h"
#include "ros/publication.h"
#include "ros/header.h"
#include "ros/connection.h"
#include "ros/transport/transport.h"
#include "ros/this_node.h"
#include "ros/connection_manager.h"
#include "ros/topic_manager.h"
#include "ros/file_log.h"

#include <boost/bind.hpp>

// 加入随机数头文件
#include <sys/time.h>
#include <time.h>
#include <stdlib.h>

// 队列头文件
#include <queue>

#include <string.h>


namespace ros
{

TransportSubscriberLink::TransportSubscriberLink()
: writing_message_(false)
, header_written_(false)
, queue_full_(false)
{
  // Changed by Johnson
  // 设置用户函数指针为空
  // user_callback_.clear();
}

TransportSubscriberLink::~TransportSubscriberLink()
{
  drop();
}

bool TransportSubscriberLink::initialize(const ConnectionPtr& connection)
{
  connection_ = connection;
  dropped_conn_ = connection_->addDropListener(boost::bind(&TransportSubscriberLink::onConnectionDropped, this, _1));

  return true;
}

bool TransportSubscriberLink::handleHeader(const Header& header)
{
  std::string topic;
  if (!header.getValue("topic", topic))
  {
    std::string msg("Header from subscriber did not have the required element: topic");

    ROS_ERROR("%s", msg.c_str());
    connection_->sendHeaderError(msg);

    return false;
  }

  // This will get validated by validateHeader below
  std::string client_callerid;
  header.getValue("callerid", client_callerid);
  PublicationPtr pt = TopicManager::instance()->lookupPublication(topic);
  if (!pt)
  {
    std::string msg = std::string("received a connection for a nonexistent topic [") +
                    topic + std::string("] from [" + connection_->getTransport()->getTransportInfo() + "] [" + client_callerid +"].");

    ROSCPP_LOG_DEBUG("%s", msg.c_str());
    connection_->sendHeaderError(msg);

    return false;
  }

  std::string error_msg;
  if (!pt->validateHeader(header, error_msg))
  {
    ROSCPP_LOG_DEBUG("%s", error_msg.c_str());
    connection_->sendHeaderError(error_msg);

    return false;
  }

  destination_caller_id_ = client_callerid;
  connection_id_ = ConnectionManager::instance()->getNewConnectionID();
  topic_ = pt->getName();
  parent_ = PublicationWPtr(pt);

  // Send back a success, with info
  M_string m;
  m["type"] = pt->getDataType();
  m["md5sum"] = pt->getMD5Sum();
  m["message_definition"] = pt->getMessageDefinition();
  m["callerid"] = this_node::getName();
  m["latching"] = pt->isLatching() ? "1" : "0";
  m["topic"] = topic_;
  connection_->writeHeader(m, boost::bind(&TransportSubscriberLink::onHeaderWritten, this, _1));

  pt->addSubscriberLink(shared_from_this());

  return true;
}

void TransportSubscriberLink::onConnectionDropped(const ConnectionPtr& conn)
{
  (void)conn;
  ROS_ASSERT(conn == connection_);

  PublicationPtr parent = parent_.lock();

  if (parent)
  {
    ROSCPP_CONN_LOG_DEBUG("Connection to subscriber [%s] to topic [%s] dropped", connection_->getRemoteString().c_str(), topic_.c_str());

    parent->removeSubscriberLink(shared_from_this());
  }
}

void TransportSubscriberLink::onHeaderWritten(const ConnectionPtr& conn)
{
  (void)conn;
  header_written_ = true;
  startMessageWrite(true);
}

// Changed by Johnson
// 
// 2020.02.16 11:31
// 设置用户回调函数
void TransportSubscriberLink::setUserCb(boost::function<void()> callback) {
  user_callback_ = callback;
}
// end

// Changed by Johnson
// 
// 2020.02.16 11:44
// 设置用户回调函数
/*void TransportSubscriberLink::setUserCb(boost::function<void()> remove_callback, boost::function<int()> full_callback) {
  user_remove_callback_ = remove_callback;
  user_full_callback_ = full_callback;
}*/
// end

void TransportSubscriberLink::onMessageWritten(const ConnectionPtr& conn)
{
  (void)conn;

  // Changed by Johnson
  // 
  // 2020.02.13 15:22
  // 检测用户回调函数
  if (!user_callback_.empty()) {
    std::cout << "You have called user_callback." << std::endl;
    user_callback_();
  }
  // end

  writing_message_ = false;
  startMessageWrite(true);
}

void TransportSubscriberLink::startMessageWrite(bool immediate_write)
{
  boost::shared_array<uint8_t> dummy;
  SerializedMessage m(dummy, (uint32_t)0);

  {
    boost::mutex::scoped_lock lock(outbox_mutex_);
    if (writing_message_ || !header_written_)
    {
      return;
    }

    if (!outbox_.empty())
    {
      writing_message_ = true;
      m = outbox_.front();
      outbox_.pop();
      // Changed by Johnson
      // 
      // 2020.02.08 15:22
      // 与待取数据同时移除队列
      // outbox_like_.erase(outbox_like_.begin());
      // end 

      // Changed by Johnson
      // 
      // 2020.02.16 11:25
      // 回调函数，控制外界queue移除元素
      // if (!user_remove_callback_.empty()) 
      //   user_remove_callback_();
      // end
    }
  }

  if (m.num_bytes > 0)
  {
    connection_->write(m.buf, m.num_bytes, boost::bind(&TransportSubscriberLink::onMessageWritten, this, _1), immediate_write);
  }
}

// Changed by Johnson
// 
// 2020.02.07 17:04
// 并行程序随机数种子初始化,微秒级
void InitRand()
{
  struct timeval tv;
  struct timezone tz;
  gettimeofday(&tv, &tz);

  srand((unsigned int)(tv.tv_sec*1000000 +tv.tv_usec));
}
// end 

// Changed by Johnson
// 
// 2020.02.08 14:30
// 用于比对图像内容的接口
double compareImages(boost::shared_array<uint8_t> image_left, boost::shared_array<uint8_t> image_right) {
  // 暂时用随机数代替
  InitRand();

  return rand() % 1000 / 1000.0;
}
// end change

void TransportSubscriberLink::enqueueMessage(const SerializedMessage& m, bool ser, bool nocopy)
{
  (void)nocopy;
  if (!ser)
  {
    return;
  }

  {
    boost::mutex::scoped_lock lock(outbox_mutex_);

    int max_queue = 0;
    if (PublicationPtr parent = parent_.lock())
    {
      max_queue = parent->getMaxQueue();
    }

    ROS_DEBUG_NAMED("superdebug", "TransportSubscriberLink on topic [%s] to caller [%s], queueing message (queue size [%d])", topic_.c_str(), destination_caller_id_.c_str(), (int)outbox_.size());

    if (max_queue > 0 && (int)outbox_.size() >= max_queue)
    {
      if (!queue_full_)
      {
        ROS_DEBUG("Outgoing queue full for topic [%s].  "
               "Discarding oldest message\n",
               topic_.c_str());
      }

      // Changed by Johnson
      // 
      // 2020.02.16 11:35
      // 通过外部queue返回删除元素来达到腾空间的目的
      /*if (!user_remove_callback_.empty() && !user_full_callback_.empty()) {
        // 返回外部对比的结果
        int max_idx = user_full_callback_();

        std::cout << max_idx << std::endl;

        int o_size = outbox_.size();
        std::queue<SerializedMessage> temp_queue;
        outbox_.swap(temp_queue);

        for (int i=0;i<o_size;i++) {
          if (i != max_idx) 
            outbox_.push(temp_queue.front());
          temp_queue.pop();
        }
      } 
      else 
        outbox_.pop();*/
      // end

      // Changed by Johnson
      // 
      // 2020.02.08 14:43
      // 删除左右两侧图像对比相似度最高的一个，同时更新值,O(n)
      /*if (max_queue < 3) outbox_.pop();
      else {
        int o_size = outbox_.size();

        // 找到要删除的索引
        double max_val = -1.0;
        int max_idx = 0;
        for (int i=0;i<o_size;i++) {
          if (outbox_like_[i] > max_val) {
            max_val = outbox_like_[i];
            max_idx = i;
          }
        }
        
        std::cout << max_idx << std::endl;
        // 删除对应数据
        outbox_like_.erase(outbox_like_.begin() + max_idx);
        std::queue<SerializedMessage> temp_queue;
        outbox_.swap(temp_queue);

        for (int i=0;i<o_size;i++) {
          if (i != max_idx) 
            outbox_.push(temp_queue.front());
          temp_queue.pop();
        }
      }*/
      // end 

      // Changed by Johnson
      // 
      // 2020.02.06 15:33
      // 添加random drop代码,O(n)
      /*int o_size = outbox_.size();
      std::queue<SerializedMessage> temp_queue;
      outbox_.swap(temp_queue);

      InitRand();
      // srand((unsigned int)time(NULL));
      
      int rand_idx = rand() % o_size;
      std::cout << rand_idx << std::endl;
      for (int i=0;i<o_size;i++) {
        if (i != rand_idx)
          outbox_.push(temp_queue.front());
        temp_queue.pop();
      }*/
      // end change

      // 原代码
      outbox_.pop(); // toss out the oldest thing in the queue to make room for us
      queue_full_ = true;
    }
    else
    {
      queue_full_ = false;
    }

    // Changed by Johnson
    // 
    // 2020.02.08 14:36
    // 在新添加一个消息的时候，更新当前队尾元素的outbox_like_值
    /*int o_size = outbox_like_.size();
    if (o_size > 1) {
      outbox_like_[o_size - 1] = compareImages(outbox_.back().buf, m.buf);
    }
    
    outbox_like_.push_back(0.0);*/
    // end change
    outbox_.push(m);
  }

  // Changed by Johnson
  // 
  // 2020.02.06 15:36
  // 输出outbox_ 
  std::cout << "ROS DEBUG: " << std::endl;

  std::queue<SerializedMessage> temp_queue = outbox_;
  // std::cout << temp_queue.size() << std::endl;
  while (!temp_queue.empty()) {
    // 输出seq，不过是按照开始订阅时pushlish进行自增的
    // uint32_t* p_stamp = (uint32_t*)(temp_queue.front().message_start) + 1;
    // ros::Time* p_frameId = (ros::Time*)p_stamp + 1;
    // std::cout << *((ros::Time*)p_stamp) << std::endl;
    // std::cout << p_frameId << std::endl;
    std::cout << *((uint32_t*)(temp_queue.front().message_start)) << " ";
    // std::cout << temp_queue.front().num_bytes << " ";
    temp_queue.pop();
  }
  std::cout << std::endl;
  // end

  // Changed by Johnson
  // 
  // 2020.02.08 14:56
  // 输出outbox_like_
  /*int o_size = outbox_like_.size();
  for (int i=0;i<o_size;i++) {
    std::cout << outbox_like_[i] << " ";
  }
  std::cout << std::endl;*/
  // end change

  // 当调用startMessageWrite，才是真正的将Message发出（比如通过socket）
  // 所以outbox_其实是发送端的缓存
  startMessageWrite(false);

  stats_.messages_sent_++;
  stats_.bytes_sent_ += m.num_bytes;
  stats_.message_data_sent_ += m.num_bytes;
}

std::string TransportSubscriberLink::getTransportType()
{
  return connection_->getTransport()->getType();
}

std::string TransportSubscriberLink::getTransportInfo()
{
  return connection_->getTransport()->getTransportInfo();
}

void TransportSubscriberLink::drop()
{
  // Only drop the connection if it's not already sending a header error
  // If it is, it will automatically drop itself
  if (connection_->isSendingHeaderError())
  {
    connection_->removeDropListener(dropped_conn_);
  }
  else
  {
    connection_->drop(Connection::Destructing);
  }
}

} // namespace ros
