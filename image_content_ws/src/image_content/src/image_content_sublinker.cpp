#include <image_content_sublinker.h>
#include <image_content_algorithm.h>

#define MIN_COMPARE 0.0

void ImageContentSubLinker::enqueueMessage(sensor_msgs::ImagePtr message) {

    {
        boost::mutex::scoped_lock lock(out_mutex_);

        if (max_queue_ > 0 && (int)outbox_.size() >= max_queue_) {
            
            if (max_queue_ < 3) outbox_.pop_back();
            else {
                int o_size = outbox_.size();

                // 找到要删除的索引
                double max_val = -1.0;
                int max_idx = 0;
                for (int i=0;i<o_size;i++) {
                    if (outbox_[i]->val > max_val) {
                        max_val = outbox_[i]->val;
                        max_idx = i;
                    }
                }
                
                std::cout << max_idx << std::endl;
                // 删除对应数据
                outbox_.erase(outbox_.begin() + max_idx);

                // 更新相似度
                if (max_idx == 0) {
                    if (last_frame_->height != 0) {
                        outbox_[max_idx]->val = CompareImageContent(last_frame_, outbox_[max_idx+1]->image);
                    }
                    else {
                        outbox_[max_idx]->val = MIN_COMPARE;
                    }
                }
                else if (max_idx == 1) {
                    if (last_frame_->height != 0) {
                        outbox_[max_idx]->val = CompareImageContent(outbox_[max_idx-1]->image, outbox_[max_idx+1]->image);
                        outbox_[max_idx-1]->val = CompareImageContent(outbox_[max_idx]->image, last_frame_);
                    }
                    else {
                        outbox_[max_idx]->val = CompareImageContent(outbox_[max_idx-1]->image, outbox_[max_idx+1]->image);
                        outbox_[max_idx-1]->val = MIN_COMPARE;
                    }
                }
                else {
                    outbox_[max_idx]->val = CompareImageContent(outbox_[max_idx-1]->image, outbox_[max_idx+1]->image);
                    outbox_[max_idx-1]->val = CompareImageContent(outbox_[max_idx-2]->image, outbox_[max_idx]->image);
                }
            }
        }

        int o_size = outbox_.size();
        if (o_size > 1) {
            outbox_[o_size - 1]->val = CompareImageContent(outbox_[o_size - 2]->image, message);
        }
        else if (o_size == 1 && last_frame_->height != 0) {
            outbox_[0]->val = CompareImageContent(last_frame_, message);
        }

        outbox_.push_back(new ImageAndContent(message, 0.0));
        last_frame_ = message;

        std::cout << "MY_QUEUE DEBUG: " << std::endl;
        o_size = outbox_.size();
        std::cout << "o_size = " << o_size << std::endl;
        for (int i=0;i<o_size;i++) {
            std::cout << outbox_[i]->image->header.frame_id << " ";
        }
        std::cout << std::endl;

        for (int i=0;i<o_size;i++) {
            std::cout << outbox_[i]->val << " ";
        }
        std::cout << std::endl;
    }

    startWriteMessage();
}

void ImageContentSubLinker::startWriteMessage() {
    {
        boost::mutex::scoped_lock lock(out_mutex_);
        
        if (is_writting_) return;

        if(!outbox_.empty()) {
            is_writting_ = true;
            ImageAndContent* cur = outbox_[0];
            outbox_.erase(outbox_.begin());
            pub_->publish(cur->image);
        }
    }
}

void ImageContentSubLinker::onMessageWritten() {
    is_writting_ = false;
    startWriteMessage();
}

void ImageContentSubLinker::setMaxQueue(int max_queue) {
    max_queue_ = max_queue;
}

void ImageContentSubLinker::setImageTransportPub(image_transport::Publisher* pub) {
    pub_ = pub;
}