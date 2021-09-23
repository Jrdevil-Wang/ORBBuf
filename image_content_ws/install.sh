#!/bin/bash

sudo cp src/image_content/include/* $TARGET_INCLUDE_DIR
# sudo cp src/image_content/include/image_content_algorithm.h src/image_content/include/image_content_sublinker.h /opt/ros/kinetic/include/image_content/
sudo cp devel/lib/libimage_content.so $TARGET_LIB_DIR
echo "INSTALL SUCCESS!"
