#!/bin/bash

cd build
make install
cd ../install
sudo cp -r include/* $TARGET_INCLUDE_DIR
sudo cp -r lib/pkgconfig/* $TARGET_LIB_DIR/pkgconfig/
sudo cp -r lib/theora_image_transport/ lib/lib* $TARGET_LIB_DIR
sudo cp -r lib/python2.7/dist-packages/* $TARGET_LIB_DIR/python2.7/dist-packages/
sudo cp -r share/common-lisp/ros/theora_image_transport/msg/* $TARGET_SHARE_DIR/common-lisp/ros/theora_image_transport/msg/
sudo cp -r share/compressed_depth_image_transport/ share/compressed_image_transport/ $TARGET_SHARE_DIR
sudo cp -r share/gennodejs/ros/theora_image_transport/ $TARGET_SHARE_DIR/gennodejs/ros/
sudo cp -r share/image_transport_plugins/ $TARGET_SHARE_DIR
sudo cp -r share/roseus/ros/theora_image_transport $TARGET_SHARE_DIR/roseus/ros/
sudo cp -r share/theora_image_transport/ $TARGET_SHARE_DIR
cd ..
echo "INSTALL SUCCESS!"