#!/bin/bash

cd build
make install
cd ../install
sudo cp -r include/* $TARGET_INCLUDE_DIR
sudo cp -r lib/camera_calibration_parsers/ lib/image_transport/ lib/pkgconfig/ lib/polled_camera/ lib/*.so $TARGET_LIB_DIR
sudo cp -r lib/python2.7/dist-packages/* $TARGET_LIB_DIR/python2.7/dist-packages/
sudo cp -r share/* $TARGET_SHARE_DIR
cd ..
echo "INSTALL SUCCESS!"