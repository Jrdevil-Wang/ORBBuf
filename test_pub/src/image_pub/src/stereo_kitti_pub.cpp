//
// Created by zouzx on 2019/10/22.
//

#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/publication.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <image_content_publication.h>
#include <image_content_sublinker.h>

using namespace std;

int main(int argc, char **argv) {
    if (argc != 2) {
        cout << "Usage: ./stereo_kitti_pub [SEQUENCE_PATH]\n\tFor example: ./stereo_kitti_pub /path/to/dataset/sequences/00/" << endl;
        return 1;
    }

    ros::init(argc, argv, "stereo_image_pub");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    ImageContentPublication left_image_publication(it, "/camera/left/image_raw", 20);
    ImageContentPublication right_image_publication(it, "/camera/right/image_raw", 20);

    ros::Publisher left_caminfo_pub = nh.advertise<sensor_msgs::CameraInfo>("/camera/left/camera_info", 20);
    ros::Publisher right_caminfo_pub = nh.advertise<sensor_msgs::CameraInfo>("/camera/right/camera_info", 20);
    nh.setParam("/camera/left/image_raw/compressed/png_level", 5);
    nh.setParam("/camera/right/image_raw/compressed/png_level", 5);
    string sequence = argv[1];
    printf("read sequence: %s\n", argv[1]);
    string dataPath = sequence + "/";
    FILE * file = std::fopen((dataPath + "times.txt").c_str() , "r");
    if (file == NULL) {
        printf("cannot find file: %stimes.txt\n", dataPath.c_str());
        ROS_BREAK();
        return 0;          
    }
    double imageTime;
    vector<double> imageTimeList;
    while (fscanf(file, "%lf", &imageTime) != EOF) {
        imageTimeList.push_back(imageTime);
    }
    std::fclose(file);

    string leftImagePath, rightImagePath;
    cv::Mat imLeft, imRight;
    sensor_msgs::CameraInfo left_caminfo, right_caminfo;

    file = std::fopen((dataPath + "calib.txt").c_str(), "r");
    if (file == NULL) {
        printf("cannot find file: %scalib.txt\n", dataPath.c_str());
        ROS_BREAK();
        return 0;
    } else {
        char str[10];
        fscanf(file, "%s",  str);
        for (int i = 0; i < 12; i++) {
            double param;
            fscanf(file, "%lf", &param);
            left_caminfo.P[i] = param;
        }
        fscanf(file, "%s",  str);
        for (int i = 0; i < 12; i++) {
            double param;
            fscanf(file, "%lf", &param);
            right_caminfo.P[i] = param;
        }
        for (int i = 0; i < 3; i++) left_caminfo.K[i] = left_caminfo.P[i];
        for (int i = 4; i < 7; i++) left_caminfo.K[i - 1] = left_caminfo.P[i];
        for (int i = 8; i < 11; i++) left_caminfo.K[i - 2] = left_caminfo.P[i];

        for (int i = 0; i < 3; i++) right_caminfo.K[i] = right_caminfo.P[i];
        for (int i = 4; i < 7; i++) right_caminfo.K[i - 1] = right_caminfo.P[i];
        for (int i = 8; i < 11; i++) right_caminfo.K[i - 2] = right_caminfo.P[i];
    }
    left_caminfo.height = right_caminfo.height = 376;
    left_caminfo.width =  right_caminfo.height = 1241;

    ros::Rate loop_rate(10);
    for (size_t i = 0; i < imageTimeList.size(); i += 1) {	
        if (ros::ok()) {
            printf("\nprocess image %d\n", (int)i);
            stringstream ss;
            ss << setfill('0') << setw(6) << i;
            leftImagePath = dataPath + "image_0/" + ss.str() + ".png";
            rightImagePath = dataPath + "image_1/" + ss.str() + ".png";

            left_caminfo.header.stamp = ros::Time(imageTimeList[i]);
            right_caminfo.header.stamp = ros::Time(imageTimeList[i]);
            left_caminfo_pub.publish(left_caminfo);
            right_caminfo_pub.publish(right_caminfo);

            imLeft = cv::imread(leftImagePath, CV_LOAD_IMAGE_GRAYSCALE );
            sensor_msgs::ImagePtr imLeftMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imLeft).toImageMsg();
            imLeftMsg->header.stamp = ros::Time(imageTimeList[i]);
            imLeftMsg->header.seq = (uint32_t)i;
            imLeftMsg->header.frame_id = to_string(i);
            left_image_publication.publish(imLeftMsg);
            // left_image_pub.publish(imLeftMsg);

            imRight = cv::imread(rightImagePath, CV_LOAD_IMAGE_GRAYSCALE );
            sensor_msgs::ImagePtr imRightMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imRight).toImageMsg();
            imRightMsg->header.stamp = ros::Time(imageTimeList[i]);
            right_image_publication.publish(imRightMsg);
            // right_image_pub.publish(imRightMsg);

            ros::spinOnce();
            loop_rate.sleep();
            //sleep(4);
        } else
            break;
    }
    return 0;
}
