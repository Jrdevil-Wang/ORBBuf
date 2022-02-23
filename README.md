# ORBBuf

A Robust Buffering Method for Remote Visual SLAM

# Description

- <strong>/test_pub</strong> is an example of how to use the ORBBuf interface.

- <strong>/image_content_ws</strong> is the main implementation of the ORBBuf interface.

In order to make the ORBBuf interface work, we have to modify underlining ROS code. To be specific:

- <strong>/ros_comm_ws</strong> is the modified [ros_comm](https://github.com/ros/ros_comm) code, based on the Kinetic version.
Main changes are around [here](https://github.com/Jrdevil-Wang/ORBBuf/blob/968ae38b879019ae7f5d7ef3b51fadeb81196779/ros_comm_ws/src/ros_comm/clients/roscpp/include/ros/transport_subscriber_link.h#L106) and [here](https://github.com/Jrdevil-Wang/ORBBuf/blob/968ae38b879019ae7f5d7ef3b51fadeb81196779/ros_comm_ws/src/ros_comm/clients/roscpp/src/libros/transport_subscriber_link.cpp#L184).
These changes allow us to trigger a user-defined callback function whenever a message has been sent.

- <strong>/image_transport_ws</strong> is the modified [image_transport](https://github.com/ros-perception/image_common) code.

- <strong>/plugins_ws</strong> is the modified [image_transport_plugins](https://github.com/ros-perception/image_transport_plugins) code.
These changes allow us to pass the user-defined callback function along the way.

# Usage

1. Run <strong>build.sh</strong>. Note that this process will update your ROS library (libroscpp.so). But if you don't use the ORBBuf interface, there is no harm to your ROS.
2. Run the generated <strong>stereo_kitti_pub</strong> for a test with the KITTI dataset.

# Citation

The full paper can be accessed with [arXiv](https://arxiv.org/abs/2010.14861).

If you are interested, please cite our IROS 2021 paper:

```
@inproceedings{DBLP:conf/iros/WangZWDQM21,
  author    = {Yu{-}Ping Wang and
               Zi{-}Xin Zou and
               Cong Wang and
               Yue{-}Jiang Dong and
               Lei Qiao and
               Dinesh Manocha},
  title     = {ORBBuf: {A} Robust Buffering Method for Remote Visual {SLAM}},
  booktitle = {{IEEE/RSJ} International Conference on Intelligent Robots and Systems,
               {IROS} 2021, Prague, Czech Republic, September 27 - Oct. 1, 2021},
  pages     = {8706--8713},
  publisher = {{IEEE}},
  year      = {2021},
  url       = {https://doi.org/10.1109/IROS51168.2021.9635950},
  doi       = {10.1109/IROS51168.2021.9635950},
  timestamp = {Wed, 22 Dec 2021 12:37:50 +0100},
  biburl    = {https://dblp.org/rec/conf/iros/WangZWDQM21.bib},
  bibsource = {dblp computer science bibliography, https://dblp.org}
}
```
