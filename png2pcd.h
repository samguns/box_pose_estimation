//
// Created by Sam on 2018-12-08.
//

#ifndef BOX_POSE_ESTIMATION_PNG2PCD_H
#define BOX_POSE_ESTIMATION_PNG2PCD_H

#include <pcl/point_cloud.h>

struct cam_intrinsics {
  float fx;
  float fy;
  float ppx;
  float ppy;
};

class png2pcd {
 public:
  png2pcd() {}

  int generate_pointclouds(int arg_base, char **argv,
      const struct cam_intrinsics *intrin,
      pcl::PointCloud<pcl::PointXYZRGB>& rgb_depth_cloud);
};

#endif //BOX_POSE_ESTIMATION_PNG2PCD_H
