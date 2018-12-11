#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

#include "png2pcd.h"

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::visualization;


const struct cam_intrinsics cam_8022_intrin {
  923.121216,
  922.401917,
  639.088623,
  374.246979,
  1280,
  720
};

const struct cam_intrinsics cam_8213_intrin {
  931.251526,
  931.601929,
  636.607727,
  367.57843,
  1280,
  720
};


void point_picking_cb(const visualization::PointPickingEvent& event) {
  if (event.getPointIndex() != -1) {
    float x, y, z;
    event.getPoint(x, y, z);
    pcl::console::print_info("Selected point x:%f y:%f z:%f\n", x, y, z);
  }
}

void rigid_transform_8022 (PointCloud<PointXYZRGB>& cloud_in, PointCloud<PointXYZRGB>& cloud_out)
{
  Eigen::Matrix4f cam_extrinsics;
  cam_extrinsics << -0.8885602320468353, 0.07700865227582554, 0.4522503526803875, 0.5490860440797882,
      -0.4582141975635262, -0.1009064094455465, -0.8830954906939686, 0.5348574571331621,
      -0.0223710343097415, -0.9919110665818123, 0.12494788039996335, 0.38599111429878424,
      0, 0, 0, 1;

  Eigen::Matrix3f R_T;
  R_T << cam_extrinsics.topLeftCorner(3, 3).transpose();
  Eigen::Vector3f t;
  t << cam_extrinsics.topRightCorner(3, 1);

  Eigen::Matrix4f homo_trans;
  homo_trans << R_T, -(R_T * t),
	  Eigen::MatrixXf::Zero(1, 3), 1;

  std::cout << "R_T " << R_T << std::endl;
  std::cout << "t " << t << std::endl;
  std::cout << "homo_trans " << homo_trans << std::endl;

  transformPointCloud(cloud_in, cloud_out, cam_extrinsics);
}

void rigid_transform_8213 (PointCloud<PointXYZRGB>& cloud_in, PointCloud<PointXYZRGB>& cloud_out)
{
  Eigen::Matrix4f cam_extrinsics;
  cam_extrinsics << 0.029831420704392507, -0.9367804309474376, -0.34864381614004075, 0.9571586947475353,
      0.03336628924906919, -0.34767139562274835, 0.937022567181537, -0.5429056723360132,
      -0.9989978864244434, -0.039585664825330144, 0.020885355152154568, 0.39505420278238745,
      0, 0, 0, 1;

  Eigen::Matrix3f R_T;
  R_T << cam_extrinsics.topLeftCorner(3, 3).transpose();
  Eigen::Vector3f t;
  t << cam_extrinsics.topRightCorner(3, 1);

  Eigen::Matrix4f homo_trans;
  homo_trans << R_T, -(R_T * t),
	  Eigen::MatrixXf::Zero(1, 3), 1;

  transformPointCloud(cloud_in, cloud_out, cam_extrinsics);
}

int main(int argc, char** argv) {
  std::vector<int> png_file_indices =
      parse_file_extension_argument (argc, argv, ".png");

  if (png_file_indices.size () != 4) {
    print_error ("Need four input PNG files.\n");
    return -1;
  }

  PointCloud<PointXYZRGB>::Ptr cloud_8022_ptr (new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr cloud_8213_ptr (new PointCloud<PointXYZRGB>);
  png2pcd converter;
  int result = converter.generate_pointclouds(png_file_indices[0], argv,
                                              &cam_8022_intrin,
                                              *cloud_8022_ptr);
  if (result < 0) {
    print_error("Failed to generate PointCloud.\n");
    return -1;
  }

  result = converter.generate_pointclouds(png_file_indices[2], argv,
                                              &cam_8213_intrin,
                                              *cloud_8213_ptr);
  if (result < 0) {
    print_error("Failed to generate PointCloud.\n");
    return -1;
  }
#if 0
  /**
   * Using PassThrough Filter to get box
   */
  PassThrough<PointXYZRGB> pass;
  pass.setInputCloud(cloud_8022_ptr);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(0, 0.15f);
  pass.filter(*cloud_8022_ptr);

  pass.setInputCloud(cloud_8022_ptr);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.4f, 0.7f);
  pass.filter(*cloud_8022_ptr);

  pass.setInputCloud(cloud_8213_ptr);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(0, 0.2f);
  pass.filter(*cloud_8213_ptr);

  pass.setInputCloud(cloud_8213_ptr);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.4f, 0.6f);
  pass.filter(*cloud_8213_ptr);
#endif
  /**
   * Rigid homogeneous transform camera coordinate to
   * world coordinate.
   */
  rigid_transform_8022(*cloud_8022_ptr, *cloud_8022_ptr);
  rigid_transform_8213(*cloud_8213_ptr, *cloud_8213_ptr);

  /**
   * Concatenate two halves and compute centroid of the box
   */
  PointCloud<PointXYZRGB>::Ptr box_cloud_ptr (new PointCloud<PointXYZRGB>);
  *box_cloud_ptr = *cloud_8022_ptr;
  *box_cloud_ptr += *cloud_8213_ptr;

  Eigen::Vector4f centroid;
  compute3DCentroid(*box_cloud_ptr, centroid);
  PointXYZRGB center_p;
  center_p.r = 255;
  center_p.g = 0;
  center_p.b = 0;
  center_p.x = centroid(0);
  center_p.y = centroid(1);
  center_p.z = centroid(2);
  box_cloud_ptr->points.push_back(center_p);
  pcl::console::print_info("Centroid x:%f y:%f z:%f\n",
      centroid(0), centroid(1), centroid(2));

  visualization::PCLVisualizer viewer("3D viewer");
  viewer.addPointCloud(box_cloud_ptr, "box");
  viewer.registerPointPickingCallback(point_picking_cb);
  viewer.addCoordinateSystem (1.0);
  viewer.initCameraParameters();
  viewer.spin();

  return 0;
}