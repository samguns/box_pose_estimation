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
#include <pcl/features/gasd.h>

#include "png2pcd.h"

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::visualization;


const struct cam_intrinsics cam_8022_intrin {
  .fx     = 923.121216,
  .fy     = 922.401917,
  .ppx    = 639.088623,
  .ppy    = 374.246979,
  .width  = 1280,
  .height = 720
};

const struct cam_intrinsics cam_8213_intrin {
  .fx     = 931.251526,
  .fy     = 931.601929,
  .ppx    = 636.607727,
  .ppy    = 367.57843,
  .width  = 1280,
  .height = 720
};


void point_picking_cb(const visualization::PointPickingEvent& event)
{
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

  transformPointCloud(cloud_in, cloud_out, cam_extrinsics);
}

void rigid_transform_8213 (PointCloud<PointXYZRGB>& cloud_in, PointCloud<PointXYZRGB>& cloud_out)
{
  Eigen::Matrix4f cam_extrinsics;
  cam_extrinsics << 0.029831420704392507, -0.9367804309474376, -0.34864381614004075, 0.9571586947475353,
      0.03336628924906919, -0.34767139562274835, 0.937022567181537, -0.5429056723360132,
      -0.9989978864244434, -0.039585664825330144, 0.020885355152154568, 0.39505420278238745,
      0, 0, 0, 1;

  transformPointCloud(cloud_in, cloud_out, cam_extrinsics);
}

boost::shared_ptr<PCLVisualizer> visualize_box(PointCloud<PointXYZRGB>::Ptr box_cloud_ptr,
    Eigen::Matrix4f& inv_trans)
{
  PointCloud<PointXYZRGB> cust_axis;
  cust_axis.is_dense = false;
  cust_axis.resize(300);
  for (int i = 0; i < 100; i++) {
    PointXYZRGB axis_x;
    axis_x.x = 0.001 * i;
    axis_x.r = 255;
    cust_axis.points.push_back(axis_x);
  }
  for (int i = 0; i < 100; i++) {
    PointXYZRGB axis_y;
    axis_y.y = 0.001 * i;
    axis_y.g = 255;
    cust_axis.points.push_back(axis_y);
  }
  for (int i = 0; i < 100; i++) {
    PointXYZRGB axis_z;
    axis_z.z = 0.001 * i;
    axis_z.b = 255;
    cust_axis.points.push_back(axis_z);
  }
  transformPointCloud(cust_axis, cust_axis, inv_trans);

  *box_cloud_ptr += cust_axis;

  boost::shared_ptr<PCLVisualizer> viewer (new PCLVisualizer ("3D Viewer"));
  viewer->addPointCloud(box_cloud_ptr, "box");
  viewer->registerPointPickingCallback(point_picking_cb);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters();

  return viewer;
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

  /**
   * Rigid homogeneous transform camera coordinate back to
   * world coordinate.
   */
  rigid_transform_8022(*cloud_8022_ptr, *cloud_8022_ptr);
  rigid_transform_8213(*cloud_8213_ptr, *cloud_8213_ptr);

  /**
   * Concatenate two half-box
   */
  PointCloud<PointXYZRGB>::Ptr box_cloud_ptr (new PointCloud<PointXYZRGB>);
  *box_cloud_ptr = *cloud_8022_ptr;
  *box_cloud_ptr += *cloud_8213_ptr;

  /**
   * Using PassThrough Filter to get box
   */
  PassThrough<PointXYZRGB> pass;

  pass.setInputCloud(box_cloud_ptr);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(0.6f, 0.9f);
  pass.filter(*box_cloud_ptr);

  pass.setInputCloud(box_cloud_ptr);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-0.15f, 0.15f);
  pass.filter(*box_cloud_ptr);

  pass.setInputCloud(box_cloud_ptr);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.3f, 0.613f);
  pass.filter(*box_cloud_ptr);

  /**
   * Using Globally Aligned Spatial Distribution (GASD) descriptors
   * to estimate box pose
   */
  GASDColorEstimation<PointXYZRGB, pcl::GASDSignature984> gasd;
  gasd.setInputCloud(box_cloud_ptr);
  PointCloud<pcl::GASDSignature984> descriptor;
  gasd.compute(descriptor);
  Eigen::Matrix4f transformation = gasd.getTransform();
  print_info ("Alignment transform matrix:\n");
  print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
  print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
  print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
  print_info ("\n");
  print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
  print_info ("\n");

  /**
   * GASD aligned box shape to our world frame, thus the
   * inverse translation (-R_T * t) of above alignment matrix
   * is where the box center located.
   */
  Eigen::Matrix3f R_T;
  R_T = transformation.topLeftCorner(3, 3).transpose();
  Eigen::Vector3f t;
  t = transformation.topRightCorner(3, 1);
  Eigen::Matrix4f inv_trans;
  inv_trans << R_T, -R_T * t,
              Eigen::MatrixXf::Zero(1, 3), 1;
  print_info("Box center is at x:%f y:%f z:%f\n",
            inv_trans(0, 3), inv_trans(1, 3), inv_trans(2, 3));

  boost::shared_ptr<PCLVisualizer> viewer;
  viewer = visualize_box(box_cloud_ptr, inv_trans);
  viewer->spin();

  return 0;
}