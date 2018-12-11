#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/vtk_lib_io.h>
#include "png2pcd.h"

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

/**
 * The depth pixel has value ranges from 0-65535,
 * where the scale is 10^-4: pixel value of 1000 means 0.1 meter in depth.
 */
const float depth_unit_magic = 10000.0f;

int png2pcd::generate_pointclouds(int arg_base, char **argv,
                                  const struct cam_intrinsics *intrin,
                                  PointCloud<PointXYZRGB>& rgb_depth_cloud) {
  // Load the color input file
  vtkSmartPointer<vtkImageData> color_image_data;
  vtkSmartPointer<vtkPNGReader> color_reader = vtkSmartPointer<vtkPNGReader>::New ();
  color_reader->SetFileName (argv[arg_base]);
  color_reader->Update ();
  color_image_data = color_reader->GetOutput ();

  int components = color_image_data->GetNumberOfScalarComponents ();
  if (components != 3) {
    print_error ("Component number of RGB input file should be 3.\n");
    return -1;
  }
  int dimensions[3];
  color_image_data->GetDimensions (dimensions);

  // Load the depth input file
  vtkSmartPointer<vtkImageData> depth_image_data;
  vtkSmartPointer<vtkPNGReader> depth_reader;
  depth_reader = vtkSmartPointer<vtkPNGReader>::New ();
  depth_reader->SetFileName (argv[arg_base+1]);
  depth_reader->Update ();
  depth_image_data = depth_reader->GetOutput ();

  if (depth_reader->GetNumberOfScalarComponents () != 1)
  {
    print_error ("Component number of depth input file should be 1.\n");
    return -1;
  }

  int depth_dimensions[3];
  depth_image_data->GetDimensions (depth_dimensions);
  if (depth_dimensions[0] != dimensions[0] || depth_dimensions[1] != dimensions[1])
  {
    print_error ("Width or height of the color and depth input file should be the same.\n");
    return -1;
  }

  // Retrieve the entries from the image data and copy them into the output RGB cloud
  double* pixel = new double [4];
  memset (pixel, 0, sizeof (double) * 4);
  float depth;
  rgb_depth_cloud.width = dimensions[0];
  rgb_depth_cloud.height = dimensions[1];
  rgb_depth_cloud.is_dense = false;
  rgb_depth_cloud.resize (rgb_depth_cloud.width * rgb_depth_cloud.height);
  float h_fov = atan2(intrin->ppx + 0.5f, intrin->fx) +
	  atan2(intrin->width - (intrin->ppx + 0.5f), intrin->fx);
  float v_fov = atan2(intrin->ppy + 0.5f, intrin->fy) +
	  atan2(intrin->height - (intrin->ppy + 0.5f), intrin->fy);
  float width = std::tan(h_fov / 2) * 2;
  float height = std::tan(v_fov / 2) * 2;

  for (int y = 0; y < dimensions[1]; y++)
  {
    for (int x = 0; x < dimensions[0]; x++)
    {
      pixel[0] = color_image_data->GetScalarComponentAsDouble (x, y, 0, 0);
      pixel[1] = color_image_data->GetScalarComponentAsDouble (x, y, 0, 1);
      pixel[2] = color_image_data->GetScalarComponentAsDouble (x, y, 0, 2);
      depth = depth_image_data->GetScalarComponentAsFloat (x, y, 0, 0) / depth_unit_magic;

      PointXYZRGB xyzrgb;
	  float px = (x - intrin->width / 2.0) / intrin->width * width * depth;
	  float py = (intrin->height / 2.0 - y) / intrin->height * height * depth;

      // In our camera frame, X-Axis starts at ppx, pointing left,
      // assign minus sign to px to correct mirroring.
      xyzrgb.x = px;
      xyzrgb.y = py;
      xyzrgb.z = depth;
      xyzrgb.r = static_cast<uint8_t> (pixel[0]);
      xyzrgb.g = static_cast<uint8_t> (pixel[1]);
      xyzrgb.b = static_cast<uint8_t> (pixel[2]);

      rgb_depth_cloud(x, dimensions[1] - y - 1) = xyzrgb;
    }
  }

  delete[] pixel;
  return 0;
}