### To estimate box pose from PointClouds captured by two opposite located RGB-D cameras

[//]: # (Image References)

[image1]: ./images/off_box.png

---
#### My thoughts in pursuing this task
1. Convert the aligned color & depth images to PointCloud for each RGB-D camera according to the given cameras' intrinsics.
2. Then I used PCL PassThrough filter to get the corresponding box points.
3. By now I have two half-box PointClouds in their relative camera frame. Since each cameras' extrinsics is provided, I applied a homogeneous transform to convert each PointCloud from camera frame to world frame.
4. Concatenate these two half-box PointClouds, I get a 360-degree view of the the whole box with respect to the world frame.
5. Compute centroid to get the center of the box.
6. Using PCL Visualizer to show the box and `shift + mouse left click` to output point coordinate in console.

#### What I've accomplished and the issues
I implemented all steps above, however, the concatenated PointCloud shows these two-half boxes is not *match* as it should be. There's some offset especially in the Z-Axis. I can't really figure it out by now, any thoughts on this?

![alt text][image1]

#### How to compile and run?
1. `make build && cd build`
2. `cmake .. && make`
3. `./box_pose_estimation -1_802212060971_color.png -1_802212060971_aligned_depth.png -1_821312062288_color.png -1_821312062288_aligned_depth.png`

#### Running environment
MacOS Mojave + Homebrew installed Point Cloud Library (PCL).