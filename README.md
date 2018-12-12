### To estimate box pose from PointClouds captured by two opposite located RGB-D cameras

[//]: # (Image References)

[image1]: ./images/active_vision_0.png
[image2]: ./images/active_vision_237.png
[image3]: ./images/active_vision_276.png
[image4]: ./images/active_vision_457.png
[image5]: ./images/active_vision_488.png
[image6]: ./images/active_vision_772.png
[image7]: ./images/active_vision_920.png
[image8]: ./images/active_vision_1007.png
[image9]: ./images/active_vision_1502.png
[image10]: ./images/active_vision_2035.png

---
#### My thoughts in pursuing this task
1. Convert the aligned color & depth images to PointCloud for each RGB-D camera according to the given cameras' intrinsics.
2. By now I have two half-box PointClouds in their relative camera frame. Since each cameras' extrinsics is provided, I applied a homogeneous transform to convert each PointCloud from camera frame to world frame.
3. Then I used PCL PassThrough filter to get the corresponding box points.
4. Concatenate these two filtered half-box PointClouds, I get a 360-degree view of the the whole box with respect to world frame.
5. I tried to estimate the box pose as this [GASD tutorial](http://pointclouds.org/documentation/tutorials/gasd_estimation.php#gasd-estimation) suggests. It generated a homogeneous transform matrix that aligns box to the origin of world frame. So the inverse translation of this matrix tells me where the box center is.
6. Using PCL Visualizer to show the box and `shift + mouse left click` to output point coordinate in console.

##### active_vision_0
![alt text][image1]

##### active_vision_237
![alt text][image2]

##### active_vision_276
![alt text][image3]

##### active_vision_457
![alt text][image4]

##### active_vision_488
![alt text][image5]

##### active_vision_772
![alt text][image6]

##### active_vision_920
![alt text][image7]

##### active_vision_1007
![alt text][image8]

##### active_vision_1502
![alt text][image9]

##### active_vision_2035
![alt text][image10]

#### How to compile and run?
1. `make build && cd build`
2. `cmake .. && make`
3. `./box_pose_estimation -1_802212060971_color.png -1_802212060971_aligned_depth.png -1_821312062288_color.png -1_821312062288_aligned_depth.png`

#### Testing environment
MacOS Mojave + Homebrew installed Point Cloud Library (PCL).

Win7 + msvc2017 + PCL-1.9.1-AllInOne-msvc2017-win64.exe
