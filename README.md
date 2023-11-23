# NeuPeak_Challenge

> **_NOTE:_**  I have followed the standard ROS format for the directory structure. The row_nav.py and test_row_nav.py are located inside the `/src` directory.
> In this case, I am publishing a Twist message to the `/cmd_vel` topic only once. However, we can keep publishing the same by using the loop until the ROS node is active.


## Row Centering Algorithm Description:

- To determine the correction angle for the robot's movement, I employ the concept that the angle of correction for the robot is equivalent to the angle of correction of the walls but in the opposite direction. For instance, if the robot is steered 10 degrees to the left, it is analogous to saying the wall is turned 10 degrees to the right. In the most ideal scenario, both walls will be perfectly parallel to each other.

- Moreover, considering the substantial computational load posed by around 90,000 data points in 3D, I questioned whether all these points are necessary for calculating the angle. Recognizing that a top-down view in the X-Y axis plane is adequate for angle determination, I removed the height axis, resulting in 50,000 points after eliminating duplicates.

- To streamline the computation, I sorted the data by decreasing values on the Y-axis. This sorting allowed me to initiate operations from the farthest point from the camera to the closest.

- Utilizing efficient techniques, I identified points on both walls without imposing a heavy computational burden. By finding the X-intercept and Y-intercept of the lines forming the walls, I could calculate the angles at which they are turned using the arctangent function.

- For a clearer understanding, refer to the image below.

- The point cloud was segmented, and the furthest left and furthest right points in each segment were determined. These points on the wall were then utilized to calculate slopes and intercepts. 

- See the image below for a visual representation.

- To enhance accuracy, I computed the average of the angles obtained from both walls.

## Functions Explained

1. `anglePublishOne` - Starting function that contains the overall pattern of the execution.
2. `preprocessPoints` - Function that takes a point cloud, deletes the height axis, removes duplicates, and sorts from furthest to nearest along the row.
3. `extractEdgePoints` - Extract points on both the wall to find slopes. (Put arg `--debug  True` to visualize)
4. `calculateInterceptfromPoints` - Returns the slopes and intercepts of the two walls which are used to calculate the angle.
5. `calculateAnglefromIntercept` - Returns the angle to turn by calculating the arctan. (right turn -> positive and left turn -> negative)
6. `publishTwistAngle` - Publishes the angle obtained to the topic cmd_vel and returns the angular_rate and end_of_row to the main function.

> **_NOTE:_** Given more time, I'd definitely try to improve the accuracy of the system. Although, even right now it is pretty accurate I believe. If run continuously on the robot at least 10Hz (the algorithm takes less than 0.1 sec to run), it should give pretty accurate results.
> However, I strongly feel that using 2 RGB cameras to calculate depth is much cheaper, much faster to run, and less computationally expensive.
