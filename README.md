# NeuPeak_Challenge

## Row-Centering Algorithm

### Objective:
Create an algorithm that computes the angular rate required for a robot to stay centered in a row based on a scanned point cloud.

### Input:
- `row_pointcloud`: Scanned point cloud representing the row (3D numpy array) from the depth camera.

### Output:
- `angular_rate`: Angular rate in degrees per second that the robot should turn. Positive values mean turning right, and negative values mean turning left. A rate of 0 indicates the robot is centered in the row.

### Algorithm Description:
To determine the correction angle for the robot's movement, I employ the concept that the angle of correction for the robot is equivalent to the angle of correction of the walls but in the opposite direction. For instance, if the robot is steered 10 degrees to the left, it is analogous to saying the wall is turned 10 degrees to the right. In the most ideal scenario, both walls will be perfectly parallel to each other.

Moreover, considering the substantial computational load posed by around 90,000 data points in 3D, I questioned whether all these points are necessary for calculating the angle. Recognizing that a top-down view in the X-Y axis plane is adequate for angle determination, I removed the height axis, resulting in 50,000 points after eliminating duplicates.

To streamline the computation, I sorted the data by decreasing values on the Y-axis. This sorting allowed me to initiate operations from the farthest point from the camera to the closest.

Utilizing efficient techniques, I identified points on both walls without imposing a heavy computational burden. By finding the X-intercept and Y-intercept of the lines forming the walls, I could calculate the angles at which they are turned using the arctangent function.

For a clearer understanding, refer to the image below.

The point cloud was segmented, and the furthest left and furthest right points in each segment were determined. These points on the wall were then utilized to calculate slopes and intercepts. 

See the image below for a visual representation.

To enhance accuracy, I computed the average of the angles obtained from both walls.

## Task 2: ROS Node Implementation

### Objective:
Implement a ROS Node that publishes the correction angular rate (`cmd_vel`) for the robot.

### Implementation Details:
- The ROS Node should output the correction angular rate to a `twist` ROS topic.
- Use the `cmd_vel` message type to publish angular rates.

### Pseudocode:
```python
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def row_centering_node():
    # ROS Node Initialization
    rospy.init_node('row_centering_node', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz

    # Publisher Setup
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    while not rospy.is_shutdown():
        # Calculate angular rate using the row-centering algorithm from Task 1
        angular_rate = generate_angular_rate(row_pointcloud)

        # Publish the correction angular rate
        twist_msg = Twist()
        twist_msg.angular.z = angular_rate
        cmd_vel_pub.publish(twist_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        row_centering_node()
    except rospy.ROSInterruptException:
        pass
```

## Task 3: Unit Test Script

### Objective:
Write a unit test script using the standard Python `unittest` library to validate the row-centering algorithm.

### Test Cases:
- Test the algorithm with different inputs, including edge cases.
- Ensure that the generated angular rates meet the expected results.

### Example Pseudocode:
```python
import unittest

class TestRowCenteringAlgorithm(unittest.TestCase):
    def test_centered_row(self):
        # Test when the row is perfectly centered
        # Assert that the generated angular rate is 0

    def test_off_center_row(self):
        # Test when the row is off-center
        # Assert that the generated angular rate is non-zero

    # Add more test cases as needed

if __name__ == '__main__':
    unittest.main()
```

## Optional Bonus 1: `end_of_row` Parameter

### Objective:
Implement an optional Boolean parameter, `end_of_row`, that is `True` if the end of the row is detected.

### Implementation:
Modify the row-centering algorithm to consider the `end_of_row` parameter and adjust the angular rate accordingly.

### Pseudocode:
```python
def generate_angular_rate(row_pointcloud, end_of_row=False):
    # Implementation logic goes here
    if end_of_row:
        # Adjust angular rate for end of row
    return angular_rate
```

Feel free to implement and test this optional feature if time permits.
