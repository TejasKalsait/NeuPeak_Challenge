import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import os
import argparse
import matplotlib.pyplot as plt

'''
    Test Description:

    [TASK 1]
    Create an algorithm that takes in the following:
        row_pointcloud : scanned pointcloud (3D numpy array) for the row from depth camera

    And outputs the following:
        angular_rate : rate in deg/s the robot should turn at such that it will centre itself in the row
	
	The goal is to genereate a target angular rate that the robot uses to turn. Positive rate means robot
	will turn right, negative rate means robot will turn left. Assume the robot moves forward at some constant velocity
	set by the user. Your algorithm then generates angular rates for the robot while it moves through the row. The ideal angular 
	rate will make it so that the robot is always centred in the row. When the robot is perfectly centred the angular rate should be 0
	since we just need the robot to go straight.

    [TASK 2]
    Implement a ROS Node that will output correction anglular rate for the robot. Publish cmd_vel to a twist
	ROS topic

    [TASK 3]
    unit-test: Write a unit test script using standard Python unittest library

    [OPTIONAL BONUS 1]
    end_of_row: Boolean parameter that is True if end of row is detected

    


    Evaluation Criteria: Method runtime. Lower is better.
'''


class Angle_Publisher(Node):

    def __init__(self, filename, debug, num_points):
        super().__init__('angle_publisher')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.filename = filename
        self.isdebug = debug
        self.num_of_points = num_points

    def preprocessPoints(self, row_pointcloud):

        # Deleting the height information as my algorithm doesnt need it
        row_pointcloud = np.delete(row_pointcloud, 1, axis = 1)

        # Remove duplicate rows 
        row_pointcloud = np.unique(row_pointcloud, axis = 0)

        # Sorting from furthest to closesnt to the camera
        sorted_indices = np.argsort(row_pointcloud[:, 1])[::-1]
        row_pointcloud = row_pointcloud[sorted_indices]

        if self.isdebug:
            print("After deleting height-axis, removing duplicates, and sorting with x-values, shape is", row_pointcloud.shape)

        return row_pointcloud
    
    def extractEdgePoints(self, row_pointcloud):

        # Lists -> [[x, y]] to store the co-ordinates of points on the wall
        left_points, right_points = [], []

        # Diving the Y-direction into segments
        max_on_y = row_pointcloud[0,1]
        partitions = np.linspace(max_on_y, 0.0, self.num_of_points)

        if self.isdebug:
            print("Starting to extract Points on the Walls")

        
        # This loop extracts the points on the walls. To visualize these points, switch debug_flag = True in the driver code
        for i in range(1, len(partitions)):
            leftmost, rightmost = [float('inf'), -1], [float('-inf'), -1]

            for x, y in row_pointcloud:

                # Base Case 1 (Partition already processed)
                if y > partitions[i - 1]:
                    continue

                # Base case 2 (End of partition)
                if y < partitions[i]:
                    # Add these to the lists as the leftmost and rightmost points extracted in the particular partition
                    left_points.append(leftmost)
                    right_points.append(rightmost)
                    leftmost, rightmost = [float('inf'), -1], [float('-inf'), -1]
                    break

                # Leftmost point in the segment
                if x < leftmost[0]:
                    # found new leftmost
                    leftmost = [x, y]

                # Rightmost point in the segment
                if x > rightmost[0]:
                    # found new rightmost
                    rightmost = [x, y]

        left_points = np.array(left_points)
        right_points = np.array(right_points)

        # Visualize the points on walls
        if self.isdebug:
            # Plotting in x-y plane
            fig = plt.figure()
            ax = fig.add_subplot()
            ax.scatter(row_pointcloud[:, 0], row_pointcloud[:, 1], s = 0.1)

            ax.scatter(left_points[:, 0], left_points[:, 1], c = 'r')
            ax.scatter(right_points[:, 0], right_points[:, 1], c = 'r')

            plt.show()

        return left_points, right_points
    
    def calculateInterceptfromPoints(self, left_points, right_points):

        # Extracting two points from the left wall
        lx1, ly1 = left_points[0]
        lx2, ly2 = left_points[1]

        # # Extracting two points from the right wall
        rx1, ry1 = right_points[0]
        rx2, ry2 = right_points[1]

        # If divided by zero, that means line has slope infinity and no turn is required (return 0 degrees)
        try:
            l_slope = (ly2 - ly1) / (lx2 - lx1)
        except:
            print("Found the left wall fully straight")
            return None, None, None, None
        try:
            r_slope = (ry2 - ry1) / (rx2 - rx1)
        except:
            print("Found the right wall fully straight")
            return None, None, None, None
        
        if self.isdebug:
            print("Left wall slope is", l_slope, "and right wall slope is", r_slope)

        # X and Y intercept of left wall line
        l_y_intercept = -(l_slope * lx2) + ly1
        l_x_intercept = lx1 - (ly1 / l_slope)

        # X and Y intercept of right wall line
        r_y_intercept = -(r_slope * rx2) + ry1
        r_x_intercept = rx1 - (ry1 / r_slope)

        return (abs(l_x_intercept), abs(l_y_intercept), l_slope, abs(r_x_intercept), abs(r_y_intercept), r_slope)
    
    def calculateAnglefromIntercepts(self, left_x_intercept, left_y_intercept, left_slope, right_x_intercept, right_y_intercept, right_slope):

        # Line was found with infinite slope, therefore return 0 degree turn
        if left_x_intercept == None or right_x_intercept == None:
            return 0.0

        # using tan-inverse and x and y intercept to find the angle by which the wall has turned
        # minus sign because right turn = positive angle and left turn = negative angle. We can determine which side to rotate based on the walls slope
        
        # turn angle based on left wall
        if left_slope >= 0.0:
            left_angle_turn = 90 - np.degrees(np.arctan(left_y_intercept/left_x_intercept))
        else:
            left_angle_turn = -(90 - np.degrees(np.arctan(left_y_intercept/left_x_intercept)))

        # turn angle based on right wall
        if right_slope >= 0:
            right_angle_turn = 90 - np.degrees(np.arctan(right_y_intercept/right_x_intercept))
        else:
            right_angle_turn = -(90 - np.degrees(np.arctan(right_y_intercept/right_x_intercept)))

        # returning the average turn. (Also helps to keep the robot straight when it is the end of the row. Using the point clouds of the surface if no walls found)
        return (left_angle_turn + right_angle_turn) / 2.0

    def publishTwistAngle(self, angle, publisher):

        # new twist message to publish the angle
        twist_msg = Twist()
        twist_msg.angular.z = angle

        publisher.publish(twist_msg)

    def anglePublishOnce(self):

        # Open the .npz file with the given file name
        try:
            # Opening file
            row_np_array = np.load("../Depth_Output/" + self.filename)
            row_pointcloud = row_np_array['arr_0']
        except Exception as e:
            print(f"Error opening the .npz file {e}")

        # Function that takes a pointcloud, deletes height axis, removes duplicates, and sorts from furthest to nearest along the row
        row_pointcloud = self.preprocessPoints(row_pointcloud)

        # Extracts #self.num_of_points - 2' points on each wall
        left_edge_points, right_edge_points = self.extractEdgePoints(row_pointcloud, plot = False)

        # Returns the slopes and intercepts of the two walls which is used to calculate the angle
        left_x_intercept, left_y_intercept, left_slope, right_x_intercept, right_y_intercept, right_slope = self.calculateInterceptfromPoints(left_edge_points, right_edge_points)
        
        # Returns the angle to turn by calculating the arctan
        angle_to_turn = self.calculateAnglefromIntercepts(left_x_intercept, left_y_intercept, left_slope, right_x_intercept, right_y_intercept, right_slope)
        
        print("The robot should turn", angle_to_turn, "degrees")

        # Publishes the Twist message on the topic cmd_vel
        self.publishTwistAngle(angle_to_turn, self.publisher)




# _______________________________________________________________________________________________________________________________________________

# DRIVER CODE
def main():
    rclpy.init()

    filename = "1.npz"
    # If True, Prints information and displays plots
    debug_flag = False
    # Refer to the README.md or (image name)
    # The number 4 works perfect for whichever depth camera the bot is using in the example
    num_of_points_to_extract = 4

    # We can use the rclpy spin functionality to keep the node alive and keep publishing to the topic at some frequency
    # but since we need to publish only once here, I am calling the function only once without spin.
    
    angle_publisher = Angle_Publisher(filename, debug_flag, num_of_points_to_extract)
    angle_publisher.anglePublishOnce()

    angle_publisher.destroy_node()
    rclpy.shutdown()
# _______________________________________________________________________________________________________________________________________________

if __name__ == '__main__':
    main()