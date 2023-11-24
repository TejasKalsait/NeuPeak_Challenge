import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import argparse
import matplotlib.pyplot as plt
import time


class Angle_Publisher(Node):

    def __init__(self, filename, debug, num_points):
        super().__init__('angle_publisher')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.filename = filename
        self.isdebug = debug
        self.end_of_row = False
        self.num_of_points = num_points  # Done to create a more generalized algorithm. Two points on both sides works very well but instead of hardcoding, we could change this value. 
        self.wall_height = 0.4           # Works wonderfully well for this depth sensor and wall settings. But if in future we upgrade the sensor, we just have to change this parameter

    def plot_row_pointcloud(self, pointcloud):

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.view_init(azim=0, elev=-180)
        ax.scatter(pointcloud[:, 2], pointcloud[:, 0], pointcloud[:, 1], s = 1)
        print(len(pointcloud), "total points found")
        plt.show()

    def endOfRow(self, row_pointcloud):
        # Changes the variable self.end_of_row to True if it is the end of the road 
        
        # Extract the points that are above a certain threshold height which we consider as a wall
        walls = row_pointcloud[row_pointcloud[:,1] < -self.wall_height]

        # Visualize the walls
        if self.isdebug:
            print("Showing the walls...")
            print(len(walls))
            self.plot_row_pointcloud(walls)

        if len(walls) == 0:
            self.end_of_row = True
    
    def preprocessPoints(self, row_pointcloud):

        # Deleting the height information as my algorithm doesnt need it
        row_pointcloud = np.delete(row_pointcloud, 1, axis = 1)
        
        # Remove duplicate rows 
        row_pointcloud = np.unique(row_pointcloud, axis = 0)

        # Sorting from furthest to closesnt to the camera
        sorted_indices = np.argsort(row_pointcloud[:, 1])[::-1]
        row_pointcloud = row_pointcloud[sorted_indices]

        if self.isdebug:
            print("After deleting height-axis, removing duplicates, extracting walls, and sorting with x-values, shape is", row_pointcloud.shape)

        return row_pointcloud
    
    def extractEdgePoints(self, row_pointcloud):

        # Lists -> [[x, y]] to store the co-ordinates of points on the wall
        left_points, right_points = [], []

        # Diving the Y-direction into segments
        min_on_y = 0.0
        max_on_y = row_pointcloud[0,1]
        partitions = np.linspace(max_on_y, min_on_y, self.num_of_points)

        if self.isdebug:
            print("Starting to extract Points on the Walls")

        
        # Edge case if the row_pointcloud has very few points for some reason 
        if len(row_pointcloud) < (self.num_of_points * 2):
            print("Sensor error! Enough points not found. Total points after preprocessing are", len(row_pointcloud))
            return left_points, right_points
        
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
            ax.scatter(row_pointcloud[0,0], max_on_y, c = 'g')
            ax.scatter(row_pointcloud[-1,0], min_on_y, c = 'g')
            ax.scatter(left_points[:, 0], left_points[:, 1], c = 'r')
            ax.scatter(right_points[:, 0], right_points[:, 1], c = 'r')

            plt.show()

        return left_points, right_points
    
    def calculateInterceptfromPoints(self, left_points, right_points):

        # Error handeling! If no points are extracted or points are less than 2 on both sides
        if len(left_points) < 2 or len(right_points) < 2:
            print("Not enought points extracted. Total points on the left wall ->", len(left_points), "and right wall ->", len(right_points))
            return None, None, None, None
        
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
            #row_np_array = np.load("Depth_Output/" + self.filename)
            row_pointcloud = row_np_array['arr_0']
        except Exception as e:
            print(f"Error opening the .npz file {e}")
            return None, None

        # To visualie the pointcloud. Put debug flag = True
        if self.isdebug:
            print("Showing the initial pointcloud...")
            self.plot_row_pointcloud(row_pointcloud)

        # Determine if it is the end of the road. (This will set the self.end_of_row flag)
        self.endOfRow(row_pointcloud)

        # If we determine that this is the end of the row, we don't want to compute the angular_rate based on walls. As there are no walls here.
        if self.end_of_row == True:
            # Changing it to False just incase we want to run this in a loop without construting the class again
            self.end_of_row = False
            return 0.0, True
        
        # Function that takes a pointcloud, deletes height axis, removes duplicates, and sorts from furthest to nearest along the row
        row_pointcloud = self.preprocessPoints(row_pointcloud)

        # Extracts #self.num_of_points - 2' points on each wall
        left_edge_points, right_edge_points = self.extractEdgePoints(row_pointcloud)

        # Returns the slopes and intercepts of the two walls which is used to calculate the angle
        left_x_intercept, left_y_intercept, left_slope, right_x_intercept, right_y_intercept, right_slope = self.calculateInterceptfromPoints(left_edge_points, right_edge_points)
        
        # Returns the angle to turn by calculating the arctan
        angle_to_turn = self.calculateAnglefromIntercepts(left_x_intercept, left_y_intercept, left_slope, right_x_intercept, right_y_intercept, right_slope)

        # Publishes the Twist message on the topic cmd_vel
        self.publishTwistAngle(angle_to_turn, self.publisher)

        return angle_to_turn, self.end_of_row




# _______________________________________________________________________________________________________________________________________________

# DRIVER CODE
def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--input', type = str, help = "File name. Default is 3.npz", default = "3.npz")
    parser.add_argument('-d', '--debug', type = bool, help = "Debig mode flag. Prints information and displays plots", default = False)
    parser.add_argument('-n', '--num-of-points', type = int, help = "Number of points to extract from each wall. (Default is 2)", default = 2)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('row_nav')

    filename = args.input

    # If True, Prints information and displays plots
    debug_flag = args.debug

    # Refer to the README.md or (image name)
    # The number 2 works perfect for whichever depth camera the bot is using in the example
    num_of_points_to_extract = args.num_of_points + 2

    # We can use the rclpy spin functionality to keep the node alive and keep publishing to the topic at some frequency
    # but since we need to publish only once here, I am calling the function only once without spin.
    
    # Constructor and base function in the class
    angle_publisher = Angle_Publisher(filename, debug_flag, num_of_points_to_extract)
    angular_rate, end_of_row = angle_publisher.anglePublishOnce()

    print("The robot should turn", angular_rate, "degrees and end_of_row flag is", end_of_row)

    # cleanup
    angle_publisher.destroy_node()
    rclpy.shutdown()
# _______________________________________________________________________________________________________________________________________________

if __name__ == '__main__':
    main()