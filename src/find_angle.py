import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import os
import argparse
import matplotlib.pyplot as plt


class Angle_Publisher(Node):

    def __init__(self):
        super().__init__('angle_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def preprocessPoints(self, row_pointcloud):

        # Deleting the height information as my algorithm doesnt need it
        row_pointcloud = np.delete(row_pointcloud, 1, axis = 1)

        # Remove duplicate rows 
        row_pointcloud = np.unique(row_pointcloud, axis = 0)

        # Sorting from furthest to closesnt to the camera
        sorted_indices = np.argsort(row_pointcloud[:, 1])[::-1]
        row_pointcloud = row_pointcloud[sorted_indices]

        print("After deleting height-axis, removing duplicates, and sorting with x-values, shape is", row_pointcloud.shape)

        return row_pointcloud
    
    def extractEdgePoints(self, row_pointcloud, plot = False):

        left_points, right_points = [], []
        max_on_y = row_pointcloud[0,1]
        num_of_pivots = 4
        partitions = np.linspace(max_on_y, 0.0, num_of_pivots)

        print("Extracting Edge points")

        for i in range(1, len(partitions)):

            leftmost, rightmost = [float('inf'), -1], [float('-inf'), -1]

            for x, y in row_pointcloud:

                # Base Case 1
                if y > partitions[i - 1]:
                    continue
                
                # Base case 2
                if y < partitions[i]:
                    #remove the first (highest) value
                    left_points.append(leftmost)
                    right_points.append(rightmost)
                    leftmost, rightmost = [float('inf'), -1], [float('-inf'), -1]
                    break

                # left point
                if x < leftmost[0]:
                    # found new leftmost
                    leftmost = [x, y]

                # right point
                if x > rightmost[0]:
                    # found new rightmost
                    rightmost = [x, y]

        left_points = np.array(left_points)
        right_points = np.array(right_points)

        if plot:
            # Plotting in x-y plane
            fig = plt.figure()
            ax = fig.add_subplot()
            ax.scatter(row_pointcloud[:, 0], row_pointcloud[:, 1], s = 0.1)

            ax.scatter(left_points[:, 0], left_points[:, 1], c = 'r')
            ax.scatter(right_points[:, 0], right_points[:, 1], c = 'r')

            plt.show()

        return left_points, right_points
    
    def calculateInterceptfromPoints(self, left_points, right_points):

        # Left line
        lx1, ly1 = left_points[0]
        lx2, ly2 = left_points[1]

        try:
            l_slope = (ly2 - ly1) / (lx2 - lx1)
        except:
            print("Found the left wall fully straight")
            return None, None, None, None
        
        # Right line
        rx1, ry1 = right_points[0]
        rx2, ry2 = right_points[1]

        try:
            r_slope = (ry2 - ry1) / (rx2 - rx1)
        except:
            print("Found the right wall fully straight")
            return None, None, None, None
        
        print("Left wall slope is", l_slope, "and right wall slope is", r_slope)

        l_y_intercept = -(l_slope * lx2) + ly1
        l_x_intercept = lx1 - (ly1 / l_slope)

        r_y_intercept = -(r_slope * rx2) + ry1
        r_x_intercept = rx1 - (ry1 / r_slope)

        return (l_x_intercept, l_y_intercept, r_x_intercept, r_y_intercept)

    
    def anglePublishOnce(self):

        try:
            # Opening file
            row_np_array = np.load("../Depth_Output/1.npz")
            row_pointcloud = row_np_array['arr_0']
        except Exception as e:
            print(f"Error opening the .npz file {e}")

        row_pointcloud = self.preprocessPoints(row_pointcloud)

        left_edge_points, right_edge_points = self.extractEdgePoints(row_pointcloud, plot = True)

        left_x_intercept, left_y_intercept, right_x_intercept, right_y_intercept = self.calculateInterceptfromPoints(left_edge_points, right_edge_points)

        if not left_x_intercept or not right_x_intercept:
            #Publish 0 deg
            print("Zero Degree Turn")

        # Left line angle
        left_angle_turn = 90 + np.degrees(np.arctan(left_y_intercept/left_x_intercept))

        # Right line angle
        right_angle_turn = 90 + np.degrees(np.arctan(right_y_intercept/right_x_intercept))
        print((left_angle_turn + right_angle_turn) / 2.0)
        



def main(args=None):
    rclpy.init(args=args)

    angle_publisher = Angle_Publisher()
    angle_publisher.anglePublishOnce()        


    angle_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()