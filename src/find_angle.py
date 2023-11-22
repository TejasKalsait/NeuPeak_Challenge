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

        print("After deleting height-axis, removing duplicates, and sorting with x-values, (x, y) is", row_pointcloud[0, 0], row_pointcloud[0, 1], "and shape is", row_pointcloud.shape)

        return row_pointcloud
    
    def extractEdgePoints(self, row_pointcloud, plot):

        left_points, right_points = [], []
        max_on_y = row_pointcloud[0,1]
        num_of_pivots = 10
        partitions = np.linspace(max_on_y, 0.0, num_of_pivots)
        # print(partitions)

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


    
    
    def anglePublishOnce(self):

        
        # Opening file
        row_np_array = np.load("../Depth_Output/3.npz")
        row_pointcloud = row_np_array['arr_0']

        row_pointcloud = self.preprocessPoints(row_pointcloud)

        left_edge_points, right_edge_points = self.extractEdgePoints(row_pointcloud, plot = True)

        return


def main(args=None):
    rclpy.init(args=args)

    angle_publisher = Angle_Publisher()
    angle_publisher.anglePublishOnce()        


    angle_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()