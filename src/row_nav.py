import numpy as np
import matplotlib.pyplot as plt
import rclpy

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


def plot_row_pointcloud(file):
    """
    file: [string]: npz file to be processed
    return: pointcloud in numpy array
    """
    row_np_array = np.load(file)
    row_pointcloud = row_np_array['arr_0']
    row_pointcloud = np.delete(row_pointcloud, 1, axis = 1)

    print("Before shape", row_pointcloud.shape)
    row_pointcloud = np.unique(row_pointcloud, axis = 0)
    print("After shape", row_pointcloud.shape)
    
    
    # Extract leftmost and rightmost points
    sorted_indices = np.argsort(row_pointcloud[:, 1])[::-1]
    row_pointcloud = row_pointcloud[sorted_indices]
    


    fig = plt.figure()
    ax = fig.add_subplot()
    ax.scatter(row_pointcloud[:, 0], row_pointcloud[:, 1], s = 0.1)

    ax.scatter(row_pointcloud[306, 0], row_pointcloud[1000, 1], c = "r")
    #ax.scatter(row_pointcloud[-1, 0], row_pointcloud[-1, 1], c = "r")

    ax.scatter(row_pointcloud[9000, 0], row_pointcloud[9000, 1], c = "g")
    #ax.scatter(row_pointcloud[-10000, 0], row_pointcloud[-10000, 1], c = "g")
    plt.show()

    return row_pointcloud


if __name__ == "__main__":
    pointcloud = plot_row_pointcloud("1.npz")