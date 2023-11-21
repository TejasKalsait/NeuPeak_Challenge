import numpy as np
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


def plot_row_pointcloud(file):
    """
    file: [string]: npz file to be processed
    return: pointcloud in numpy array
    """
    row_np_array = np.load(file)
    row_pointcloud = row_np_array['arr_0']

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.view_init(azim=0, elev=-180)
    ax.scatter(row_pointcloud[:, 2], row_pointcloud[:, 0], row_pointcloud[:, 1], s = 1)
    print(row_pointcloud[0, 0], row_pointcloud[0, 1], row_pointcloud[0, 2])
    plt.show()

    return row_pointcloud


if __name__ == "__main__":
    pointcloud = plot_row_pointcloud("2.npz")