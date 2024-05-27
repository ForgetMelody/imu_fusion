#read points in ~/catkin_ws/data/position.txt
import numpy as np

#Rotation matrix
# y-axis rotation 30 degrees
matrix = np.array([[np.cos(np.pi/6), 0, np.sin(np.pi/6)], [0, 1, 0], [-np.sin(np.pi/6), 0, np.cos(np.pi/6)]])

if __name__ == '__main__':
    #new file

    f = open('fast_lio_fixed.txt', 'w')
    with open('/home/sosilent/catkin_ws/data/paths/fast_lio_raw.txt', 'r') as file:
        lines = file.readlines()
        for line in lines:
            x,y,z = line.split()
            point = np.array([float(x),float(y),float(z)])
            new_point = np.dot(matrix, point)
            f.write(str(new_point[0]) + " " + str(new_point[1]) + " " + str(new_point[2]) + "\n")
