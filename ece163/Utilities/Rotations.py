"""
Engineer: Robert Gaines
File Name : Rotations.py
Purpose : Contains functions to compute Rotation Matrix Math for use in a flight sim
"""
import math
from . import MatrixMath


def dcm2Euler(DCM):
    yaw = pitch = roll = 0
    #returns yaw,pitch,roll in radians
    
    yaw = math.atan2(DCM[0][1], DCM[0][0])
    
    pitch = math.sinh(DCM[0][2])  #check for +-1 bounds
    roll = math.atan2(DCM[1][2], DCM[2][2])




    a = [yaw, pitch, roll]

    return a


def euler2DCM(yaw, pitch, roll):


    matrix=[[0,0,0],[0,0,0],[0,0,0]]
    #first row of DCM
    matrix[0][0] = math.cos(pitch) * math.cos(yaw)
    matrix[0][1] = math.cos(pitch) * math.sin(yaw)
    matrix[0][2] = -1 * math.sin(pitch)

    #second row
    matrix[1][0] = math.sin(roll)*math.sin(pitch)*math.cos(yaw) - math.cos(roll)*math.sin(yaw)
    matrix[1][1] = math.sin(roll)*math.sin(pitch)*math.sin(yaw) + math.cos(roll)*math.cos(yaw)
    matrix[1][2] = math.sin(roll)*math.cos(pitch)

    #third row
    matrix[2][0] = math.cos(roll)*math.sin(pitch)*math.cos(yaw) + math.sin(roll)*math.sin(yaw)
    matrix[2][1] = math.cos(roll)*math.sin(pitch)*math.sin(yaw) - math.sin(roll)*math.cos(yaw)
    matrix[2][2] = math.cos(roll)*math.cos(pitch)

    return matrix


#points is a Nx3 matrix of points (x,y,z);
#function will take xyz coordinates in a NED frame and convert them for the ENU frame

# Pned = [North,
#         East
#         Down]

#Penu = [East
#        North
#        Up]
def ned2enu(points):
    #basically x,y coordinates are switched in the vector, and the z coordinate is negated.
    temp = 0
    for point in points:
        temp = point[0]
        point[0] = point[1]
        point[1] = temp
        point[2] = point[2] * -1

    return points
