from Utilities import MatrixMath
from Utilities import Rotations
import math

ROTATION_MAT = [
                [math.cos(-50)*math.cos(145), math.sin(0)*math.sin(-50)*math.cos(145)-math.cos(0)*math.sin(145), math.cos(0)*math.sin(-50)*math.cos(145)+math.sin(0)*math.sin(145)],
                [math.cos(-50)*math.sin(145), math.sin(0)*math.sin(-50)*math.sin(145)+math.cos(0)*math.sin(145), math.cos(0)*math.sin(-50)*math.sin(145)-math.sin(0)*math.cos(145)],
                [-math.sin(-50), math.sin(0)*math.cos(-50), math.cos(0)*math.sin(-50)]
]

VELOCITY_VEC = [[22.5],[0],[0]]

Position_der = MatrixMath.multiply(ROTATION_MAT, VELOCITY_VEC)

print("answer to Q1 part A->")

MatrixMath.matrixPrint(Position_der)

print("answer to Q1 part B->")

Mass = 0.5 # in kg
G = 9.8 # m/s^2
F_g = Mass * G
FORCE_VEC = [
                [-4.8 - math.sin(40)*F_g],
                [0],
                [-4.8 - math.cos(40)*F_g]
            ]

q1pb_ans = MatrixMath.scalarMultiply(1/Mass, FORCE_VEC)

MatrixMath.matrixPrint(q1pb_ans)

print("answer to Q1 part C->")

SKEW_PARTC = [
                [0,0,3.5],
                [0,0,0],
                [-3.5,0,0]
]

FORCE_VEC_PARTC = [[0],[0],[7.2]]

q1pc_ans = MatrixMath.add(MatrixMath.multiply(SKEW_PARTC,FORCE_VEC_PARTC),q1pb_ans)

MatrixMath.matrixPrint(q1pc_ans)


print("answer to Q2 part B->")

ROTATION_MAT = [
                [1, math.sin(90)*math.tan(-51.6), math.cos(90)*math.tan(-51.6)],
                [0, math.cos(90), -1*math.sin(90)],
                [0, math.sin(90)/math.cos(-51.6), math.cos(90)/math.cos(-51.6)]
]

ANGULAR_RATES_Q2B = [[0],[(2*math.pi)/90],[0]]

q2pb_ans = MatrixMath.multiply(ROTATION_MAT,ANGULAR_RATES_Q2B)

MatrixMath.matrixPrint(q2pb_ans)


print("Question 4 Rexp analysis:")
print("First Approach->forward euler integration on euler angles themselves")

#givens at t0

changeInTime = 0.1
                #ROLL, PITCH, YAW
Euler_angles = [[0],[0],[0]]
angle_rates  = [[1],[1],[1]]

roll = Euler_angles[0][0]
pitch = Euler_angles[1][0]
yaw = Euler_angles[2][0]

ROTATION_MAT_Q4PA = [
                [1, math.sin(roll)*math.tan(pitch), math.cos(roll)*math.tan(pitch)],
                [0, math.cos(roll), -1*math.sin(roll)],
                [0, math.sin(roll)/math.cos(pitch), math.cos(roll)/math.cos(pitch)]
]

ANGULAR_RATES_Q4PB = [[1],[1],[1]]

Q4PA_ans = MatrixMath.add(Euler_angles,MatrixMath.scalarMultiply(changeInTime, MatrixMath.multiply(ROTATION_MAT_Q4PA,ANGULAR_RATES_Q4PB)))

print("Ans->")
print(Q4PA_ans)

print("Q4 Part B ->")

VehicleToBodyRotationMat  = [
                [math.cos(pitch)*math.cos(yaw), math.cos(pitch)*math.sin(yaw), -1*math.sin(pitch)],
                [math.sin(roll)*math.sin(pitch)*math.cos(yaw)-math.cos(pitch)*math.sin(yaw), math.sin(roll)*math.sin(pitch)*math.sin(yaw)+math.cos(roll)*math.cos(yaw), math.sin(roll)*math.cos(pitch)],
                [math.cos(roll)*math.sin(pitch)*math.cos(yaw)+math.sin(roll)*math.sin(yaw), math.cos(pitch)*math.sin(pitch)*math.sin(yaw)-math.sin(roll)*math.cos(yaw), math.cos(roll)*math.cos(pitch)]
]

SKEW_4B = [
    [0,-1,1],
    [1,0,-1],
    [-1,1,0]
]
Rdv = MatrixMath.scalarMultiply(-1,MatrixMath.multiply(SKEW_4B, VehicleToBodyRotationMat))

Rnext = MatrixMath.add(VehicleToBodyRotationMat, MatrixMath.scalarMultiply(0.1, Rdv))

print("Euler angles at time t0.1 are")

print(Rotations.dcm2Euler(Rnext))

IDENTITY =[[1,0,0],[0,1,0],[0,0,1]]
angularMag = math.sqrt(3)
trigTerm = angularMag * 0.1

term1 = MatrixMath.subtract(IDENTITY, MatrixMath.scalarMultiply(math.sin(trigTerm)/angularMag, SKEW_4B))
term2 = MatrixMath.add(term1, MatrixMath.scalarMultiply((1-math.cos(trigTerm))/math.pow(angularMag,2),MatrixMath.multiply(SKEW_4B,SKEW_4B)))

RnextC = MatrixMath.multiply(term2, VehicleToBodyRotationMat)

print("euler angles at time 0.1 are:")

print(Rotations.dcm2Euler(RnextC))

print("part D")
x = 0.001
changeInTime = 0.001
for i in range(1, 100):

    changeInTime = changeInTime + x
                #ROLL, PITCH, YAW
    Euler_angles = [[0],[0],[0]]
    angle_rates  = [[1],[1],[1]]

    roll = Euler_angles[0][0]
    pitch = Euler_angles[1][0]
    yaw = Euler_angles[2][0]

    ROTATION_MAT_Q4PA = [
                    [1, math.sin(roll)*math.tan(pitch), math.cos(roll)*math.tan(pitch)],
                    [0, math.cos(roll), -1*math.sin(roll)],
                    [0, math.sin(roll)/math.cos(pitch), math.cos(roll)/math.cos(pitch)]
    ]

    ANGULAR_RATES_Q4PB = [[1],[1],[1]]

    Q4PA_ans = MatrixMath.add(Euler_angles,MatrixMath.scalarMultiply(changeInTime, MatrixMath.multiply(ROTATION_MAT_Q4PA,ANGULAR_RATES_Q4PB)))

    print("Ans->")
    print(Q4PA_ans)

    print("Q4 Part B ->")

    VehicleToBodyRotationMat  = [
                    [math.cos(pitch)*math.cos(yaw), math.cos(pitch)*math.sin(yaw), -1*math.sin(pitch)],
                    [math.sin(roll)*math.sin(pitch)*math.cos(yaw)-math.cos(pitch)*math.sin(yaw), math.sin(roll)*math.sin(pitch)*math.sin(yaw)+math.cos(roll)*math.cos(yaw), math.sin(roll)*math.cos(pitch)],
                    [math.cos(roll)*math.sin(pitch)*math.cos(yaw)+math.sin(roll)*math.sin(yaw), math.cos(pitch)*math.sin(pitch)*math.sin(yaw)-math.sin(roll)*math.cos(yaw), math.cos(roll)*math.cos(pitch)]
    ]

    SKEW_4B = [
        [0,-1,1],
        [1,0,-1],
        [-1,1,0]
    ]
    Rdv = MatrixMath.scalarMultiply(-1,MatrixMath.multiply(SKEW_4B, VehicleToBodyRotationMat))

    Rnext = MatrixMath.add(VehicleToBodyRotationMat, MatrixMath.scalarMultiply(changeInTime, Rdv))

    print("Euler angles at time tchangeInTime are")

    print(Rotations.dcm2Euler(Rnext))

    IDENTITY =[[1,0,0],[0,1,0],[0,0,1]]
    angularMag = math.sqrt(3)
    trigTerm = angularMag * changeInTime

    term1 = MatrixMath.subtract(IDENTITY, MatrixMath.scalarMultiply(math.sin(trigTerm)/angularMag, SKEW_4B))
    term2 = MatrixMath.add(term1, MatrixMath.scalarMultiply((1-math.cos(trigTerm))/math.pow(angularMag,2),MatrixMath.multiply(SKEW_4B,SKEW_4B)))

    RnextC = MatrixMath.multiply(term2, VehicleToBodyRotationMat)

    print("euler angles at time changeInTime are:")

    print(Rotations.dcm2Euler(RnextC))