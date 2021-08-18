from numpy import *   # imports all function so we don't have to use np.function()
import matplotlib.pyplot as plt

shoulder = 500
elbow = 500
wrist = 50

offsetX = 90
offsetY = 90
end_angle = 60

zero = ((0, 60, 60, 0, 0, 60 ), (125, 125, 0, 0, 125, 125))
one = ((75, 75), (125, 0))
two = ((0, 80, 80, 0, 0,80), (125, 125, 62.5, 62.5, 0, 0))
four = ((70, 70, 0, 95), (0, 125, 45, 45))
six = ((0, 35, 0, 0, 0, 75, 75, 0, 75), (125, 125, 125, 125, 0, 0, 50, 50, 50))
nine = ((0, 70, 70, 0, 0, 80), (0, 0, 125, 125, 55, 55))

MyStudentID = (one, four, zero, nine, six, two, six, four)

angle_List1 = []
angle_List2 = []
angle_List3 = []

coordinate_List_X_axis = []
coordinate_List_Y_axis = []
coordinate_List = []

def forward_kinematics(angleList1, angleList2, angleList3):
        for i in range(len(angleList1)):       
                theta_1 = angleList1[i]
                theta_2 = angleList2[i]
                theta_3 = angleList3[i]
                # Angles
                theta_1 = (theta_1/180)*pi  # theta 1 in radians
                theta_2 = (theta_2/180)*pi  # theta 2 in radians
                theta_3 = (theta_3/180)*pi  # theta 3 in radians

                # DH Parameter Table for 3 DOF Planar
                PT = [[theta_1, 0, shoulder, 0],
                         [theta_2, 0, elbow, 0],
                         [theta_3, 0, wrist, 0]]

                # Homogeneous Transformation Matrices
                i = 0
                H0_1 = [[cos(PT[i][0]), -sin(PT[i][0])*cos(PT[i][1]), sin(PT[i][0])*sin(PT[i][1]), PT[i][2]*cos(PT[i][0])],
                            [sin(PT[i][0]), cos(PT[i][0])*cos(PT[i][1]), -cos(PT[i][0])*sin(PT[i][1]), PT[i][2]*sin(PT[i][0])],
                            [0, sin(PT[i][1]), cos(PT[i][1]), PT[i][3]],
                            [0, 0, 0, 1]]

                i = 1
                H1_2 = [[cos(PT[i][0]), -sin(PT[i][0])*cos(PT[i][1]), sin(PT[i][0])*sin(PT[i][1]), PT[i][2]*cos(PT[i][0])],
                            [sin(PT[i][0]), cos(PT[i][0])*cos(PT[i][1]), -cos(PT[i][0])*sin(PT[i][1]), PT[i][2]*sin(PT[i][0])],
                            [0, sin(PT[i][1]), cos(PT[i][1]), PT[i][3]],
                            [0, 0, 0, 1]]

                i = 2
                H2_3 = [[cos(PT[i][0]), -sin(PT[i][0])*cos(PT[i][1]), sin(PT[i][0])*sin(PT[i][1]), PT[i][2]*cos(PT[i][0])],
                            [sin(PT[i][0]), cos(PT[i][0])*cos(PT[i][1]), -cos(PT[i][0])*sin(PT[i][1]), PT[i][2]*sin(PT[i][0])],
                            [0, sin(PT[i][1]), cos(PT[i][1]), PT[i][3]],
                            [0, 0, 0, 1]]

                print("H0_1 =")
                print(matrix(H0_1))
                print("H1_2 =")
                print(matrix(H1_2))
                print("H2_3 =")
                print(matrix(H2_3))

                H0_2 = dot(H0_1,H1_2)
                H0_3 = dot(H0_2,H2_3)

                print("H0_3 =")
                print(matrix(H0_3))

                coordinate_List_X_axis.append(H0_2[0][3])
                coordinate_List_Y_axis.append(H0_3[1][3])

                print(coordinate_List_X_axis)
                print(coordinate_List_Y_axis)
    

def inverse_kinematics(px, py, phi):

        # Desired Position of End effector
   
        phi = deg2rad(phi)

        # Equations for Inverse kinematics
        wx = px - wrist*cos(phi)
        wy = py - wrist*sin(phi)

        delta = wx**2 + wy**2
        c2 = ( delta -shoulder**2 -elbow**2)/(2*shoulder*elbow)
        s2 = sqrt(1-c2**2)  # elbow down
        theta_2 = arctan2(s2, c2)

        s1 = ((shoulder+elbow*c2)*wy - elbow*s2*wx)/delta
        c1 = ((shoulder+elbow*c2)*wx + elbow*s2*wy)/delta
        theta_1 = arctan2(s1,c1)
        theta_3 = phi-theta_1-theta_2

        
        print('theta_1: ', rad2deg(theta_1))
        print('theta_2: ', rad2deg(theta_2))
        print('theta_3: ', rad2deg(theta_3))

        wristangle = rad2deg(theta_1)
        elbowangle = rad2deg(theta_2)
        shoulderangle = rad2deg(theta_3)

        angle_List1.append(wristangle)         
        angle_List2.append(elbowangle)                 
        angle_List3.append(shoulderangle) 
               
def DrawNumber(StudentIDNumber, offset_X, offset_y, end):
        for i in range(len(StudentIDNumber[0])):
                inverse_kinematics( StudentIDNumber[0][i] + offset_X, StudentIDNumber[1][i] + offset_y, end)

def DrawNumbersList(StudentIDNumbers):
    for i in range(len(StudentIDNumbers)):
        DrawNumber(StudentIDNumbers[i], offsetX + 110 * i, offsetY, end_angle)
    forward_kinematics(angle_List1, angle_List2, angle_List3)

def plotStudentID():
    for i in range(len(coordinate_List_X_axis)):
        coordinate = [coordinate_List_X_axis[i],coordinate_List_Y_axis[i]]
        coordinate_List.append(coordinate)    
    x,y = zip(*coordinate_List)                  
    fig, ax = plt.subplots()                
    plt.plot(x,y)                                               
    plt.title('Draw 14096264')
    ax.set_aspect('equal', adjustable = 'box')
    plt.show()

DrawNumbersList(MyStudentID)
print(coordinate_List_X_axis)
print(coordinate_List_Y_axis)
plotStudentID()