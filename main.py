# import the OpenCV library for computer vision
import cv2
import Kinematics.Kinematic as ik
import serial
import time
import numpy as np
import math


# 고정된 값 지정
marker_size = 5
cam_point = (30, 0, 45)

# 선언
#py_serial = serial.Serial(
#    # Window
#    port='COM3',
#    # 보드 레이트 (통신 속도)
#    baudrate=9600,
#)

class Point3D:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


matrix_coefficients = np.array([[4.9350028035371434e+02, 0., 3.1889472537064762e+02],
                                [0.,4.9752379240241839e+02,2.3323580951832940e+02],
                                [0., 0., 1.]])
distortion_coefficients = np.array([1.3514513045692980e-01, -4.8663060594638929e-01,
                                    6.3572242938879548e-04, 5.6972282484044220e-04,
                                    5.4433200932025450e-01])

point = Point3D(0,0,0)

# Load the dictionary that was used to generate the markers.
# There's different aruco marker dictionaries, this code uses 6x6
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

# Initialize the detector parameters using default values
parameters = cv2.aruco.DetectorParameters_create()
camera = cv2.VideoCapture(0)
# loop that runs the program forever
# at least until the "q" key is pressed
# creates an "img" var that takes in a camera frame

while True:
    ret, img = camera.read()
    height, width = img.shape[:2]
    # Convert to grayscale+
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # x,y 축 표시
    hh = int(height / 2)
    ww = int(width / 2)

    markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)

    # draw box around aruco marker within camera frame
    img = cv2.aruco.drawDetectedMarkers(img, markerCorners, markerIds)
    cv2.line(img, (0, hh), (width, hh), (255, 0, 0), 2)
    cv2.line(img, (ww, 0), (ww, height), (0, 0, 255), 2)
    # if a tag is found...
    if markerIds is not None:
        # for every tag in the array of detected tags...
        for i in range(len(markerIds)):

            print(markerIds[0])
            # detect aruco tags within the frame
            print('what x ?')
            point.x = int(input(int))
            print('what y ?')
            point.y = int(input(int))
            print('what z ?')
            point.z = int(input(int))
            print('Input clear')
            # get the center point of the tag
            center = markerCorners[i][0]
            M = cv2.moments(center)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            # draws a red dot on the marker center
            #cv2.circle(img, (cX, cY), 1, (0, 0, 255), 8)
            # writes the coordinates of the center of the tag
            #cv2.putText(img, str(cX) + "," + str(cY), (cX, cY), cv2.FONT_HERSHEY_COMPLEX, 0.7,(0, 255, 0), 2)
            (topLeft, topRight, bottomRight, bottomLeft) = markerCorners[i][0]


            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            # draw the bounding box of the ArUCo detection
            cv2.line(img, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(img, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(img, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(img, bottomLeft, topLeft, (0, 255, 0), 2)

            # marker axis
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorners[i], 0.02, matrix_coefficients, distortion_coefficients)
            (rvec - tvec).any()  # get rid of that nasty numpy value array error
            cv2.aruco.drawAxis(img, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)  # Draw axis
            print('rvec', rvec) #원점에서 마커의 변환 xyz 값
            rotMat = np.zeros((4, 4), np.float32)
            rotMat, _ = cv2.Rodrigues(rvec)
            print('rotMat', rotMat)
            # print('tvec', tvec) #회전축과 회전축의 각도 (rdian value)
            # print(tvec * (180/math.pi)) #회전축과 회전축의 각도 (angle value)
            #print('tvec[0]', tvec[0][0][0])
            # print('width : ', width)
            # print('height : ', height)

            # aruco marker rotation
            Px = (bottomRight[0] + bottomLeft[0]) / 2
            Py = (bottomRight[1] + bottomLeft[1]) / 2
            Py_axis = Point3D(Px, Py, 0)
            Aruco_T_Radian = -math.atan2((cY - Py_axis.y),(cX - Py_axis.x))
            #
            # Point gradient Radian
            gradient_Radian = math.atan2(-point.y,-point.x)

            # check value
            TransRadian = math.degrees(Aruco_T_Radian)
            print('Aruco_T_Radian - angle : ', TransRadian)
            CheckRadian = math.degrees(gradient_Radian)
            print('gradient_Radian - angle : ', CheckRadian)

            # 단순 계산 1 만큰 떨어진 x or y 축 값
            # 마커 중심 x과 점에 대한 XY 거리
            #Xl = (markerCorners[0][0][1][0] - cX) / (marker_size/2) * abs(point.x)
            #Yl = (markerCorners[0][0][1][1] - cY) / (marker_size/2) * abs(point.y)

            X_len = (markerCorners[0][0][1][0] - cX) / (marker_size / 2) * abs(point.x)
            Y_len = (markerCorners[0][0][1][1] - cY) / (marker_size / 2) * abs(point.y)
            Xl = ((X_len * math.cos(Aruco_T_Radian) - Y_len * math.sin(Aruco_T_Radian)))
            Yl = ((X_len * math.sin(Aruco_T_Radian) + Y_len * math.cos(Aruco_T_Radian)))
            print(Xl, Yl)
            print('check Point Xl,Yl : ', Xl, Yl)
            if (point.x > 0 and point.y > 0):
                Px1 = ((Xl * math.cos(-Aruco_T_Radian)) - (Yl * math.sin(-Aruco_T_Radian))) + cX
                Py1 = ((Xl * math.sin(-Aruco_T_Radian)) + (Yl * math.cos(-Aruco_T_Radian))) + cY
            elif (point.x < 0 and point.y > 0):
                Px1 = ((Xl * math.cos(-Aruco_T_Radian - (90*math.pi/180))) -
                       (Yl * math.sin(-Aruco_T_Radian - (90*math.pi/180)))) + cX
                Py1 = ((Xl * math.sin(-Aruco_T_Radian - (90*math.pi/180))) +
                       (Yl * math.cos(-Aruco_T_Radian - (90*math.pi/180)))) + cY
            elif (point.x > 0 and point.y < 0):
                Px1 = ((Xl * math.cos(-Aruco_T_Radian + (90*math.pi/180))) -
                       (Yl * math.sin(-Aruco_T_Radian + (90*math.pi/180)))) + cX
                Py1 = ((Xl * math.sin(-Aruco_T_Radian + (90*math.pi/180))) +
                       (Yl * math.cos(-Aruco_T_Radian + (90*math.pi/180)))) + cY
            elif (point.x < 0 and point.y < 0):
                Px1 = ((Xl * math.cos(-Aruco_T_Radian + math.pi)) - (Yl * math.sin(-Aruco_T_Radian + math.pi))) + cX
                Py1 = ((Xl * math.sin(-Aruco_T_Radian + math.pi)) + (Yl * math.cos(-Aruco_T_Radian + math.pi))) + cY

            elif(point.x == 0 and point.y < 0 or point.x < 0 and point.y == 0):
                Px1 = -((Xl * math.cos(-Aruco_T_Radian)) - (Yl * math.sin(-Aruco_T_Radian))) + cX
                Py1 = -((Xl * math.sin(-Aruco_T_Radian)) + (Yl * math.cos(-Aruco_T_Radian))) + cY

            else:
                Px1 = ((Xl * math.cos(-Aruco_T_Radian)) - (Yl * math.sin(-Aruco_T_Radian))) + cX
                Py1 = ((Xl * math.sin(-Aruco_T_Radian)) + (Yl * math.cos(-Aruco_T_Radian))) + cY

            #print('used_TransRad : ', Px1,' | used_TransRad : ', Py1)

            # img(video)에 좌표 표시
            Point_X = int(Px1)
            Point_Y = int(Py1)
            #print('check Point:', Point_X, Point_Y)

            cv2.circle(img, (Point_X, Point_Y), 1, (0, 0, 255), 8)
            cv2.putText(img, str(point.x) + "," + str(point.y), (Point_X, Point_Y), cv2.FONT_HERSHEY_COMPLEX, 0.35,
                        (255, 255, 0), 2)

            # 카메라 기준 중간 부터 마커 거리
            #fvy = -((cX - (width/2)) / abs((markerCorners[0][0][1][1] - cY) / (marker_size / 2)))
            #fvx = (cY - (height/2)) / abs((markerCorners[0][0][1][0] - cX) / (marker_size / 2))
            fvy = -((cX - (width/2)) /10)
            fvx = (cY - (height/2)) / 10
            print((width/2) - cX)
            print((height/2) - cY)

            print('x축 떨어진 거리 : ', fvx)
            print('y축 떨어진 거리 : ', fvy)
            # 마커기준 목표 좌표 계산
            NX = ((point.x * math.cos(Aruco_T_Radian)) - (point.y * math.sin(Aruco_T_Radian))) + fvx
            NY = -((point.x * math.sin(Aruco_T_Radian)) + (point.y * math.cos(Aruco_T_Radian))) + fvy
            print('aRUCO 마커기준 회전 X : ', NX)
            print('aRUCO 마커기준 회전 Y : ', NY)
            x = NX + cam_point[0]
            y = NY
            z = point.z
            print(x, y, z)

            Kinematics_Result = ik.Inverse_Kinematics(x, y, z)

            print('기구학 계산 후 각 팔의 각도 : ', Kinematics_Result)
    # Display the resulting frame
    cv2.imshow('frame', img)
    #python -> aduino
#    commend = input(Ik_value)
#    py_serial.write(commend.encode())
#    time.sleep(0.1)

    # handler to press the "q" key to exit the program
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
camera.release()
cv2.destroyAllWindows()