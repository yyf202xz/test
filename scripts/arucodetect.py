# The following code is used to watch a video stream, detect Aruco markers, and use
# a set of markers to determine the posture of the camera in relation to the plane
# of markers.
#
# Assumes that all markers are on the same plane, for example on the same piece of paper
#
# Requires camera calibration (see the rest of the project for example calibration)

import math
import numpy
import cv2
import cv2.aruco as aruco

def arucodetect(QueryImg):

    # Constant parameters used in Aruco methods
    ARUCO_PARAMETERS = aruco.DetectorParameters_create()
    ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_6X6_1000)

    # Create grid board object we're using in our stream
    board = aruco.GridBoard_create(
            markersX=2,
            markersY=2,
            markerLength=0.09,
            markerSeparation=0.01,
            dictionary=ARUCO_DICT)

    gray = cv2.cvtColor(QueryImg, cv2.COLOR_BGR2GRAY)

    # Detect Aruco markers
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
    
    # Make sure all 5 markers were detected before printing them out
    if ids is not None:

        #pos0 = (corners[0][0][2]+corners[0][0][3])/2
        areas = numpy.array(list(map(cv2.contourArea,corners[0])))
        length = int(math.sqrt(areas))
        perspective1 = numpy.float32([corners[0][0][3],corners[0][0][2],corners[0][0][1],corners[0][0][0]])
        perspective2 = numpy.float32([[0, 0],[length, 0],[length, length],[0, length]])  
        pos = (length/2,0)  
        psp_matrix = cv2.getPerspectiveTransform(perspective1,perspective2)
        rev_psp_matrix = numpy.linalg.inv(psp_matrix)
        
        #pos.reshape((2,1))
        pos1 = numpy.append (pos, [1])
        pos2 = numpy.dot(rev_psp_matrix ,pos1)
        pos2 = pos2 / pos2[2]
        pos2 = numpy.delete(pos2,2,0)
        pos2 = pos2.astype(numpy.int)
        # Print corners and ids to the console
        #for i, corner in zip(ids, corners):
            
            #print('ID: {}; Corners: {}'.format(i, corner))

        #print(pos2)

        # Outline all of the markers detected in our image
        QueryImg = aruco.drawDetectedMarkers(QueryImg, corners, borderColor=(0, 0, 255))
        
        QueryImg = cv2.circle(QueryImg,tuple(pos2),5,(255,0,0),-1)
        #QueryImg = cv2.circle(QueryImg,tuple(pos0),5,(255,0,255),-1)
    
    else:
        pos2 = (0,0)

    return pos2, QueryImg
