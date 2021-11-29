#!/usr/bin/env python3 -W ignore::DeprecationWarning
# asl.py

import asyncio
import cv2
import time
import keyboard
import math

import numpy as np
import mediapipe as mp

from mqtt import MODE


class handDetector():
    def __init__(self, mode=False, maxHands=2, modelComplexity=1, detectionConfidence=0.5, trackConfidence=0.5):
        self.mode = mode
        self.maxHands = maxHands
        self.detectionConfidence = detectionConfidence
        self.trackConfidence = trackConfidence

        # related to hand detection module
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(mode, maxHands, modelComplexity, detectionConfidence, trackConfidence)
        self.mpDraw = mp.solutions.drawing_utils

    def findHands(self, img, draw=True):
        # convert to RGB
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)
        h, w, c = img.shape

        # print(results.multi_hand_landmarks)
        # draw hand
        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(img, handLms, self.mpHands.HAND_CONNECTIONS)
        return img

    def findPosition(self, img, handIndex=0, raw=False, includeIndex=True):
        lmList = []
        h, w, c = img.shape

        # print(h, w, c)
        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handIndex]

            # get all hand landmarks
            for id, lm in enumerate(myHand.landmark):
                # print(id, lm)
                if raw:
                    if includeIndex:
                        lmList.append([id, lm.x, lm.y])
                    else:
                        lmList.append([lm.x, lm.y])
                else:
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    if includeIndex:
                        lmList.append([id, cx, cy])
                    else:
                        lmList.append([cx, cy])
        return lmList

    def getBoundingBox(self, img, padded_amount=10):
        lmList = []
        h, w, c = img.shape

        x1 = 0
        x2 = 0
        y1 = 0
        y2 = 0

        if self.results.multi_hand_landmarks:
            for id, lm in enumerate(self.results.multi_hand_landmarks):
                for landmark in lm.landmark:
                    lmList.append((int(landmark.x * w), int(landmark.y * h), int(landmark.z * w)))

                # get all x-coordinates values
                x_coord = np.array(lmList)[:, 0]

                # get all y-coordinates values
                y_coord = np.array(lmList)[:, 1]

                # get bounding box coordinates with padding
                # x1, y1 is top left coord
                # x2, y2 is bottom right coord
                x1 = max(int(np.min(x_coord) - padded_amount), 1)
                y1 = max(int(np.min(y_coord) - padded_amount), 1)
                x2 = min(int(np.max(x_coord) + padded_amount), w)
                y2 = min(int(np.max(y_coord) + padded_amount), h)

        return (x1, y1), (x2, y2)

class static_recognition():
    def __init__(self, mqClient):
        self.DEBOUNCE_TIME = 0.15     
        self.keyPressedDict = {'b': 0}
        self.interestJoints = [0, 4, 8, 12, 16, 20]
        self.ExitProgram = False
        self.client = mqClient

    def getJointCoord(self, joints, joint):
        # get the x y coord of a joint given its index
        # * 2 since the list is in pairs x0, y0, x1, y1, etc...
        return joints[(joint * 2)], joints[(joint * 2) + 1]

    def getFeatures(self, joints):
        features = []
        allX = []
        allY = []
        allAngles = []
        for joint1 in self.interestJoints:
            for joint2 in self.interestJoints:
                if joint2 > joint1:
                    # print(joint1, joint2)
                    joint1Coord = self.getJointCoord(joints, joint1)
                    joint2Coord = self.getJointCoord(joints, joint2)

                    (xDist, yDist) = (joint1Coord[0] - joint2Coord[0]), (joint1Coord[1] - joint2Coord[1])
                    angle = math.atan2(yDist, xDist) * 180.0 / math.pi + 180.0
                    allX.append(xDist)
                    allY.append(yDist)
                    allAngles.append(angle)

        features.append(allX)
        features.append(allY)
        features.append(allAngles)
        return features

    def getKeyPressed(self, key, currTime):
        if key in self.keyPressedDict:
            timePassed = float(currTime - self.keyPressedDict[key])
            if keyboard.is_pressed(key) and timePassed > self.DEBOUNCE_TIME:
                self.keyPressedDict[key] = currTime
                return True
        return False

    def disconnect(self):
        self.ExitProgram = True
        # program exits
        print(f'==========   ASL is disconnected   ==========')

    async def run_asl(self):
        detector = handDetector(maxHands=1, detectionConfidence=0.9, trackConfidence=0.7)

        # set up webcam
        wCam, hCam = 640, 480
        # framerate
        prevTime = 0
        # 0 sets the webcam to be used
        cam = cv2.VideoCapture(0)
        cam.set(3, wCam)
        cam.set(4, hCam)

        # flag to show bounding box
        showBoundingBox = True

        print("==========   Set up asl   ==========")

        # to reduce the load on the classifier
        count = 0

        while not self.ExitProgram:
            currTime = time.time()
            success, img = cam.read()

            img = detector.findHands(img, draw=False)
            (x1, y1), (x2, y2) = detector.getBoundingBox(img, padded_amount=50)

            if x1 == 0 and y1 == 0 and x2 == 0 and y2 == 0:
                boundingBoxExist = False
            else:
                boundingBoxExist = True

            landmarks = detector.findPosition(img, includeIndex=False, raw=False)
            joints = [allCoords for coord in landmarks for allCoords in coord]

            if self.getKeyPressed('b', currTime):
                showBoundingBox = not showBoundingBox
            
            if self.client.gesture == 'IDLE':
                if len(joints) > 0:
                    features = np.reshape(self.getFeatures(joints), (1, -1))
                    if count >= 10:
                        # send to server for prediction
                        self.client.publish(MODE.ASL.name, features.tolist())
                        count = 0

                # show bounding box
                if showBoundingBox and boundingBoxExist:
                    cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 0), 3)
                    # show ASL prediction only when bounding box exist
                    if self.client.asl:
                        cv2.rectangle(img, (200, 430), (440, hCam), (0,0,0), -1)
                        # im guessing 20 is the default font size, so from 320, you shift left by half of the len(prediction)
                        offset = 20*(0.5 if not len(self.client.asl)/2 else len(self.client.asl)/2)
                        cv2.putText(img, self.client.asl, (320-int(offset), 470), cv2.FONT_HERSHEY_PLAIN, 2, (180, 105, 255), 3)
            else:
                # Gesture prediction
                if self.client.gesture:
                    cv2.rectangle(img, (200, 430), (440, hCam), (0,0,0), -1)
                    # im guessing 20 is the default font size, so from 320, you shift left by half of the len(prediction)
                    offset = 20*(0.5 if not len(self.client.gesture)/2 else len(self.client.gesture)/2)
                    cv2.putText(img, self.client.gesture, (320-int(offset), 470), cv2.FONT_HERSHEY_PLAIN, 2, (180, 105, 255), 3)

            count += 1

            cv2.imshow("Group 7", img)
            await asyncio.sleep(0.1)  # sleep for 100ms
            
            if cv2.waitKey(1) == ord('q'):
                self.ExitProgram = True
            
        cam.release()
        cv2.destroyAllWindows()
        print("==========   Closing asl   ==========")
        
     