from controller import Robot, Motor, DistanceSensor, Camera, Emitter, GPS
import struct
from random import randint
import numpy as np
import cv2 as cv

timeStep = 32
maxV = 5

robot = Robot()

motorL = robot.getDevice("ML motor")
motorR = robot.getDevice("MR motor")

speeds = [maxV, maxV]

USFR1 = robot.getDevice("USFR1")
USFR1.enable(timeStep)
USFR2 = robot.getDevice("USFR2")
USFR2.enable(timeStep)

USFL1 = robot.getDevice("USFL1")
USFL1.enable(timeStep)
USFL2 = robot.getDevice("USFL2")
USFL2.enable(timeStep)

USR = robot.getDevice("USR")
USR.enable(timeStep)

USL = robot.getDevice("USL")
USL.enable(timeStep)

USOR = robot.getDevice("USOpendR")
USOR.enable(timeStep)
USOL = robot.getDevice("USOpendL")
USOL.enable(timeStep)

cam = robot.getDevice("CF")
cam.enable(timeStep)

camR = robot.getDevice("CR")
camR.enable(timeStep)

camL = robot.getDevice("CL")
camL.enable(timeStep)

CS = robot.getDevice("CS")
CS.enable(timeStep)

emitter = robot.getDevice("emitter")

gps = robot.getDevice("gps")
gps.enable(timeStep)

motorL.setPosition(float("inf"))
motorR.setPosition(float("inf"))


def numToBlock(num):
    if num > 0.7:
        return '▁'
    elif num > 0.6:
        return '▂'
    elif num > 0.5:
        return '▃'
    elif num > 0.4:
        return '▄'
    elif num > 0.3:
        return '▅'
    elif num > 0.2:
        return '▆'
    elif num > 0.1:
        return '▇'
    elif num > 0:
        return '█'


def detectVisualSimple(image_data, cam):
    coords_list = []
    img = np.array(np.frombuffer(image_data, np.uint8).reshape((cam.getHeight(), cam.getWidth(), 4)))
    img[:, :, 2] = np.zeros([img.shape[0], img.shape[1]])

    # convert from BGR to HSV color space
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # apply threshold
    thresh = cv.threshold(gray, 140, 255, cv.THRESH_BINARY)[1]

    # draw all contours in green and accepted ones in red
    contours, h = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    for c in contours:
        if cv.contourArea(c) > 600:
            coords = list(c[0][0])
            coords_list.append(coords)
            print("Victim at x=" + str(coords[0]) + " y=" + str(coords[1]))
            return True
        else:
            return False


def checkVic(img):
    img = np.frombuffer(img, np.uint8).reshape((cam.getHeight(), cam.getWidth(), 4))
    # print('--->h: '+str(cam.getHeight())+'  w: '+str(cam.getWidth())+'<---')
    img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    img, thresh = cv.threshold(img, 80, 255, cv.THRESH_BINARY_INV)
    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        x, y, w, h = cv.boundingRect(cnt)
        contArea = cv.contourArea(cnt)
        ratio = w / h
        if contArea > 300 and contArea < 1500 and ratio > 0.7 and ratio <= 1:
            return True
    return False


def report(victimType):
    motorL.setVelocity(0)
    motorR.setVelocity(0)
    delay(1300)
    victimType = bytes(victimType, "utf-8")
    posX = int(gps.getValues()[0] * 100)
    posZ = int(gps.getValues()[2] * 100)
    message = struct.pack("i i c", posX, posZ, victimType)
    emitter.send(message)
    robot.step(timeStep)


def delay(ms):
    initTime = robot.getTime()
    while robot.step(timeStep) != -1:
        if (robot.getTime() - initTime) * 1000.0 > ms:
            break


def getColor():
    img = CS.getImage()
    return CS.imageGetGray(img, CS.getWidth(), 0, 0)


def rotR():
    speeds[0] = maxV
    speeds[1] = maxV * (-0.2)


def slowRotR():
    speeds[0] = maxV
    speeds[1] = maxV * (0.5)


def rotL():
    speeds[0] = maxV * (-0.2)
    speeds[1] = maxV


def slowRotL():
    speeds[0] = maxV * (0.5)
    speeds[1] = maxV


def rot():
    if LRD == True:
        speeds[0] = maxV * (-1)
        speeds[1] = maxV
    else:
        speeds[0] = maxV
        speeds[1] = maxV * (-1)


d = False
forD = 0
forLRD = 0
LRD = True
a = 0
posX = 0
posZ = 0
posXA = [0] * 100
posZA = [0] * 100

posZh = [[-10000, -10000], [-10000, -10000], [-10000, -10000], [-10000, -10000], [-10000, -10000], [-10000, -10000],
         [-10000, -10000], [-10000, -10000], [-10000, -10000], [-10000, -10000]]
nZh = 0
flagZh = False

while robot.step(timeStep) != -1:
    # print(numToBlock(USL.getValue()),numToBlock(USFL2.getValue()),numToBlock(USFL1.getValue()),numToBlock(USF.getValue()),numToBlock(USFR1.getValue()),numToBlock(USFR2.getValue()),numToBlock(USR.getValue()))
    if detectVisualSimple(cam.getImage(), cam):
        posX = int(gps.getValues()[0] * 100)
        posZ = int(gps.getValues()[2] * 100)
        for i in range(0, 10):
            if ((posX // 10) * 10 == (posZh[i][0] // 10) * 10 or ((posX // 10) - 1) * 10 == (
                    posZh[i][0] // 10) * 10 or ((posX // 10) + 1) * 10 == (posZh[i][0] // 10) * 10) \
                    and ((posX // 10) * 10 == (posZh[i][1] // 10) * 10 or ((posX // 10) - 1) * 10 == (
                    posZh[i][1] // 10) * 10 or ((posX // 10) + 1) * 10 == (posZh[i][1] // 10) * 10):
                print('camF FALSE')
                flagZh = True
        if flagZh == False:
            report('T')
            print('camF')
            posZh[nZh] = [posX, posZ]
            print(posZh[nZh])
            nZh += 1
        else:
            flagZh = False

    if detectVisualSimple(camR.getImage(), cam):
        posX = int(gps.getValues()[0] * 100)
        posZ = int(gps.getValues()[2] * 100)
        for i in range(0, 10):
            if ((posX // 10) * 10 == (posZh[i][0] // 10) * 10 or ((posX // 10) - 1) * 10 == (
                    posZh[i][0] // 10) * 10 or ((posX // 10) + 1) * 10 == (posZh[i][0] // 10) * 10) \
                    and ((posX // 10) * 10 == (posZh[i][1] // 10) * 10 or ((posX // 10) - 1) * 10 == (
                    posZh[i][1] // 10) * 10 or ((posX // 10) + 1) * 10 == (posZh[i][1] // 10) * 10):
                print('camR FALSE')
                flagZh = True
        if flagZh == False:
            report('T')
            print('camR')
            posZh[nZh] = [posX, posZ]
            print(posZh[nZh])
            nZh += 1
        else:
            flagZh = False

    if detectVisualSimple(camL.getImage(), cam):
        posX = int(gps.getValues()[0] * 100)
        posZ = int(gps.getValues()[2] * 100)
        for i in range(0, 10):
            if ((posX // 10) * 10 == (posZh[i][0] // 10) * 10 or ((posX // 10) - 1) * 10 == (
                    posZh[i][0] // 10) * 10 or ((posX // 10) + 1) * 10 == (posZh[i][0] // 10) * 10) \
                    and ((posX // 10) * 10 == (posZh[i][1] // 10) * 10 or ((posX // 10) - 1) * 10 == (
                    posZh[i][1] // 10) * 10 or ((posX // 10) + 1) * 10 == (posZh[i][1] // 10) * 10):
                print('camL FALSE')
                flagZh = True
        if flagZh == False:
            report('T')
            print('camL')
            posZh[nZh] = [posX, posZ]
            print(posZh[nZh])
            nZh += 1
        else:
            flagZh = False

    speeds[0] = maxV
    speeds[1] = maxV
    posX = int(gps.getValues()[0] * 100)
    posZ = int(gps.getValues()[2] * 100)
    posXA[a] = posX
    posZA[a] = posZ
    a += 1
    if a > 99:
        a = 0
    forI = 0
    for i in range(0, 100):
        if posXA[i] == posX and posZA[i] == posZ:
            forI += 1

    if forI > 98:
        print('help')
        rot()
        motorL.setVelocity(speeds[0])
        motorR.setVelocity(speeds[1])
        delay(randint(700, 900))
        motorL.setVelocity(maxV)
        motorR.setVelocity(maxV)
        delay(randint(0, 100))

    if LRD == True:

        if USOR.getValue() > 0.1:
            if d == True:
                forD += 1
                if forD > 100:
                    d = False
                speeds[0] = maxV
                speeds[1] = 0.65 * maxV
            else:
                speeds[0] = maxV
                speeds[1] = 0
        elif USOR.getValue() < 0.1:
            speeds[0] = 0.65 * maxV
            speeds[1] = maxV

        forLRD += 1
        if forLRD > 1000:
            LRD = not (LRD)
            forLRD = 0
            if LRD == True:
                print('R')
            else:
                print('L')
    else:

        if USOL.getValue() > 0.1:
            if d == True:
                forD += 1
                if forD > 100:
                    d = False
                speeds[0] = 0.65 * maxV
                speeds[1] = maxV
            else:
                speeds[0] = 0
                speeds[1] = maxV
        elif USOL.getValue() < 0.1:
            speeds[0] = maxV
            speeds[1] = 0.65 * maxV

        forLRD += 1
        if forLRD > 1000:
            forLRD = 0
            LRD = not (LRD)
            if LRD == True:
                print('R')
            else:
                print('L')
    if d == True:
        print('d = True ' + str(forD))

    if USFR2.getValue() < 0.15:
        rotL()

    if USFL2.getValue() < 0.15:
        rotR()

    if USR.getValue() < 0.15:
        slowRotL()

    if USL.getValue() < 0.15:
        slowRotR()

    if (USFL1.getValue() < 0.05) or (USFR1.getValue() < 0.05):
        rot()
        motorL.setVelocity(speeds[0])
        motorR.setVelocity(speeds[1])
        delay(randint(350, 650))

    if getColor() > 80:
        rot()
        motorL.setVelocity(speeds[0])
        motorR.setVelocity(speeds[1])
        delay(randint(500, 700))
        d = True
        forD = 0

    # print(np.frombuffer(CS.getImage(), np.uint8))
    '''if checkVicL(camL.getImage()):
        report('T')'''

    # print('--->'+str(camR.getImage())+'<---')
    # print('speeds:  '+str(speeds)+'   US:  '+str(USL.getValue())+' '+str(USFL2.getValue())+' '+str(USFL1.getValue())+' '+str(USF.getValue())+' '+str(USFR1.getValue())+' '+str(USFR2.getValue())+' '+str(USR.getValue())+'  color: '+str(getColor()))
    motorL.setVelocity(speeds[0])
    motorR.setVelocity(speeds[1])



