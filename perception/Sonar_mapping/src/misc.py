#!/usr/bin/env python
import numpy as np
import math

# Function that takes in two points, and returns all pixel between them
def getLineBetweenPoints(r0,c0,r1,c1):
    # There are multiple cases that must be handled, depending on where the non-static point is

    if abs(c1-c0) < abs(r1-r0):
        # If the rows spawns a bigger area than the columns
        xx,yy,val = getLineBetweenPoints(c0,r0,c1,r1)
        return(yy,xx,val)
        
    if c0>c1:
        # If the static point is to the left of the point of intrest
        return getLineBetweenPoints(r1,c1,r0,c0)
    
    # Declaring y as a function of x
    x = np.arange(c0,c1+1,dtype=float)
    y = x*(r1-r0) / (c1-c0) + (c1*r0-c0*r1)/(c1-c0)
    valbot = np.floor(y)-y+1
    valtop = y-np.floor(y)
    
    return (np.concatenate((np.floor(y), np.floor(y)+1)).astype(int), np.concatenate((x,x)).astype(int),
            np.concatenate((valbot, valtop)))
    

def rotatePointAroundCenter(point,center,yaw):
    #if yaw < 0:
    c,s = np.cos(yaw),np.sin(yaw)
    rotMatrix = np.array([[c,-s],[s,c]])
    return rotMatrix.dot(point-center)+center
    #else:
    #    c,s = np.cos(yaw),np.sin(yaw)
    #    rotMatrix = np.array([[c,s],[-s,c]])
    #    return rotMatrix.dot(point-center)+center


def quaternion_to_euler(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]
