#!/usr/bin/python3
import json 
import numpy as np
from json.decoder import JSONDecodeError
import math


file_json = 'data.json'

def check_flag(data):
    for i in data.keys():
        if len(data[i]) != 1000:
            return False
        
    return True

def read_json():
    
    try:
        with open(file_json, 'r') as f:
            data = json.load(f)
    except JSONDecodeError:
        return {"angular_velocity":[],"linear_acceleration":[]}  
    
    return data
    
def write_json(data):
    # Data to be written
    with open(file_json, "w") as outfile:
        json.dump(data, outfile)

def calibration_IMU(data):
    sd_arr = []
    for i in data.keys():
        for j in range(3):
            m = np.array(data[i])[:,j]
            a = np.sqrt(sum(m**2)/999)
            sd_arr.append(a)
    return sd_arr



h = 0.135
l1 = 0.04
l2 = 0.15
l3 = 0.12
DH_parameter = np.array([
    [0      ,0          ,h              ,0      ],
    [l1     ,np.pi/2    ,0              ,0      ],  
    [l2     ,0          ,0              ,0      ]] )

def forki(q):
    # radian input
    H = np.eye(4)
    for i in range(len(DH_parameter)):
        Tx = np.array([
            [1,0,0,DH_parameter[i][0]],
            [0,1,0,0],
            [0,0,1,0],
            [0,0,0,1]
        ])
        theta_x = DH_parameter[i][1]
        Rx = np.array([
            [1,0,0,0],
            [0,np.cos(theta_x),-np.sin(theta_x),0],
            [0,np.sin(theta_x),np.cos(theta_x),0],
            [0,0,0,1]
        ])
        Tz = np.array([
            [1,0,0,0],
            [0,1,0,0],
            [0,0,1,DH_parameter[i][2]],
            [0,0,0,1]
        ])
        theta_z = DH_parameter[i][3]
        Rz = np.array([
            [np.cos(theta_z),-np.sin(theta_z),0,0],
            [np.sin(theta_z),np.cos(theta_z),0,0],
            [0,0,1,0],
            [0,0,0,1]
        ]) 
        H = H@Tx@Rx@Tz@Rz

        #joint update
        Rz_joint = np.array([
            [np.cos(q[i]),-np.sin(q[i]),0,0],
            [np.sin(q[i]),np.cos(q[i]),0,0],
            [0,0,1,0],
            [0,0,0,1]
        ])
        H = H @ Rz_joint
           
    H_en = np.array([
        [0,0,1,l3],
        [1,0,0,0],
        [0,1,0,0],
        [0,0,0,1]
    ])    
    return H @ H_en


def invki(gamma1,gamma2,x,y,z,flag=False):
    q = [0.,0.,0.]
    a = gamma1 * ((x**2) + (y**2))**0.5 - l1
    b = z-h
    if ((l2-l3) <= (a**2 + b **2)**0.5) and ((l2+l3) >= (a**2 + b **2)**0.5):
        q[0] =  math.atan2(y/gamma1,x/gamma1) 
        c3 = (a**2 + b **2 - l2**2 - l3**2)/(2*l2*l3)
        # c3 = round(c3,15)
        s3 = gamma2 * np.sqrt((1-(c3**2)))
        q[1] = math.atan2(b,a) - math.atan2(l3*s3,l2+l3*c3)
        q[2] = math.atan2(s3,c3)
        flag = True
    return q,flag



# a = {"a" : [[1,2,-4],[1,2,-3],[1,2,-3],[1,2,-3]],"b" : [[1,2,4],[1,2,3],[1,2,3],[1,2,-3]]}

# # print(len([]))
# print(np.array([1.2,4.6]).astype(int))




