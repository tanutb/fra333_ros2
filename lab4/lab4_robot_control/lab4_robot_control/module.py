import numpy as np
import math

h = 0.135
l1 = 0.04
l2 = 0.15
l3 = 0.12
DH_parameter = np.array([
    [0      ,0          ,h              ,0      ],
    [l1     ,np.pi/2    ,0              ,0      ],  
    [l2     ,0          ,0              ,0      ]] )

def FPK(q):
    # radian input
    H = np.eye(4)
    H_arr = []
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
        H_arr.append(H)
           
    H_en = np.array([
        [0,0,1,l3],
        [1,0,0,0],
        [0,1,0,0],
        [0,0,0,1]
    ])    
    H_arr.append(H @ H_en)
    return H_arr


def IPK(gamma1,gamma2,x,y,z,flag=False):
    q = [0.,0.,0.]
    a = gamma1 * ((x**2) + (y**2))**0.5 - l1
    b = z-h
    if ((l2-l3) <= (a**2 + b **2)**0.5) and ((l2+l3) >= (a**2 + b **2)**0.5):
        q[0] =  math.atan2(y/gamma1,x/gamma1) 
        c3 = (a**2 + b **2 - l2**2 - l3**2)/(2*l2*l3)
        c3 = round(c3,15)
        s3 = gamma2 * np.sqrt((1-(c3**2)))
        q[1] = math.atan2(b,a) - math.atan2(l3*s3,l2+l3*c3)
        q[2] = math.atan2(s3,c3)
        flag = True
    return q,flag



def Jacobian(q):
    z = np.array([0,0,1]).T
    H = FPK(q)
    
    z0 , z1 , z2 = (H[0][:3,:3] @ z) , (H[1][:3,:3] @ z)  , (H[2][:3,:3] @ z)
    
    
    i_11,i_21,i_31 = z0
    i_12,i_22,i_32 = z1
    i_13,i_23,i_33 = z2
    
    p_e = H[-1][:3,3]
    # p_e = H[-1][:3,3]
    i_41,i_51,i_61 = np.cross(z0,p_e-H[0][:3,3]) 
    i_42,i_52,i_62 = np.cross(z1,p_e-H[1][:3,3]) 
    i_43,i_53,i_63 = np.cross(z2,p_e-H[2][:3,3]) 
    
    return [ [i_11, i_12, i_13],
          [i_21, i_22, i_23],
          [i_31, i_32, i_33],
          [i_41, i_42, i_43],
          [i_51, i_52, i_53],
          [i_61, i_62, i_63] ]
    
def checkSingularity(q):
    J = Jacobian(q)[3::]
    ## abs check 
    if abs(np.linalg.det(J)) < 0.001:
        return True
    return False  
    
    
def IVK(q,p_dot):
    J = np.array(Jacobian(q)[3::])
    J_inv = np.linalg.inv(J)
    return J_inv @ p_dot



if __name__ == "__main__":
    a = Jacobian([0,0,0])
    print(np.array(a))
    