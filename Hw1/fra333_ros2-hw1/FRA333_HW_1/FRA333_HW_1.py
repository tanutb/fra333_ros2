#!/usr/bin/python3
from trackbeebot import BeeBot
import matplotlib.pyplot as plt 
from matplotlib.patches import Polygon
import numpy as np

class MyBeeBot(BeeBot):
    def __init__(self,a_i):                                           # Constructor
        super().__init__(a_i)                                       # inherit all the methods จาก Beebot:
        self.init_pos = a_i                                         # set init_pos จาก a_i                      
    # a_i is initial position
    # c is command {'0'->stop, '1'->forward, '2'->backward, '3'->turn right, '4'->turn left}
    # o is obstacle

    ## สร้าง function template homogeneous matrix transformation 3x3
    # [ cos(θ), -sin(θ)  , x ]
    # [ sin(θ),  cos(θ)  , y ]
    # [      0,      0   , 1 ]
    def tranfrom_matrix(self,theta,pos=[0,0]):
        return np.array([[np.math.cos(theta),-1*np.math.sin(theta),pos[0]],
                     [np.math.sin(theta),np.math.cos(theta),pos[1]],
                     [0,0,1]],dtype = np.float64)

    ## สร้าง function pos to index
    def pos2idx(self,x,y):
        a = np.array([[np.sqrt(3)/2 , 3/2],[-np.sqrt(3)/2,3/2]])*2/(3*np.sqrt(3))
        return np.matmul(a, np.array([x,y]))

    # def idx2pos(self,m,n):
    #     a = np.array([[3/2,-3/2,],[np.sqrt(3)/2,np.sqrt(3)/2]])
    #     return np.matmul(a, np.array([m,n]))
    
    def trackBeeBot(self, com, W):
        ## เปลี่ยนจาก index จากเฟรม local เป็น pos เฟรม Global และ สร้าง initial matrix posisiton
        # [ cos(θ), -sin(θ)  , x_global ]
        # [ sin(θ),  cos(θ)  , y_global ]
        # [      0,      0   , 1 ]
        init_xy = self.tranfrom_matrix(0,self.idx2pos(self.init_pos[0],self.init_pos[1]))   
        ## สร้าง list ของ index และ set initial position 
        ab = [[self.init_pos[0],self.init_pos[1]]]             
        ## เปลี่ยนรูปแบบของ Wall เป็น [x,y]                              
        wall = list(map(lambda x,y : [x,y],W[0],W[1]))
        ## สร้าง list ของ homogeneous matrix ตามค่าที่ได้จาก command 
        # arr =
        # 0 (อยู่กับที่ )  :  # [ cos(0), -sin(0)  , 0 ]  
                         # [ sin(0),  cos(0)  , 0 ]
                         # [      0,      0   , 1 ]
        # 1 (เดินหน้า)  :  # [ cos(0), -sin(0)  , 0 ]  
                         # [ sin(0),  cos(0)  , sqrt(3)]  #เคลี่ยนที่ทางแกน Y     
                         # [      0,      0   , 1 ]
        # 2 (ถอยหลัง)  :  # [ cos(0), -sin(0)  , 0 ]  
                         # [ sin(0),  cos(0)  , -sqrt(3) ]
                         # [      0,      0   , 1 ]
        # 3 (เลี้ยวขวา)  :  # [ cos(60), -sin(60)  , 0 ]  
                         # [ sin(60),  cos(60)  , 0 ]
                         # [      0,      0     , 1 ]
        # 4 (เลี้ยวซ้าย)  :  # [ cos(-60), -sin(-60) , 0 ]  
                         # [ sin(-60),  cos(-60)  , 0 ]
                         # [      0,      0       , 1 ]                
        arr = [self.tranfrom_matrix(0),self.tranfrom_matrix(0,[0,np.math.sqrt(3)])
               ,self.tranfrom_matrix(0,[0,-1*np.math.sqrt(3)]),self.tranfrom_matrix(1*np.math.pi/3,[0,0])
               ,self.tranfrom_matrix(-1*np.math.pi/3)]
        for i in com:   #Loop command
            #ทำ่การคูณ matrix โดยเรียกใช้ matrix ตาม command 
            # [ cos(θ), -sin(θ)  , x_new]       [ cos(θ), -sin(θ)  , x_current ]       
            # [ sin(θ),  cos(θ)  , y_new]   =   [ sin(θ),  cos(θ)  , y_current ]  *  arr[i]
            # [      0,      0   , 1 ]          [      0,      0   ,       1   ]
            temp = np.matmul(init_xy,arr[int(i)])
            # เปลี่ยนจากพิกัดเฟรม local เป็น Global 
            # โดยใช้ function pos2idx และ ทำการ round และเปลี่ยนเป็น int จะได้พิกัด [a,b] แกน local
            # สร้าง ตัวแปรชั่วคราว(temp_index) มาเก็บค่า ตำแหน่ง
            temp_index = self.pos2idx((temp[0][2]),(temp[1][2]))
            temp_index = [int(round(temp_index[0])),int(round(temp_index[1]))]
            if(temp_index not in wall):                                     # check ว่า พิกัดนั้น ต้อง ไม่อยู่ใน wall
                init_xy = np.matmul(init_xy,arr[int(i)])                    # update ค่าเข้า ตัวแปร init_xy 
                if int(i) in [1,2]:                                         # check command 1,2
                    ab.append(temp_index)                                   # append เก็บเส้นทางการเดินของแกน Local
        xy = list(map(lambda x : self.idx2pos(x[0],x[1]),ab))               # เปลี่ยน list เส้นทางการเดินของ พิกัดแกน local เป็น Global
        return np.array(ab).T,np.array(xy).T                                # return เป็น numpy array และ transpose


    # a_i is initial position
    # com is command {'0'->stop, '1'->forward, '2'->backward, '3'->turn right, '4'->turn left}
    # W is wall
    
# #     # for test
# if __name__=='__main__':
#     a = MyBeeBot([-4, -1])
#     ans = a.trackBeeBot("10004031304133412113203401202434122434340313312420340143230302444311410144134242324403414413032441433",
#     [[-1, 0, -2, 0, -4, 2, 5, -2, 2, 1, 4, 2, -2, -4], [1, 0, -1, 5, 2, 2, 1, -4, -5, 1, -5, -1, 4, 0]])
#     print(ans)
#     p = [[-4, -3, -2, -1, -1, -1, -1, -1, 0, -1, 0, 1, 1, 1, 1, 1, 0, 1, 2, 1, 2, 2, 1, 1, 1, 2, 1, 1, 0, -1, -1, 0, 0], [-1, 0, 1, 2, 3, 2, 3, 4, 4, 4, 4, 4, 5, 4, 3, 4, 3, 4, 4, 4, 4, 5, 5, 6, 7, 7, 7, 8, 8, 7, 8, 9, 10]]
#     if(p == ans):
#         print("true")