# -*- coding: utf-8 -*-
"""
Created on Sun Oct  9 13:11:23 2016

@author: lijiahui
"""

"""
* Calculates and sets the current value of the DHT matrix.
 * 
 * DHT Matrix:
 * 
 *  [   cos(theta)  -sin(theta)cos(alpha)     sin(theta)sin(alpha)      a*cos(theta)    ]
 *  [   sin(theta)   cos(theta)cos(alpha)    -cos(theta)sin(alpha)      a*sin(theta)    ]
 *  [     0             sin(alpha)                  cos(alpha)              d           ]
 *  [     0                 0                           0                   1           ]
 * 
"""

import numpy as np
"""""""""""""""""""""""""""define DHconvention"""""""""""""""""""""""""""
def DHconvention(alpha, a, theta, d):
    T= np.zeros((4,4),dtype = np.float)
    cTheta = np.cos(theta*np.pi/180.0)
    sTheta = np.sin(theta*np.pi/180.0)
    cAlpha = np.cos(alpha*np.pi/180.0)
    sAlpha = np.sin(alpha*np.pi/180.0)

#first line    
    T[0][0]=cTheta;
    T[0][1]=-sTheta*cAlpha;
    T[0][2]=sTheta*sAlpha;
    T[0][3]=a*cTheta;
#second line     
    T[1][0]=sTheta;
    T[1][1]=cTheta*cAlpha;
    T[1][2]=-cTheta*sAlpha;
    T[1][3]=a*sTheta;
#third line   
    T[2][1]=sAlpha;
    T[2][2]=cAlpha;
    T[2][3]=d;
#forth line    
    T[3][3]=1;
    T=np.mat(T)
    return T;
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

"""
parameter 
* alpha   : angle between axes zi−1 and zi about axis xi to be taken 
    positive when rotation is made counter-clockwise
*  a       : distance between O_i and O_iprime
*  d   : coordinate of Oi′ along zi−1,
*  theta       : angle between axes xi−1 and xi about axis zi−1 to be taken 
                    positive when rotation is made counter-clockwise.

"""

   
alpha = [-90,0,0,0]
a=[5.0,15.0,15.0,5.0]
d= [1,0,0,0] 

"""origin state"""
theta=[-90.0,-90.0,0.0,-90.0]  

#constrain of theta
theta_limit=[[-180.0,180.0],[-180,0.0],[0,180.0],[-180,0.0]]

T1 = np.eye(4)
T1 = np.mat(T1)

for i in range(0,4):
    T1 = T1*DHconvention(alpha[i], a[i], theta[i], d[i])

Tp=np.mat([0,0,0,1]).T
end_effector = (T1.round(5)*Tp)[0:3]
print end_effector

"""Target position"""
Target = [0,15,16]
print Target

distance = np.sqrt((Target[0]-end_effector.tolist()[0][0])**2+
(Target[1]-end_effector.tolist()[1][0])**2 +
(Target[2]-end_effector.tolist()[2][0])**2)

times=0
xNode=[]
yNode=[]
zNode=[]
while (times<200 and distance >0.5):
    times+=1    
    step=1
    T = np.eye(4)
    T = np.mat(T)
    Tpi = {}
    # to get the T
    for i in range(0,4):
        Tpi[i] = T*Tp
        T = T*DHconvention(alpha[i], a[i], theta[i], d[i])
    Tpi[4]=T*Tp   
    
    
    Tp=np.mat([0,0,0,1]).T
    end_effector = (T.round(5)*Tp)[0:3]
    #to get the first three line of Jacobian
    Jacobian = np.zeros((6,4),dtype=float )
    for i in range(0,4):
        for j in range(0,3):
            Jacobian[j][i]=(Tpi[4][j]-Tpi[i][j])
            #Jacobian[j][i]=Tpi[i][j]
    Jacobian[3]=Jacobian[2]
    Jacobian[2]=Jacobian[1]
    Jacobian[1]=Jacobian[3]   
    Jacobian[3]=1  #The last three line
   
    Jacobian=np.mat(Jacobian[0:3])
    
    #To get the Pseudo inverse Jacobian
    Jacobian_in =np.linalg.pinv(Jacobian)
    err_matrix = [[Target[0]-end_effector.tolist()[0][0]],
                   [Target[1]-end_effector.tolist()[1][0]],
                    [Target[2]-end_effector.tolist()[2][0]]]
                    
    distance = np.sqrt((Target[0]-end_effector.tolist()[0][0])**2+
        (Target[1]-end_effector.tolist()[1][0])**2 +
        (Target[2]-end_effector.tolist()[2][0])**2)
       
    '''J_inverse * ve'''
    delta_theta=Jacobian_in*err_matrix
    delta_theta=np.array(delta_theta).tolist()
   
    for i in range(0,4):
        temp = step*delta_theta[i][0]
        theta[i] = theta[i] +temp
        if(theta[i]<theta_limit[i][0]):
            theta[i] = theta_limit[i][0]
            
        if(theta[i]>theta_limit[i][1]):
            theta[i] = theta_limit[i][1]
            
    xNode.append(end_effector.tolist()[0][0])
    yNode.append(end_effector.tolist()[1][0])
    zNode.append(end_effector.tolist()[2][0])

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
               
ax.scatter(xNode,yNode,zNode) 
            
       
  

    