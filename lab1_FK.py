# -*- coding: utf-8 -*-
"""
Created on Thu Oct  6 22:16:53 2016

@author: lijiahui

parameter 
* alpha   : angle between axes zi−1 and zi about axis xi to be taken 
                    positive when rotation is made counter-clockwise
*  a       : distance between O_i and O_iprime
*  d   : coordinate of Oi′ along zi−1,
*  theta       : angle between axes xi−1 and xi about axis zi−1 to be taken 
                    positive when rotation is made counter-clockwise.

"""

import numpy as np


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

    
alpha = [-90,0,0,0]
a=[5.0,15.0,15.0,5.0]
theta=[-90,-180,90,-90]  
d= [1,0,0,0] 

T1 = np.eye(4)
T1 = np.mat(T1)

for i in range(0,4):
    T1 = T1*DHconvention(alpha[i], a[i], theta[i], d[i])
#T1 = np.mat(T1)
#T2 =T1.I

#print T1

Te =np.eye(4,dtype = np.float)
Te[0][3]=0;
Te[1][3]=0;
Te[2][3]=0
Te = np.mat(Te)

print T1.round(5)

"""""""""""""""""""""""""""plot trajectory"""""""""""""""""""""""""""
from matplotlib import pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

"""
theta1 [-180,180]
theta2 [0,-180]
theta3 [0,180]
theta4 [0,-180]
"""
Tx=[]
Ty=[]
Tz=[]

alpha = [-90,0,0,0]
a=[5.0,15.0,15.0,5.0]
theta=[-90,-90,0,-90]  
d= [1,0,0,0] 

Ttotal = np.eye(4)
Ttotal = np.mat(Ttotal)
"""
#for i1 in range(-180,181,1):
for i2 in range(-180,10,5):
    for i3 in range(0,190,5):
        for i4 in range(-180,10,5):
            theta = [-90,i2,i3,i4]
            for j in range(4):
                Ttotal = Ttotal*DHconvention(alpha[j], a[j], theta[j], d[j])
            Ty.append(round(np.array(Ttotal)[1][3],2))
            Tz.append(round(np.array(Ttotal)[2][3],2))
            Ttotal = np.eye(4)
            Ttotal = np.mat(Ttotal)
            
"""
plt.scatter(Ty,Tz)

for i1 in range(-180,190,10):
    for i2 in range(-180,10,10):
        for i3 in range(0,190,10):
            for i4 in range(-180,10,10):
                theta = [i1,i2,i3,i4]
                for j in range(4):
                    Ttotal = Ttotal*DHconvention(alpha[j], a[j], theta[j], d[j])
                Tx.append(round(np.array(Ttotal)[0][3],2))
                Ty.append(round(np.array(Ttotal)[1][3],2))
                Tz.append(round(np.array(Ttotal)[2][3],2))
                Ttotal = np.eye(4)
                Ttotal = np.mat(Ttotal)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
               
ax.scatter(Tx,Ty,Tz)