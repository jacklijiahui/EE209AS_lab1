# -*- coding: utf-8 -*-
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

import sympy as sym
import numpy as np
from sympy import *

"""""""""""""""""""""""""""define DHconvention"""""""""""""""""""""""""""
def DHconvention(alpha, a, theta0, d):
    T= np.zeros((4,4),dtype = sym.Function)
       
    cTheta = sym.cos(theta0*sym.pi/180.0)
    sTheta = sym.sin(theta0*sym.pi/180.0)
    cAlpha = sym.cos(alpha*sym.pi/180.0)
    sAlpha = sym.sin(alpha*sym.pi/180.0)
    
    T= np.zeros((4,4),dtype = sym.Function)
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
"""""""""""""""""""""""""""define DHconvention"""""""""""""""""""""""""""
    
"""origin state"""
alpha = [270,0,0,0]
a=[5.0,15.0,15.0,5.0]
d= [1,0,0,0] 
#theta=[-90.0,-90.0,0.0,-90.0]

"""Target position"""
Target = [0,15,16]


#constrain of theta
theta_limit=[[180.0,360.0],[180.0,360.0],[0,180.0],[180,360.0]] 

theta0 = sym.Symbol('theta0')  
theta1 = sym.Symbol('theta1')
theta2 = sym.Symbol('theta2')
theta3 = sym.Symbol('theta3')
theta=[theta0,theta1,theta2,theta3]

times=0
xNode=[]
yNode=[]
zNode=[]

    #T0 = DHconvention(alpha[0],a[0],theta0,d[0])
T1= np.eye(4,dtype = sym.Function)
for i in range(0,4):
    T1 = T1*DHconvention(alpha[i], a[i], theta[i], d[i])
    
Tp=np.mat([0,0,0,1]).T
end_effector = (T1*Tp)[0:3]

"""To find Jacobian"""


"""
Jacobian = [diff(Px,theta_1), diff(Px,theta_2), diff(Px,theta_3), diff(Px,theta_4);
            diff(Py,theta_1), diff(Py,theta_2), diff(Py,theta_3), diff(Py,theta_4);
            diff(Pz,theta_1), diff(Pz,theta_2), diff(Pz,theta_3), diff(Pz,theta_4)];
"""
Jacobian=np.zeros((3,4),dtype=sym.Function)
Jacobian[0]=[end_effector[0,0].diff(theta0),end_effector[0,0].diff(theta1),
             end_effector[0,0].diff(theta2),end_effector[0,0].diff(theta3)]

Jacobian[1]=[end_effector[1,0].diff(theta0),end_effector[1,0].diff(theta1),
             end_effector[1,0].diff(theta2),end_effector[1,0].diff(theta3)]

Jacobian[2]=[end_effector[2,0].diff(theta0),end_effector[2,0].diff(theta1),
             end_effector[2,0].diff(theta2),end_effector[2,0].diff(theta3)]
    

theta=[270.0,270.0,0.0,270.0]
distance=1; 
while(times<100 and distance>0.001 ):
    times +=1
    Jacobian_value =sym.lambdify((theta0,theta1,theta2,theta3),Matrix(Jacobian))   
    Jacobian_temp = Jacobian_value(theta[0],theta[1],theta[2],theta[3])
    
    end_effector_value = sym.lambdify((theta0,theta1,theta2,theta3),Matrix(end_effector))
    end_effector_temp=end_effector_value(theta[0],theta[1],theta[2],theta[3])        
    
    Jacobian_in =np.linalg.pinv(Jacobian_temp)
    
    err_matrix = [[Target[0]-end_effector_temp[0][0]],
                   [Target[1]-end_effector_temp[1][0]],
                    [Target[2]-end_effector_temp[2][0]]]
                    
    delta_theta=np.dot(Jacobian_in,err_matrix)
    delta_theta=np.array(delta_theta).tolist()
    for i in range(0,4):
            temp = (1*delta_theta[i][0])
            theta[i] = (theta[i] +temp)%360
            if(theta[i]<theta_limit[i][0]):
                theta[i] = theta_limit[i][0]
                
            if(theta[i]>theta_limit[i][1]):
                theta[i] = theta_limit[i][1]
    distance = np.sqrt((Target[0]-end_effector_temp[0][0])**2+
            (Target[1]-end_effector_temp[1][0])**2 +
            (Target[2]-end_effector_temp[2][0])**2)
    xNode.append(np.round(end_effector_temp[0][0],3))
    yNode.append(np.round(end_effector_temp[1][0],3))
    zNode.append(np.round(end_effector_temp[2][0],3))

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(xNode,yNode,zNode)               
#ax.scatter(xNode,yNode,zNode) 
    
            
    
