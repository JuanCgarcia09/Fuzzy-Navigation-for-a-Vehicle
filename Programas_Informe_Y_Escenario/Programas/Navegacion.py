# -*- coding: utf-8 -*-
"""
Created on Tue Mar  1 19:11:38 2022

@author: juanc
"""

import sys
import sim as vrep # access all the VREP elements
#import vrep
import time
import numpy as np
import matplotlib.pyplot as plt
import math

def gauss(u,center,std):
    return np.exp(-((u-center)**2)/(2*std**2) ) 


UG=np.arange(-360,360,0.1)
UO=np.arange(-360,1000,0.2)
US=np.arange(0,1.2,0.1)

uGA=gauss(UG,0,45)+gauss(UG,360,45)+gauss(UG,-360,45)
uGAD=gauss(UG,30,15)+gauss(UG,-330,15)
uGAI=gauss(UG,-30,15)+gauss(UG,330,15)
uGD=gauss(UG,90,90)+gauss(UG,-270,90)
uGI=gauss(UG,-90,90)+gauss(UG,270,90)
uGR=gauss(UG,180,60)+gauss(UG,-180,60)
"""
plt.figure(0)
plt.plot(UG,uGA)
plt.title("Conjunto difuso PG Adelante")
plt.xlabel('Ángulo de la meta, respecto a la orientación del vehículo')
plt.grid()
"""



uOA=gauss(UO,0,45)+gauss(UO,360,45)+gauss(UO,-360,45)
uOAD=gauss(UO,45,45)+gauss(UO,-315,45)
uOAI=gauss(UO,-45,45)+gauss(UO,315,45)
uOD=gauss(UO,90,90)+gauss(UO,-270,90)
uOI=gauss(UO,-90,90)+gauss(UO,270,90)
uOR=gauss(UO,180,90)+gauss(UO,-180,90)
uON=gauss(UO,450,50)

MIL=gauss(US,0,0.1)
MIM=gauss(US,0.4,0.2)
MIR=gauss(US,1.1,0.2)
MDL=gauss(US,0,0.1)
MDM=gauss(US,0.4,0.2)
MDR=gauss(US,1.1,0.2)

plt.figure(0)
plt.plot(US,MIR)
plt.title("Conjunto difuso velocidades motor")
plt.plot(US,MIM)
plt.plot(US,MIL)
plt.legend(['Velocidad Rápida', 'Velocidad Media', 'Velocidad Lenta'],fontsize='x-large',loc='lower left')
plt.grid()

vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # start aconnection
if clientID!=-1:
    print ("Connected to remote API server")
else:
    print("Not connected to remote API server")
    sys.exit("Could not connect")
    


# handlers Definition
errorCode, left_motor_handle =vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_oneshot_wait)
errorCode, right_motor_handle =vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_oneshot_wait)
errorCode, pioneer_handle =vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_visible',vrep.simx_opmode_oneshot_wait)
errorCode, goal_handle =vrep.simxGetObjectHandle(clientID,'Goal',vrep.simx_opmode_oneshot_wait)
errorCode, sensor_handle =vrep.simxGetObjectHandle(clientID,'Proximity_sensor',vrep.simx_opmode_oneshot_wait)


#returnCode=vrep.simxSetJointTargetVelocity(number clientID,number jointHandle,number targetVelocity,number operationMode)

errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,0, vrep.simx_opmode_streaming)
errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,0, vrep.simx_opmode_streaming)
#R=np.zeros((42,60))
#R=np.zeros(42)
RMI=np.zeros((42,12))
RMD=np.zeros((42,12))




errorCode, PosRobot  =vrep.simxGetObjectPosition(clientID,pioneer_handle,-1,vrep.simx_opmode_blocking)


errorCode, PosGoal  =vrep.simxGetObjectPosition(clientID,goal_handle,-1,vrep.simx_opmode_blocking)
print('Robot Position  ',PosRobot)    
print('Goal Position  ',PosGoal)


return_code, detection_state, detected_point, detected_object_handle, detected_surface_normal_vector = vrep.simxReadProximitySensor(clientID,  sensor_handle , vrep.simx_opmode_oneshot_wait)



Timbrebipbip=True
print('Sensor')
while Timbrebipbip: 
    return_code, detection_state, detected_point, detected_object_handle, detected_surface_normal_vector = vrep.simxReadProximitySensor(clientID,  sensor_handle , vrep.simx_opmode_oneshot_wait)
    
    errorCode,OriRobot=vrep.simxGetObjectOrientation(clientID,pioneer_handle,-1,vrep.simx_opmode_blocking)
    errorCode, PosRobot  =vrep.simxGetObjectPosition(clientID,pioneer_handle,-1,vrep.simx_opmode_blocking)
    errorCode, PosGoal  =vrep.simxGetObjectPosition(clientID,goal_handle,-1,vrep.simx_opmode_blocking)
    errorCode, PosObj  =vrep.simxGetObjectPosition(clientID,detected_object_handle,-1,vrep.simx_opmode_blocking)
    anguloobs=0
    angulogoal=math.degrees(math.atan((PosGoal[1]-PosRobot[1])/(PosGoal[0]-PosRobot[0])))
    if(detection_state==True):
        anguloobs=math.degrees(math.atan((PosObj[1]-PosRobot[1])/(PosObj[0]-PosRobot[0])))
        
        if(PosObj[0]<PosRobot[0] and PosObj[1]<PosRobot[1]):
            anguloobs=anguloobs-180
        elif(PosObj[0]<PosRobot[0] and PosObj[1]>PosRobot[1]):
            anguloobs=anguloobs+180
        
    if(PosGoal[0]<PosRobot[0] and PosGoal[1]<PosRobot[1]):
        angulogoal=angulogoal-180
        a=2
    elif(PosGoal[0]<PosRobot[0] and PosGoal[1]>PosRobot[1]):
        angulogoal=angulogoal+180
        b=3
    OriRobot=math.degrees(OriRobot[2])
    
    DirGoal=OriRobot-angulogoal
    DirObs=OriRobot-anguloobs
    if(detection_state==False):
        DirObs=950
    
    uGADirGoal=gauss(DirGoal,0,25)+gauss(DirGoal,360,25)+gauss(DirGoal,-360,25)
    uGADDirGoal=gauss(DirGoal,30,15)+gauss(DirGoal,-330,15)
    uGAIDirGoal=gauss(DirGoal,-30,15)+gauss(DirGoal,330,15)
    uGDDirGoal=gauss(DirGoal,90,45)+gauss(DirGoal,-270,45)
    uGIDirGoal=gauss(DirGoal,-90,45)+gauss(DirGoal,270,45)
    uGRDirGoal=gauss(DirGoal,180,45)+gauss(DirGoal,-180,45)
    
    uOADirObs=gauss(DirObs,0,25)+gauss(DirObs,360,25)+gauss(DirObs,-360,25)
    uOADDirObs=gauss(DirObs,30,15)+gauss(DirObs,-330,15)
    uOAIDirObs=gauss(DirObs,-30,15)+gauss(DirObs,330,15)
    uODDirObs=gauss(DirObs,90,45)+gauss(DirObs,-270,45)
    uOIDirObs=gauss(DirObs,-90,45)+gauss(DirObs,270,45)
    uORDirObs=gauss(DirObs,180,45)+gauss(DirObs,-180,45)
    uONDirObs=gauss(DirObs,950,50)
    
    RMI[0]= np.minimum(MIL,np.minimum(uGADirGoal,uOADirObs))
    RMI[1]= np.minimum(MIR,np.minimum(uGADDirGoal,uOADirObs))
    RMI[2]= np.minimum(MIM,np.minimum(uGAIDirGoal,uOADirObs))
    RMI[3]= np.minimum(MIR,np.minimum(uGDDirGoal,uOADirObs))
    RMI[4]= np.minimum(MIL,np.minimum(uGIDirGoal,uOADirObs))
    RMI[5]= np.minimum(MIR,np.minimum(uGRDirGoal,uOADirObs))
    
    RMI[6]= np.minimum(MIM,np.minimum(uGADirGoal,uOADDirObs))
    RMI[7]= np.minimum(MIR,np.minimum(uGADDirGoal,uOADDirObs))
    RMI[8]= np.minimum(MIM,np.minimum(uGAIDirGoal,uOADDirObs))
    RMI[9]= np.minimum(MIR,np.minimum(uGDDirGoal,uOADDirObs))
    RMI[10]= np.minimum(MIL,np.minimum(uGIDirGoal,uOADDirObs))
    RMI[11]= np.minimum(MIR,np.minimum(uGRDirGoal,uOADDirObs))
    
    RMI[12]= np.minimum(MIR,np.minimum(uGADirGoal,uOAIDirObs))
    RMI[13]= np.minimum(MIR,np.minimum(uGADDirGoal,uOAIDirObs))
    RMI[14]= np.minimum(MIR,np.minimum(uGAIDirGoal,uOAIDirObs))
    RMI[15]= np.minimum(MIR,np.minimum(uGDDirGoal,uOAIDirObs))
    RMI[16]= np.minimum(MIL,np.minimum(uGIDirGoal,uOAIDirObs))
    RMI[17]= np.minimum(MIR,np.minimum(uGRDirGoal,uOAIDirObs))
    
    RMI[18]= np.minimum(MIR,np.minimum(uGADirGoal,uODDirObs))
    RMI[19]= np.minimum(MIM,np.minimum(uGADDirGoal,uODDirObs))
    RMI[20]= np.minimum(MIM,np.minimum(uGAIDirGoal,uODDirObs))
    RMI[21]= np.minimum(MIR,np.minimum(uGDDirGoal,uODDirObs))
    RMI[22]= np.minimum(MIL,np.minimum(uGIDirGoal,uODDirObs))
    RMI[23]= np.minimum(MIL,np.minimum(uGRDirGoal,uODDirObs))
    
    RMI[24]= np.minimum(MIR,np.minimum(uGADirGoal,uOIDirObs))
    RMI[25]= np.minimum(MIR,np.minimum(uGADDirGoal,uOIDirObs))
    RMI[26]= np.minimum(MIM,np.minimum(uGAIDirGoal,uOIDirObs))
    RMI[27]= np.minimum(MIR,np.minimum(uGDDirGoal,uOIDirObs))
    RMI[28]= np.minimum(MIM,np.minimum(uGIDirGoal,uOIDirObs))
    RMI[29]= np.minimum(MIR,np.minimum(uGRDirGoal,uOIDirObs))
    
    RMI[30]= np.minimum(MIR,np.minimum(uGADirGoal,uORDirObs))
    RMI[31]= np.minimum(MIR,np.minimum(uGADDirGoal,uORDirObs))
    RMI[32]= np.minimum(MIM,np.minimum(uGAIDirGoal,uORDirObs))
    RMI[33]= np.minimum(MIR,np.minimum(uGDDirGoal,uORDirObs))
    RMI[34]= np.minimum(MIL,np.minimum(uGIDirGoal,uORDirObs))
    RMI[35]= np.minimum(MIR,np.minimum(uGRDirGoal,uORDirObs))
    
    RMI[36] = np.minimum(MIR,np.minimum(uGADirGoal,uONDirObs))
    RMI[37]= np.minimum(MIR,np.minimum(uGADDirGoal,uONDirObs))
    RMI[38]= np.minimum(MIM,np.minimum(uGAIDirGoal,uONDirObs))
    RMI[39]= np.minimum(MIR,np.minimum(uGDDirGoal,uONDirObs))
    RMI[40]= np.minimum(MIL,np.minimum(uGIDirGoal,uONDirObs))
    RMI[41]= np.minimum(MIR,np.minimum(uGRDirGoal,uONDirObs))
    
    RMD[0]= np.minimum(MDR,np.minimum(uGADirGoal,uOADirObs))
    RMD[1]= np.minimum(MDM,np.minimum(uGADDirGoal,uOADirObs))
    RMD[2]= np.minimum(MDR,np.minimum(uGAIDirGoal,uOADirObs))
    RMD[3]= np.minimum(MDL,np.minimum(uGDDirGoal,uOADirObs))
    RMD[4]= np.minimum(MDR,np.minimum(uGIDirGoal,uOADirObs))
    RMD[5]= np.minimum(MDL,np.minimum(uGRDirGoal,uOADirObs))
    
    RMD[6]= np.minimum(MDR,np.minimum(uGADirGoal,uOADDirObs))
    RMD[7]= np.minimum(MDR,np.minimum(uGADDirGoal,uOADDirObs))
    RMD[8]= np.minimum(MDR,np.minimum(uGAIDirGoal,uOADDirObs))
    RMD[9]= np.minimum(MDL,np.minimum(uGDDirGoal,uOADDirObs))
    RMD[10]= np.minimum(MDR,np.minimum(uGIDirGoal,uOADDirObs))
    RMD[11]= np.minimum(MDL,np.minimum(uGRDirGoal,uOADDirObs))
    
    RMD[12]= np.minimum(MDR,np.minimum(uGADirGoal,uOAIDirObs))
    RMD[13]= np.minimum(MDM,np.minimum(uGADDirGoal,uOAIDirObs))
    RMD[14]= np.minimum(MDR,np.minimum(uGAIDirGoal,uOAIDirObs))
    RMD[15]= np.minimum(MDL,np.minimum(uGDDirGoal,uOAIDirObs))
    RMD[16]= np.minimum(MDR,np.minimum(uGIDirGoal,uOAIDirObs))
    RMD[17]= np.minimum(MDL,np.minimum(uGRDirGoal,uOAIDirObs))
        
    RMD[18]= np.minimum(MDR,np.minimum(uGADirGoal,uODDirObs))
    RMD[19]= np.minimum(MDR,np.minimum(uGADDirGoal,uODDirObs))
    RMD[20]= np.minimum(MDR,np.minimum(uGAIDirGoal,uODDirObs))
    RMD[21]= np.minimum(MDM,np.minimum(uGDDirGoal,uODDirObs))
    RMD[22]= np.minimum(MDR,np.minimum(uGIDirGoal,uODDirObs))
    RMD[23]= np.minimum(MDR,np.minimum(uGRDirGoal,uODDirObs))
        
    RMD[24]= np.minimum(MDR,np.minimum(uGADirGoal,uOIDirObs))
    RMD[25]= np.minimum(MDM,np.minimum(uGADDirGoal,uOIDirObs))
    RMD[26]= np.minimum(MDR,np.minimum(uGAIDirGoal,uOIDirObs))
    RMD[27]= np.minimum(MDL,np.minimum(uGDDirGoal,uOIDirObs))
    RMD[28]= np.minimum(MDR,np.minimum(uGIDirGoal,uOIDirObs))
    RMD[29]= np.minimum(MDL,np.minimum(uGRDirGoal,uOIDirObs))
        
    RMD[30]= np.minimum(MDR,np.minimum(uGADirGoal,uORDirObs))
    RMD[31]= np.minimum(MDM,np.minimum(uGADDirGoal,uORDirObs))
    RMD[32]= np.minimum(MDR,np.minimum(uGAIDirGoal,uORDirObs))
    RMD[33]= np.minimum(MDL,np.minimum(uGDDirGoal,uORDirObs))
    RMD[34]= np.minimum(MDR,np.minimum(uGIDirGoal,uORDirObs))
    RMD[35]= np.minimum(MDL,np.minimum(uGRDirGoal,uORDirObs))
        
    RMD[36] = np.minimum(MDR,np.minimum(uGADirGoal,uONDirObs))
    RMD[37]= np.minimum(MDM,np.minimum(uGADDirGoal,uONDirObs))
    RMD[38]= np.minimum(MDR,np.minimum(uGAIDirGoal,uONDirObs))
    RMD[39]= np.minimum(MDL,np.minimum(uGDDirGoal,uONDirObs))
    RMD[40]= np.minimum(MDR,np.minimum(uGIDirGoal,uONDirObs))
    RMD[41]= np.minimum(MDL,np.minimum(uGRDirGoal,uONDirObs))
    
    
    
    """
    R[0]= np.minimum(uGADirGoal,uOADirObs)
    R[1]= np.minimum(uGADDirGoal,uOADirObs)
    R[2]= np.minimum(uGAIDirGoal,uOADirObs)
    R[3]= np.minimum(uGDDirGoal,uOADirObs)
    R[4]= np.minimum(uGIDirGoal,uOADirObs)
    R[5]= np.minimum(uGRDirGoal,uOADirObs)
    
    R[6]= np.minimum(uGADirGoal,uOADDirObs)
    R[7]= np.minimum(uGADDirGoal,uOADDirObs)
    R[8]= np.minimum(uGAIDirGoal,uOADDirObs)
    R[9]= np.minimum(uGDDirGoal,uOADDirObs)
    R[10]= np.minimum(uGIDirGoal,uOADDirObs)
    R[11]= np.minimum(uGRDirGoal,uOADDirObs)
    
    R[12]= np.minimum(uGADirGoal,uOAIDirObs)
    R[13]= np.minimum(uGADDirGoal,uOAIDirObs)
    R[14]= np.minimum(uGAIDirGoal,uOAIDirObs)
    R[15]= np.minimum(uGDDirGoal,uOAIDirObs)
    R[16]= np.minimum(uGIDirGoal,uOAIDirObs)
    R[17]= np.minimum(uGRDirGoal,uOAIDirObs)
    
    R[18]= np.minimum(uGADirGoal,uODDirObs)
    R[19]= np.minimum(uGADDirGoal,uODDirObs)
    R[20]= np.minimum(uGAIDirGoal,uODDirObs)
    R[21]= np.minimum(uGDDirGoal,uODDirObs)
    R[22]= np.minimum(uGIDirGoal,uODDirObs)
    R[23]= np.minimum(uGRDirGoal,uODDirObs)
    
    R[24]= np.minimum(uGADirGoal,uOIDirObs)
    R[25]= np.minimum(uGADDirGoal,uOIDirObs)
    R[26]= np.minimum(uGAIDirGoal,uOIDirObs)
    R[27]= np.minimum(uGDDirGoal,uOIDirObs)
    R[28]= np.minimum(uGIDirGoal,uOIDirObs)
    R[29]= np.minimum(uGRDirGoal,uOIDirObs)
    
    R[30]= np.minimum(uGADirGoal,uORDirObs)
    R[31]= np.minimum(uGADDirGoal,uORDirObs)
    R[32]= np.minimum(uGAIDirGoal,uORDirObs)
    R[33]= np.minimum(uGDDirGoal,uORDirObs)
    R[34]= np.minimum(uGIDirGoal,uORDirObs)
    R[35]= np.minimum(uGRDirGoal,uORDirObs)
    
    R[36]= np.minimum(uGADirGoal,uONDirObs)
    R[37]= np.minimum(uGADDirGoal,uONDirObs)
    R[38]= np.minimum(uGAIDirGoal,uONDirObs)
    R[39]= np.minimum(uGDDirGoal,uONDirObs)
    R[40]= np.minimum(uGIDirGoal,uONDirObs)
    R[41]= np.minimum(uGRDirGoal,uONDirObs)
    
    mayor=0
    i=0
    for numero in R:
        if numero > mayor:
            mayor = numero
            mayori=i
        i=i+1
    #print(mayori)
    time.sleep(0.1)
    #print(anguloobs)
    if(mayori==1 or mayori==13 or  mayori==21 or mayori==25 or mayori==31 or mayori==37):
        print('Acción:','Adelante Derecha')   
        errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,1, vrep.simx_opmode_streaming)
        errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,0.2, vrep.simx_opmode_streaming)
    if(mayori==6 or mayori==2 or mayori==8 or mayori==20 or mayori==26 or mayori==28 or mayori==32 or mayori==38 ):
        print('Acción:','Adelante Izquierda')   
        errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,0.2, vrep.simx_opmode_streaming)
        errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,1, vrep.simx_opmode_streaming)
    if(mayori==24 or mayori==3 or mayori==5 or mayori==9 or mayori==11 or mayori==15 or mayori==17 or mayori==27 or mayori==29 or mayori==33 or mayori==35 or mayori==27 or mayori==39 or mayori==41):
        print('Acción:','Derecha')   
        errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,1, vrep.simx_opmode_streaming)
        errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,0, vrep.simx_opmode_streaming)
    if(mayori==0 or mayori==4 or mayori==10 or mayori==16 or mayori==22 or mayori==23 or mayori==34 or mayori==40):
        print('Acción:','Izquierda')   
        errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,0, vrep.simx_opmode_streaming)
        errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,1, vrep.simx_opmode_streaming)
    if(mayori==19 or mayori==7 or mayori==12 or mayori==18 or mayori==30 or mayori==36 or mayori==14):
        print('Acción:','Adelante')   
        errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,1, vrep.simx_opmode_streaming)
        errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,1, vrep.simx_opmode_streaming)
    """
    AreaFD=0
    AreaFD=np.maximum(RMD[0],AreaFD)
    AreaFI=0
    AreaFI=np.maximum(RMI[0],AreaFI)
    for i in range(41):
        AreaFD=np.maximum(AreaFD,RMD[i+1])
        AreaFI=np.maximum(AreaFI,RMI[i+1])
    CoMD=np.sum(US*AreaFD)/np.sum(AreaFD)
    CoMI=np.sum(US*AreaFI)/np.sum(AreaFI)
    #time.sleep(0.3)
    #print('return_code  ',return_code)    
    #print('detection_state  ',detection_state)
   # print('Regla#',mayori)    
    #print('detected_point  ',detected_point)   
    #print('Orientación',OriRobot)
    #print('angulo',angulogoal)
    #print('Anguloobs',anguloobs)
    #print('DirGoal',DirGoal)
    #print('AcciónI',CoMI)
    #print('AcciónD',CoMD)
    #print('DirObs',DirObs)
    #print('detected_object_handle  ', detected_object_handle)    
    #print('detected_surface_normal_vector  ',detected_surface_normal_vector)
    
    if((PosRobot[0]-PosGoal[0])**2+(PosRobot[1]-PosGoal[1])**2<0.05):
        Timbrebipbip=False
    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,CoMI*1.5, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,CoMD*1.5, vrep.simx_opmode_streaming)  
    if(Timbrebipbip==False):
        errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,0, vrep.simx_opmode_streaming)
        errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,0, vrep.simx_opmode_streaming)  
     
    
 