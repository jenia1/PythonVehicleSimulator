# -*- coding: utf-8 -*-
"""
Main simulation loop.

Author:     Thor I. Fossen
Date:       25 July 2021
"""

from .kinematics import attitudeEuler
import numpy as np

# Function simInfo(vehicle)
def simInfo(vehicle):
    print('\nThe Python Vehicle Simulator:')
    print('Vehicle:         %s, L = %s' % (vehicle.name, vehicle.L))
    print('Control system:  %s' % (vehicle.controlDescription))

        
# Function simulate(N, sampleTime, vehicle)
def simulate(N, sampleTime, vehicle):
    
    DOF = 6                     # degrees of freedom
    t = 0                       # initial simulation time

    # initial state vectors
    eta = np.array( [0, 0, 0, 0, 0,0], float)    # user editable
    nu = vehicle.nu                              # defined by vehicle class   

    simInfo(vehicle)            # print simulation info

    # initialization of table used to store the simulation data
    simData = np.empty( [0, 2*DOF + vehicle.dimU], float)
    
    # simulator for-loop
    for i in range(0,N+1):
        
        t = i * sampleTime      # simulation time
        
        # vehicle specific control systems
        if (vehicle.controlMode == 'depthAutopilot'):
            u_control = vehicle.depthAutopilot(eta,nu,sampleTime)
        elif (vehicle.controlMode == 'headingAutopilot'):
            u_control = vehicle.headingAutopilot(eta,nu,sampleTime)            
        elif (vehicle.controlMode == 'stepInput'):
            u_control = vehicle.stepInput(t)          
        
        # store simulation data in simData
        simData = np.vstack( [simData, np.append(np.append(eta,nu),u_control)] )
        
        # Propagate vehicle and attitude dynamics
        nu  = vehicle.dynamics(eta,nu,u_control,sampleTime)
        eta = attitudeEuler(eta,nu,sampleTime)

    # store simulation time vector
    simTime = np.arange(start=0, stop=t+sampleTime, step=sampleTime)[:, None]

    return(simTime,simData)
