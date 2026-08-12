#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Main simulation loop called by main.py.

Author:     Thor I. Fossen
"""

import numpy as np
from .gnc import attitudeEuler
from python_vehicle_simulator.sensors import IMU
from python_vehicle_simulator.navigation import StrapdownINS

###############################################################################
# Function printSimInfo(vehicle)
###############################################################################
def printSimInfo():
    
    """
    Constructors used to define the vehicle objects as (see main.py for details):
        DSRV('depthAutopilot',z_d)                                       
        frigate('headingAutopilot',U,psi_d)
        otter('headingAutopilot',psi_d,V_c,beta_c,tau_X)                  
        ROVzefakkel('headingAutopilot',U,psi_d)                          
        semisub('DPcontrol',x_d,y_d,psi_d,V_c,beta_c)                       
        shipClarke83('headingAutopilot',psi_d,L,B,T,Cb,V_c,beta_c,tau_X)  
        supply('DPcontrol',x_d,y_d,psi_d,V_c,beta_c)      
        tanker('headingAutopilot',psi_d,V_c,beta_c,depth)    
        remus100('depthHeadingAutopilot',z_d,psi_d,V_c,beta_c)
    """     
    
    print('---------------------------------------------------------------------------------------')
    print('The Python Vehicle Simulator')
    print('---------------------------------------------------------------------------------------')
    print(' 1 - Deep submergence rescue vehicle (DSRV): controlled by a stern plane, L = 5.0 m')
    print(' 2 - Frigate: rudder-controlled ship described by a nonlinear Nomoto model, L = 100.0 m')
    print(' 3 - Otter unmanned surface vehicle (USV): controlled by two propellers, L = 2.0 m')
    print(' 4 - ROV Zefakkel: rudder-controlled ship described by a nonlinear Nomoto model, L = 54.0 m')
    print(' 5 - Semisubmersible: controlled by tunnel thrusters and main propellers, L = 84.5 m')
    print(' 6 - Ship: linear maneuvering model specified by L, B and T using the Clarke (1983) formulas')
    print(' 7 - Offshore supply vessel: controlled by tunnel thrusters and main propellers, L = 76.2 m')
    print(' 8 - Tanker: rudder-controlled ship model including shallow water effects, L = 304.8 m')
    print(' 9 - REMUS 100: AUV controlled by stern planes, a tail rudder and a propeller, L = 1.6 m')
    print("10 - Torpedo: Inspired by the REMUS 100 AUV, configurable fins and propeller, L = 1.6 m")
    print('---------------------------------------------------------------------------------------')    
    
###############################################################################    
# Function printVehicleinfo(vehicle)
###############################################################################
def printVehicleinfo(vehicle, sampleTime, N): 
    print('---------------------------------------------------------------------------------------')
    print('%s' % (vehicle.name))
    print('Length: %s m' % (vehicle.L))
    print('%s' % (vehicle.controlDescription))  
    print('Sampling frequency: %s Hz' % round(1 / sampleTime))
    print('Simulation time: %s seconds' % round(N * sampleTime))
    print('---------------------------------------------------------------------------------------')
    

###############################################################################
# Function simulate(N, sampleTime, vehicle)
###############################################################################
def simulate(N, sampleTime, vehicle, eta0=None, imu_mode="ideal"):

    DOF = 6                     # degrees of freedom
    t = 0                       # initial simulation time

    # Initial state vectors
    if eta0 is None:
        eta0 = [0, 0, 0, 0, 0, 0]
    eta = np.array(eta0, float)                  # position/attitude
    nu = vehicle.nu                              # velocity, defined by vehicle class
    u_actual = vehicle.u_actual                  # actual inputs, defined by vehicle class

    # imu_mode: "ideal" (no errors) or "simple" (bias/scale/misalignment/noise
    # error model, currently set to ideal values in IMU.py until real sensor
    # specs are added)
    imu = IMU(mode=imu_mode)

    # Navigation: dead-reckons eta_est, nu_est from IMU measurements only.
    # Initialized with the true initial state (a nav system must start
    # somewhere); estimate is what the controller sees from here on.
    # u_const: some vehicle models (e.g. DSRV) hardcode a constant cruise
    # speed without a matching Coriolis term in nu_dot[0], so pin the
    # estimate to it too instead of integrating a fictitious accelerometer
    # reading (see StrapdownINS docstring)
    u_const = getattr(vehicle, "U0", None)
    nav = StrapdownINS(eta, nu, u_const=u_const)
    eta_est, nu_est = nav.eta_est, nav.nu_est

    # Initialization of table used to store the simulation data
    simData = np.empty( [0, 2*DOF + 2 * vehicle.dimU], float)
    imuData = np.empty( [0, 6], float)            # [f_b (3), omega_b (3)]
    navData = np.empty( [0, 2*DOF], float)        # [eta_est (6), nu_est (6)]

    # Simulator for-loop
    for i in range(0,N+1):

        t = i * sampleTime      # simulation time

        # Vehicle specific control systems - use the navigation estimate,
        # not the true eta/nu, since a real controller only has access to
        # what the sensors/navigation filter provide
        if (vehicle.controlMode == 'depthAutopilot'):
            u_control = vehicle.depthAutopilot(eta_est,nu_est,sampleTime)
        elif (vehicle.controlMode == 'headingAutopilot'):
            u_control = vehicle.headingAutopilot(eta_est,nu_est,sampleTime)
        elif (vehicle.controlMode == 'depthHeadingAutopilot'):
            u_control = vehicle.depthHeadingAutopilot(eta_est,nu_est,sampleTime)
        elif (vehicle.controlMode == 'DPcontrol'):
            u_control = vehicle.DPcontrol(eta_est,nu_est,sampleTime)
        elif (vehicle.controlMode == 'stepInput'):
            u_control = vehicle.stepInput(t)

        # Store simulation data in simData (true state) and navData (estimate)
        signals = np.append( np.append( np.append(eta,nu),u_control), u_actual )
        simData = np.vstack( [simData, signals] )
        navData = np.vstack( [navData, np.append(eta_est, nu_est)] )

        # Propagate vehicle dynamics (ground truth). nu_dot can only be
        # computed here (it depends on u_control), and it's valid at the
        # pre-update eta - so the IMU call below must come before
        # attitudeEuler() advances eta, not before this.
        [nu, u_actual, nu_dot]  = vehicle.dynamics(eta,nu,u_actual,u_control,sampleTime)

        # IMU measurement (mode selected by the imu_mode flag: "ideal" or "simple")
        imu_meas = imu.measure(eta, nu, nu_dot)
        imuData = np.vstack([imuData, np.append(imu_meas["f_b"], imu_meas["omega_b"])])

        # Navigation update: IMU measurement in, eta_est/nu_est out, used by
        # the controller on the next iteration
        eta_est, nu_est = nav.update(imu_meas, sampleTime)

        # Propagate attitude dynamics (ground truth)
        eta = attitudeEuler(eta,nu,sampleTime)

    # Store simulation time vector
    simTime = np.arange(start=0, stop=t+sampleTime, step=sampleTime)[:, None]

    return(simTime,simData,imuData,navData)
