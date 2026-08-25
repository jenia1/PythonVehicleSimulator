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
# Body stepping: attaches nav/IMU state onto a vehicle-like object and steps it
###############################################################################
def initBody(body, eta0, imu_mode="ideal"):
    """
    Attaches position/attitude and navigation state onto a vehicle-like
    object, alongside the .nu/.u_actual it already carries, so simulate()
    can step any such body (a vehicle today, e.g. a dock later) through the
    same code path.

    body must provide: .dynamics(eta, nu, u_actual, u_control, sampleTime),
    .nu, .u_actual, and optionally .U0 (see StrapdownINS docstring).
    """
    body.eta = np.array(eta0, float)              # position/attitude

    # imu_mode: "ideal" (no errors) or "simple" (bias/scale/misalignment/
    # noise error model, currently set to ideal values in IMU.py until real
    # sensor specs are added)
    body.imu = IMU(mode=imu_mode)

    # Navigation: dead-reckons eta_est, nu_est from IMU measurements only.
    # Initialized with the true initial state (a nav system must start
    # somewhere); estimate is what the controller sees from here on.
    # u_const: some models (e.g. DSRV) hardcode a constant cruise speed
    # without a matching Coriolis term in nu_dot[0], so pin the estimate to
    # it too instead of integrating a fictitious accelerometer reading (see
    # StrapdownINS docstring)
    u_const = getattr(body, "U0", None)
    body.nav = StrapdownINS(body.eta, body.nu, u_const=u_const)
    body.eta_est, body.nu_est = body.nav.eta_est, body.nav.nu_est


def stepBody(body, u_control, sampleTime):
    """
    Propagates dynamics for one sample, measures the IMU at the pre-update
    eta (nu_dot is only valid there - see mainLoop history), updates the
    navigation estimate, then advances eta. Returns the IMU measurement dict
    for logging.
    """
    nu, u_actual, nu_dot = body.dynamics(
        body.eta, body.nu, body.u_actual, u_control, sampleTime
    )

    imu_meas = body.imu.measure(body.eta, nu, nu_dot)
    body.eta_est, body.nu_est = body.nav.update(imu_meas, sampleTime)

    body.eta = attitudeEuler(body.eta, nu, sampleTime)
    body.nu, body.u_actual = nu, u_actual

    return imu_meas


def _computeControl(vehicle, eta_est, nu_est, t, sampleTime):
    """
    Vehicle specific control systems - uses the navigation estimate, not the
    true eta/nu, since a real controller only has access to what the
    sensors/navigation filter provide.
    """
    if vehicle.controlMode == 'depthAutopilot':
        return vehicle.depthAutopilot(eta_est, nu_est, sampleTime)
    elif vehicle.controlMode == 'headingAutopilot':
        return vehicle.headingAutopilot(eta_est, nu_est, sampleTime)
    elif vehicle.controlMode == 'depthHeadingAutopilot':
        return vehicle.depthHeadingAutopilot(eta_est, nu_est, sampleTime)
    elif vehicle.controlMode == 'DPcontrol':
        return vehicle.DPcontrol(eta_est, nu_est, sampleTime)
    elif vehicle.controlMode == 'stepInput':
        return vehicle.stepInput(t)


###############################################################################
# Function simulate(N, sampleTime, vehicle)
###############################################################################
def simulate(N, sampleTime, vehicle, eta0=None, imu_mode="ideal"):

    DOF = 6                     # degrees of freedom

    if eta0 is None:
        eta0 = [0, 0, 0, 0, 0, 0]

    initBody(vehicle, eta0, imu_mode=imu_mode)

    # Initialization of table used to store the simulation data
    simData = np.empty( [0, 2*DOF + 2 * vehicle.dimU], float)
    imuData = np.empty( [0, 6], float)            # [f_b (3), omega_b (3)]
    navData = np.empty( [0, 2*DOF], float)        # [eta_est (6), nu_est (6)]

    # Simulator for-loop
    t = 0
    for i in range(0,N+1):

        t = i * sampleTime      # simulation time

        u_control = _computeControl(vehicle, vehicle.eta_est, vehicle.nu_est, t, sampleTime)

        # Store simulation data in simData (true state) and navData (estimate)
        signals = np.append( np.append( np.append(vehicle.eta,vehicle.nu),u_control), vehicle.u_actual )
        simData = np.vstack( [simData, signals] )
        navData = np.vstack( [navData, np.append(vehicle.eta_est, vehicle.nu_est)] )

        imu_meas = stepBody(vehicle, u_control, sampleTime)
        imuData = np.vstack([imuData, np.append(imu_meas["f_b"], imu_meas["omega_b"])])

    # Store simulation time vector
    simTime = np.arange(start=0, stop=t+sampleTime, step=sampleTime)[:, None]

    return(simTime,simData,imuData,navData)
