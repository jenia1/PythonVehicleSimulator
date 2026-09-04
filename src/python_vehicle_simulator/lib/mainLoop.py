#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Main simulation loop called by main.py.

Author:     Thor I. Fossen
"""

import numpy as np
from .gnc import attitudeEuler
from .simLog import SimLog
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
def simulate(N, sampleTime, vehicle, eta0=None, imu_mode="ideal",
             buoy=None, buoy_eta0=None, cable=None, dock=None, dock_eta0=None):
    """
    log = simulate(N, sampleTime, vehicle, eta0, imu_mode) runs the simulation
    and returns a SimLog holding one named block per body:

        log.time                     (N+1, 1) simulation time
        log["vehicle"]["eta"]        (N+1, 6) true position/attitude
        log["vehicle"]["eta_est"]    (N+1, 6) navigation estimate
        log.simData("vehicle")       eta | nu | u_control | u_actual, the
                                     layout the plotTimeSeries functions take

    buoy, cable, dock: optional docking structure, stepped alongside the
    vehicle. All default to None, in which case only the vehicle is simulated
    and the result is unchanged (see compare_simdata.py). The models
    themselves are not written yet - only the call sites below are in place:

        buoy  - anchor point, static for now, but carries its own IMU and
                navigation filter, so it is stepped like any other body
        cable - buoy <-> dock coupling, returns the force acting on the dock
        dock  - dynamic body at the cable end, with its own IMU and navigation
    """

    if eta0 is None:
        eta0 = [0, 0, 0, 0, 0, 0]

    initBody(vehicle, eta0, imu_mode=imu_mode)

    if buoy is not None:
        initBody(buoy, buoy_eta0, imu_mode=imu_mode)

    if dock is not None:
        initBody(dock, dock_eta0, imu_mode=imu_mode)

    # Simulation data, one named block per body. The control input names come
    # from the vehicle, everything else uses the SimLog defaults.
    log = SimLog(N, sampleTime)
    log.setNames("vehicle", "u_control",
                 [name + " (command)" for name in vehicle.controls])
    log.setNames("vehicle", "u_actual",
                 [name + " (actual)" for name in vehicle.controls])

    # Simulator for-loop
    for i in range(0,N+1):

        t = i * sampleTime      # simulation time

        u_control = _computeControl(vehicle, vehicle.eta_est, vehicle.nu_est, t, sampleTime)

        # Store the true state and the navigation estimate, both taken before
        # the step below advances them
        log.log("vehicle", i,
                eta=vehicle.eta, nu=vehicle.nu,
                u_control=u_control, u_actual=vehicle.u_actual,
                eta_est=vehicle.eta_est, nu_est=vehicle.nu_est)

        imu_meas = stepBody(vehicle, u_control, sampleTime)
        log.log("vehicle", i,
                f_b=imu_meas["f_b"], omega_b=imu_meas["omega_b"])

        # Docking structure: buoy -> cable -> dock, each body stepped and
        # logged the same way as the vehicle - true state and navigation
        # estimate before the step, IMU measurement after it. Neither the buoy
        # nor the dock has actuators of its own, hence u_control = None.

        # The buoy is a static anchor for now, but it still carries an IMU and
        # a navigation filter, so it is stepped like any other body - its
        # dynamics simply holds it in place.
        if buoy is not None:
            log.log("buoy", i,
                    eta=buoy.eta, nu=buoy.nu,
                    eta_est=buoy.eta_est, nu_est=buoy.nu_est)

            buoy_imu_meas = stepBody(buoy, None, sampleTime)
            log.log("buoy", i,
                    f_b=buoy_imu_meas["f_b"], omega_b=buoy_imu_meas["omega_b"])

        # The cable returns the force acting on the dock, which the dock's
        # dynamics reads back off .f_ext - the role u_control plays for a
        # vehicle. It needs both of its ends, so it is only stepped when the
        # buoy is present too.
        if dock is not None:
            log.log("dock", i,
                    eta=dock.eta, nu=dock.nu,
                    eta_est=dock.eta_est, nu_est=dock.nu_est)

            if cable is not None and buoy is not None:
                dock.f_ext = cable.step(buoy.eta, dock.eta, dock.nu, sampleTime)

            dock_imu_meas = stepBody(dock, None, sampleTime)
            log.log("dock", i,
                    f_b=dock_imu_meas["f_b"], omega_b=dock_imu_meas["omega_b"])

    return log
