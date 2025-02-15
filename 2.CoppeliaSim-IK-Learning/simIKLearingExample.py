# -*- coding: UTF-8 -*-
'''
@Project ：CoppeliaSim_SelfRepo 
@File    ：simIKLearingExample.py
@Author  ：Lucio.YipengLi@outlook.com
@Date    ：2025/2/15 下午1:43 
'''
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import coppeliaSimType
import atexit

programRunning = True

sim, simIK = coppeliaSimType.SimType,coppeliaSimType.SimIKType
ikEnv,ikGroup_ud,ikGroup_d = None,None,None

simBase, simTarget, simJoints=  None,None,[]
ikTarget, ikBase, ikJoints = None,None,[]


def sysCall_init():
    global ikEnv
    global ikGroup_d
    global ikGroup_ud
    global sim
    global simIK
    global simBase
    global simTarget
    global simJoints
    global ikJoints
    global ikTarget
    global ikBase
    #  System Init  #
    client = RemoteAPIClient()
    sim= client.require('sim')
    simIK = client.require('simIK')
    sim.setStepping(True)
    sim.startSimulation()

    # 简单方式
    simBase = sim.getObject("/UR5")
    simTip = sim.getObject("/tip1")
    simTarget = sim.getObject("/target1")

    ikEnv = simIK.createEnvironment()
    ikGroup_ud = simIK.createGroup(ikEnv)
    simIK.setGroupCalculation(ikEnv,ikGroup_ud,simIK.method_pseudo_inverse,0,20)
    simIK.addElementFromScene(ikEnv, ikGroup_ud, simBase, simTip, simTarget, simIK.constraint_pose)

    ikGroup_d = simIK.createGroup(ikEnv)
    simIK.setGroupCalculation(ikEnv, ikGroup_d, simIK.method_damped_least_squares, 1, 99)
    simIK.addElementFromScene(ikEnv, ikGroup_d, simBase, simTip, simTarget, simIK.constraint_pose)

def sysCall_actuation():
    global programRunning
    global ikEnv
    global ikGroup_d
    global ikGroup_ud
    global sim
    global simIK
    global simBase
    global simTarget
    global simJoints
    global ikJoints
    global ikTarget
    global ikBase
    while (t := sim.getSimulationTime()) < 10:
        # 简单方法
        res, *_ = simIK.handleGroup(ikEnv, ikGroup_ud, {'syncWorlds': True})
        if res != simIK.result_success:
            simIK.handleGroup(ikEnv, ikGroup_d, {'syncWorlds': True})
            sim.addLog(sim.verbosity_scriptwarnings, "IK solver failed.")
        sim.step()
    sim.stopSimulation()
    pass


def sysCall_sensing():
    # put your sensing code here
    pass


def sysCall_cleanup():
    global sim
    sim.stopSimulation()
    print("end")
    # do some clean-up here
    pass


atexit.register(sysCall_cleanup)
sysCall_init()
sysCall_actuation()
