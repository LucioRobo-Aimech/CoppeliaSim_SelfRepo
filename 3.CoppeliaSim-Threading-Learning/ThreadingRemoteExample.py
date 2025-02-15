# -*- coding: UTF-8 -*-
'''
@Project ：CoppeliaSim_SelfRepo 
@File    ：ThreadingRemoteExample.py
@Author  ：Lucio.YipengLi@outlook.com
@Date    ：2025/2/15 下午1:35 
'''
import threading
import coppeliaSimType
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import copy

def list_minus(a_list,b_list):
    # 列表元素对应相减
    c_list = [x - y for x, y in zip(a_list, b_list)]
    return c_list

def list_add(a_list,b_list):
    # 列表元素对应相加
    c_list = [x + y for x, y in zip(a_list, b_list)]
    return c_list

def BlueBall():
    client = RemoteAPIClient()
    sim:coppeliaSimType.SimType = client.require('sim')
    sim.setStepping(True)
    # 获取红蓝球对象
    b_ball = sim.getObject("/Sphere1")
    r_ball = sim.getObject("/Sphere2")
    # 分别获取物体的初始位置
    r_parent_postion = sim.getObjectPosition(r_ball,sim.handle_world)
    b_parent_position = sim.getObjectPosition(b_ball,sim.handle_world)
    while (t := sim.getSimulationTime()) < 5:
        # 获取红色的当前位置并与上一次位置做差
        r_position = sim.getObjectPosition(r_ball,sim.handle_world)
        r_minus_position = list_minus(r_position, r_parent_postion)
        # 将差值坐标赋值给蓝色球
        b_add_position = list_add(r_minus_position,b_parent_position)
        b_parent_position = sim.getObjectPosition(b_ball, sim.handle_world)
        # 必须深拷贝！
        r_parent_postion = copy.deepcopy(r_position)
        # 设置蓝色小球位置
        sim.setObjectPosition(b_ball,b_add_position,sim.handle_world)
        sim.step()



def RedBall():
    client = RemoteAPIClient()
    sim: coppeliaSimType.SimType = client.require('sim')
    sim.setStepping(True)
    # 获取红球对象
    r_ball = sim.getObject("/Sphere2")
    while (t := sim.getSimulationTime()) < 5:
        # 查询红球坐标并每次step增加红球X坐标0.001
        r_position = sim.getObjectPosition(r_ball, sim.handle_world)
        r_position[0] += 0.001
        sim.setObjectPosition(r_ball, r_position, sim.handle_world)
        sim.step()



client = RemoteAPIClient()
sim:coppeliaSimType.SimType = client.require('sim')
sim.startSimulation()
# 创建两个任务线程
blueball_thread = threading.Thread(target=BlueBall)
redball_thread = threading.Thread(target=RedBall)
# 线程！启动！
blueball_thread.start()
redball_thread.start()
# 等候两个任务都完成（但其实在任务中我设定了仿真五秒）
blueball_thread.join()
redball_thread.join()
# 结束仿真
sim.stopSimulation()