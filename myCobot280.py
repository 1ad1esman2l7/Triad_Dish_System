import numpy as np
import time
import matplotlib.pyplot as plt
import os
import pandas as pd
import swift
from scipy import linalg
from spatialmath.base import *
from spatialmath import SE3
from roboticstoolbox import jtraj
import roboticstoolbox as rtb
from ir_support import schunk_UTS_v2_0, CylindricalDHRobotPlot, SwiftUFOFleet, check_intersections
from spatialgeometry import Cylinder
from roboticstoolbox import DHLink, DHRobot


# Useful variables
from math import pi


class myCobot280(DHRobot):
    def __init__(self):
        links = self._create_DH()
        qtest = [0.0, 0.0, -pi/2, 0.0, 0.0, 0.0]
        super().__init__(links, name='myCobot280')
        self.q = qtest
        
    def _create_DH(self):
        # Your existing DH creation code stays the same
        links = []
        a = [0.0, -110.4*0.0055, -96*0.0055, 0.0, 0.0, 0.0]
        d = [131.22*0.0055, 0.0, 0.0, 63.4*0.0055, 75.05*0.0055, 45.6*0.0055]  
        alpha = [pi/2, 0.0, 0.0, pi/2, -pi/2, 0.0]  
        offset = [0.0, -pi/2, 0.0, -pi/2, pi/2, 0.0]  
        qlim = np.array([[-168*pi/180, -135*pi/180, -150*pi/180, -145*pi/180, -165*pi/180, -180*pi/180],
                        [168*pi/180,  135*pi/180,  150*pi/180,  145*pi/180,  165*pi/180,  180*pi/180]])
        
        for i in range(6):
            link = DHLink(d=d[i], a=a[i], alpha=alpha[i], offset=offset[i], qlim=qlim[:, i])
            links.append(link)
        return links
    
    def add_to_env(self, env, cylinder_radius=0.015, multicolor=True):
        cyl_viz = CylindricalDHRobotPlot(self, cylinder_radius=cylinder_radius, multicolor=multicolor)
        robot_with_cylinders = cyl_viz.create_cylinders()
        env.add(robot_with_cylinders)
        return robot_with_cylinders
    
    def test(self):
        env = swift.Swift()
        env.launch(realtime=True)
        
        self.add_to_env(env)
        
        env.set_camera_pose([12, 12, 11], [0, 0, 0.4])
        self.q = [0.0, 0.0, -pi/2, 0.0, 0.0, 0.0]
        q_goal = [self.q[i]-pi/3 for i in range(self.n)]
        qtraj = rtb.jtraj(self.q, q_goal, 50).q
        
        input("Press enter to start movement")
        
        for q in qtraj:
            self.q = q
            env.step(0.02)
        
        env.hold()
        time.sleep(3)


if __name__ == "__main__":
    r = myCobot280()
    r.test()