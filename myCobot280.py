import swift
import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3
import numpy as np
from ir_support.robots.DHRobot3D import DHRobot3D
import time
import os

# Useful variables
from math import pi

# -----------------------------------------------------------------------------------#
class myCobot280(DHRobot3D):
    def __init__(self):
        """
        myCobot280 Robot.
        See the use of `myCobot280` and base class `DHRobot3D`

        """
        # DH links
        links = self._create_DH()

        # Names of the robot link files in the directory
        link3D_names = dict(link0 = 'myCobot280Link0', color0 = (0.2,0.2,0.2,1),      # color option only takes effect for stl file
                            link1 = 'myCobot280Link1', color1 = (1.0, 1.0, 1.0, 1),
                            link2 = 'myCobot280Link2', color2 = (1.0, 1.0, 1.0, 1),
                            link3 = 'myCobot280Link3', color3 = (1.0, 1.0, 1.0, 1),
                            link4 = 'myCobot280Link4', color4 = (1.0, 1.0, 1.0, 1),
                            link5 = 'myCobot280Link5', color5 = (1.0, 1.0, 1.0, 1),
                            link6 = 'myCobot280Link6', color6 = (1.0, 1.0, 1.0, 1))

        # A joint config and the 3D object transforms to match that config
        qtest = [0,0,-pi/2,0,0,0]
        qtest_transforms = [spb.transl(0,0,0.020479),
                            spb.transl(-0.00541,0.000927,0.069135),
                            spb.transl(-0.028176,0.000762,0.105488),
                            spb.transl(-0.005693,0.000795,0.156513),
                            spb.transl(-0.028501,0.000837,0.186461),
                            spb.transl(-0.032688,0.008002,0.217543),
                            spb.transl(-0.032783,-0.016991,0.220284)]
        
        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(links, link3D_names, name = 'myCobot280', link3d_dir = current_path, qtest = qtest, qtest_transforms = qtest_transforms)
        # Lift the entire robot above ground plane to ensure no geometry sits below Z=0
        # self.base = SE3(0, 0, 0.12)
        # self.base = SE3(0, 0, 0) * SE3.Rx(pi/2) * SE3.Ry(-pi/2)
        self.q = qtest

    # -----------------------------------------------------------------------------------#
    def _create_DH(self):
        """
        Create robot's standard DH model
        """
        links = []    
        a = [0, -110.4, -96, 0, 0, 0]
        d = [131.22, 0, 0, 63.4, 75.05, 45.6]  
        alpha = [0, 0, 0, 0, 0, 0]
        offset = [0,-1.5708, 0, -1.5708, -1.5708, 0]
        qlim = [[-2*pi, 2*pi] for _ in range(6)]
        for i in range(6):
            link = rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i], offset=offset[i], qlim=qlim[i])
            links.append(link)
        return links

    # -----------------------------------------------------------------------------------#
    def test(self):
        """
        Test the class by adding 3d objects into a new Swift window and do a simple movement
        """
        env = swift.Swift()
        env.launch(realtime= True)
        self.q = self._qtest
        self.add_to_env(env)

        q_goal = [self.q[i]-pi/3 for i in range(self.n)]
        q_goal[0] = -0.8 # Move the rail link
        qtraj = rtb.jtraj(self.q, q_goal, 50).q
        # fig = self.plot(self.q, limits= [-1,1,-1,1,-1,1])
        # fig._add_teach_panel(self, self.q)
        for q in qtraj:
            self.q = q
            env.step(0.02)
            # fig.step(0.01)
        # fig.hold()
        env.hold()
        time.sleep(3)

# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":
    r = myCobot280()
    input("Press enter to test movement of myCobot280")
    r.test()