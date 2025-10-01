##  @file
#   @brief Rough version of UR3 Robot on a linear rail defined by standard static DH parameters with 3D model
#   @author Adam Scicluna
#   @date August 23, 2025

import swift
import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3
from ir_support.robots.DHRobot3D import DHRobot3D
import time
import os

# Useful variables
from math import pi

# -----------------------------------------------------------------------------------#
class DensoCobotta1300(DHRobot3D):
    def __init__(self):
        """
        UR3 Robot on a Linear Rail.
        See the use of `UR3` and base class `DHRobot3D`

        """
        # DH links
        links = self._create_DH()

        # Names of the robot link files in the directory
        link3D_names = dict(link0 = 'Denso_Link_0',      # color option only takes effect for stl file
                            link1 = 'Denso_Link_1',
                            link2 = 'Denso_Link_2',
                            link3 = 'Denso_Link_3',
                            link4 = 'Denso_Link_4',
                            link5 = 'Denso_Link_5',
                            link6 = 'Denso_Link_6')

        # A joint config and the 3D object transforms to match that config
        qtest = [0,0,-pi/2,0,0,0]
        qtest_transforms = [
                            spb.transl(-0.003159, 0.008183 , -0.001281 ) ,
                            spb.transl(-0.01086 , -0.00092 , 0.124937 ),
                            spb.transl(0.093014, -0.000036, 0.272486 ),
                            spb.transl(-0.028778 , -0.062852, 0.468212),
                            spb.transl(-0.029088 , -0.2963 , 0.464883 ),
                            spb.transl(0.050788 , -0.296175 , 0.468289 ),
                            spb.transl(0.046125, -0.361762, 0.464344 ),
                            ]

        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(links, link3D_names, name = 'DensoCobotta1300', link3d_dir = current_path, qtest = qtest, qtest_transforms = qtest_transforms)
        # self.base = self.base * SE3.Rx(pi/2) * SE3.Ry(pi/2)
        self.q = qtest

    # -----------------------------------------------------------------------------------#
    def _create_DH(self):
        """
        Create robot's standard DH model
        """
        links = []    # Stores data for each link
        a = [0, -0.24365, -0.21325, 0, 0, 0]  # a and d are the link offsets for each link , a is x and d is z
        d = [0.146, 0, 0, 0.121, 0.083, 0.0819]    
        alpha = [pi/2, 0, 0, pi/2, -pi/2, 0]      # 
        offset = [0,0,0,0,0,0]
        qlim = [[-2*pi, 2*pi] for _ in range(6)]    # Joint limits for each joint
        for i in range(6):             # dont change anything in for loop
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
    r = DensoCobotta1300()
    input("Press enter to test movement of DensoCobotta1300")
    r.test()


