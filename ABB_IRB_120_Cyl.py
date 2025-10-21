
import swift
from roboticstoolbox import DHLink, DHRobot
from roboticstoolbox import jtraj
import numpy as np
from spatialmath import SE3
from ir_support import CylindricalDHRobotPlot
from math import pi
import time
 
# -----------------------------------------------------------------------------------#
class ABB_IRB_120_Cyl(DHRobot):
    def __init__(self):
        """
        Simplified ABB IRB 120 using DHRobot with cylinders.
        """
        # DH parameters (standard)
        a = [0, 0.270, 0.070, 0, 0, 0]
        alpha = [-pi/2, 0, -pi/2, pi/2, -pi/2, 0]
        d = [0.290, 0, 0, 0.302, 0, 0.072]
        offset = [0, -pi/2, 0, 0, 0, pi]
        qlim = [
            np.deg2rad([-165, 165]),   # Joint 1
            np.deg2rad([-110, 110]),   # Joint 2
            np.deg2rad([-110, 70]),   # Joint 3
            np.deg2rad([-160, 160]),   # Joint 4
            np.deg2rad([-120, 120]),   # Joint 5
            np.deg2rad([-400, 400])    # Joint 6
        ]
 
        # Create DH links
        links = [DHLink(a=a[i], alpha=alpha[i], d=d[i], offset=offset[i], qlim=qlim[i]) for i in range(6)]
 
        # Initialize DHRobot
        super().__init__(links, name='ABB_IRB_120_Cyl')
        # Add cylindrical visualization
        cyl_viz = CylindricalDHRobotPlot(self, cylinder_radius=0.03, color="#3478f6")
        cyl_viz.create_cylinders()
 
        # Initial joint configuration
        self.q = [0,0,np.deg2rad(1.3),0,0,0]
 
# -----------------------------------------------------------------------------------#
if __name__ == "__main__":
    # Create robot
    robot = ABB_IRB_120_Cyl()
 
    # Launch Swift environment
    env = swift.Swift()
    env.launch(realtime=True)
 
    # Add robot to environment
    env.add(robot)
 
    env.step()
 
 
    # ----------------------------

    input("Press enter to execute circular Cartesian trajectory")
    steps = 300
    delta_t = 0.05

    r = 0.2
    xc, yc, zc = 0.45, 0, 0.3  # Offset center
    x = np.zeros([3, steps])

    theta = 2 * pi / steps
    for step in np.arange(steps):
        # Circle in x-y, then rotate 90 deg about y axis (x->z, z->-x)
        x_circle = r * np.cos(step * theta)
        y_circle = r * np.sin(step * theta)
        # Apply rotation: x' = z, y' = y, z' = -x
        x[0, step] = xc + 0      # x' = xc + 0 (will set below)
        x[1, step] = yc + y_circle
        x[2, step] = zc + x_circle

    # Move robot along the circular trajectory using IK
    for step in range(steps):
        # Desired end-effector pose (SE3)
        T = SE3(x[0, step], x[1, step], x[2, step])
        sol = robot.ikine_LM(T)
        if sol.success:
            robot.q = sol.q
            env.step(delta_t)
        else:
            print(f"IK failed at step {step}")

    # Update starting position to last position
    robot.q = sol.q  # Set to last successful IK solution

    input("Press enter to finish")

    # Hold final position
    env.hold()
    time.sleep(2)


  
 