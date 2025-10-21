
import swift
from roboticstoolbox import DHLink, DHRobot
from roboticstoolbox import jtraj
import numpy as np
from spatialmath import SE3
from spatialmath.base import *
from ir_support import CylindricalDHRobotPlot
from math import pi
import time
from spatialgeometry import Cuboid, Sphere 
from ir_support import RectangularPrism, line_plane_intersection
from itertools import combinations
from ast import List
from typing import List
 
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

    # # Create an obstacle (cuboid)
    # lwh = [0.35, 0.35, 0.025]       # Python list containing the desired length (X), width (Y), height (Z) of the cuboid object
    # centre = [0.5, 0.5, 0.5]       # Python list containing XYZ centre of cuboid
    # pose = transl(centre)       # Define the pose of the centre of the cuboid
    # # Create prism/cuboid and set desired pose
    # obstacle = Cuboid(scale=lwh, color=[0.0, 1.0, 0.0, 0.5])   # Set colour to green, but with some transparency to see through it (RGBA)
    # obstacle.T = pose
    # env.add(obstacle)              # Add prism to environment

    # Get the prism's mesh properties (vertices, faces, face normals) using RectangularPrism class from ir_support
    # Note: RectangularPrism first 3 parameters are length (X), width (Y), height (Z)
    # vertices, faces, face_normals = RectangularPrism(lwh[0], lwh[1], lwh[2], center=centre).get_data()
    
    input("Press enter to start trajectory")

    # Trajectory parameters
    steps = 300
    delta_t = 0.05
    r = 0.2
    xc, yc, zc = 0.45, 0, 0.3  # center for circle
    start = np.array([0.45, 0.1, 0.3])
    end   = np.array([0.45, 0.1, 0.6])  # end point for straight line

    # Choose trajectory type: "circle" or "line"
    trajectory_type = "circle"  # change to "line" or to "circle" to switch

    # Generate trajectory points
    x = np.zeros([3, steps])
    
    if trajectory_type == "circle":
        theta = 2 * pi / steps
        for step in range(steps):
            x_circle = r * np.cos(step * theta)
            y_circle = r * np.sin(step * theta)
            x[0, step] = xc + 0         # apply rotation as needed
            x[1, step] = yc + y_circle
            x[2, step] = zc + x_circle
    elif trajectory_type == "line":
        for step in range(steps):
            alpha = step / (steps - 1)
            x[:, step] = (1 - alpha) * start + alpha * end
    else:
        raise ValueError("trajectory_type must be 'circle' or 'line'")

    # Move robot along trajectory
    k = 50        # plot cube every k interpolated steps
    interp = 4    # joint interpolation steps
    q_prev = np.array(robot.q, dtype=float)

    for step in range(steps):
        T = SE3(x[0, step], x[1, step], x[2, step])
        sol = robot.ikine_LM(T, q0=q_prev)  # mask=[1,1,1,0,0,0]
        if sol.success:
            q_goal = sol.q
            traj = jtraj(q_prev, q_goal, interp)
            dt_interp = delta_t / interp

            for i, qi in enumerate(traj.q):
                robot.q = qi
                env.step(dt_interp)
                
                # Add sphere every k interpolated steps
                # if i % k == 0:
                #     ee_position = robot.fkine(robot.q).t
                #     sphere = Sphere(radius=0.005, pose=SE3(ee_position), color=[1.0, 0.0, 0.0, 1.0])
                #     env.add(sphere)

            q_prev = q_goal
        else:
            print(f"IK failed at step {step}")


# def get_link_poses(robot:DHRobot,q=None)->List[np.ndarray]|np.ndarray:
#     """
#     :param q robot joint angles
#     :param robot -  seriallink robot model
#     :param transforms - list of transforms
#     """
#     if q is None:
#         return robot.fkine_all().A
#     return robot.fkine_all(q).A


# def is_intersection_point_inside_triangle(intersect_p, triangle_verts):
#     u = triangle_verts[1, :] - triangle_verts[0, :]
#     v = triangle_verts[2, :] - triangle_verts[0, :]

#     uu = np.dot(u, u)
#     uv = np.dot(u, v)
#     vv = np.dot(v, v)

#     w = intersect_p - triangle_verts[0, :]
#     wu = np.dot(w, u)
#     wv = np.dot(w, v)

#     D = uv * uv - uu * vv

#     # Get and test parametric coords (s and t)
#     s = (uv * wv - vv * wu) / D
#     if s < 0.0 or s > 1.0:  # intersect_p is outside Triangle
#         return 0

#     t = (uv * wu - uu * wv) / D
#     if t < 0.0 or (s + t) > 1.0:  # intersect_p is outside Triangle
#         return False

#     return True  # intersect_p is in Triangle


# def is_collision(robot, q_matrix, faces, vertex, face_normals, collisions=[], env=None, return_once_found=True):

#     result = False
#     for i, q in enumerate(q_matrix):
#         # Get the transform of every joint (i.e. start and end of every link)
#         tr = get_link_poses(robot,q)
        
#         # Go through each link and also each triangle face
#         for i in range(np.size(tr,2)-1):
#             for j, face in enumerate(faces):
#                 vert_on_plane = vertex[face][0]
#                 intersect_p, check = line_plane_intersection(face_normals[j], 
#                                                             vert_on_plane, 
#                                                             tr[i][:3,3], 
#                                                             tr[i+1][:3,3])
#                 # list of all triangle combination in a face
#                 triangle_list  = np.array(list(combinations(face,3)),dtype= int)
#                 if check == 1:
#                     for triangle in triangle_list:
#                         if is_intersection_point_inside_triangle(intersect_p, vertex[triangle]):
#                             # Create a red sphere in Swift at the intersection point IF environment passed - if lagging, reduce radius
#                             if env is not None:
#                                 new_collision = Sphere(radius=0.05, color=[1.0, 0.0, 0.0, 1.0])
#                                 new_collision.T = transl(intersect_p[0], intersect_p[1], intersect_p[2])
#                                 env.add(new_collision)
#                                 collisions.append(new_collision)
#                             result = True
#                             if return_once_found:
#                                 return result
#                             break
#     return result

    input("Press enter to finish")
    env.close()

  

 
