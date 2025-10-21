import swift
from spatialmath import SE3
from myCobot280 import myCobot280
from ABB_IRB_120_Cyl import ABB_IRB_120_Cyl
from ir_support.robots import UR3
from RobotTeachGUI import RobotTeachGUI
from math import pi
import numpy as np
import time
from spatialgeometry import Sphere, Cylinder, Cuboid, Mesh
from scipy import linalg
from spatialmath.base import rpy2r, trotx, troty, trotz, tr2rpy
import keyboard
import os
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons
import roboticstoolbox as rtb 
import copy



# Main class for the assessment
class Assessment_2():
    def __init__(self):
        self.env = swift.Swift()
        self.env.launch(realtime=True, show_ground=False, show_grid=False)
        self.robot = None


    def addRobot(self, position=[0, 0, 0], rotation=[0, 0, 0]):
        robot = None
        
        # Set robot position and orientation
        robot.base = SE3(position[0], position[1], position[2]) @ SE3.RPY(rotation[0], rotation[1], rotation[2], unit='deg')
        
        robot.add_to_env(self.env)
        self.env.step(0.02)

    def calculate_reach_volume(self, robot, num_samples):
        # Set robot to a neutral starting position
        robot.q = np.array([0.0, 0.0, -pi/2, 0.0, 0.0, 0.0])
        reach_able_positions = []
        
        for i in range(num_samples):
            q_random = np.random.uniform(
                low=robot.qlim[0, :],
                high=robot.qlim[1, :],
                size=robot.n
            )
            
            T = robot.fkine(q_random)
            pos = T.t
            
            reach_able_positions.append(pos)
            
        reach_able_positions = np.array(reach_able_positions)
        
        x_min, x_max = np.min(reach_able_positions[:, 0]), np.max(reach_able_positions[:, 0])
        y_min, y_max = np.min(reach_able_positions[:, 1]), np.max(reach_able_positions[:, 1])
        z_min, z_max = np.min(reach_able_positions[:, 2]), np.max(reach_able_positions[:, 2])
        
        max_reach = np.max(np.linalg.norm(reach_able_positions, axis=1))
              
        return reach_able_positions, {'x': (x_min, x_max), 'y': (y_min, y_max), 'z': (z_min, z_max)}, max_reach

    def visualize_reach_volume(self, reach_able_positions, point_freq, color=[0.0, 0.0, 1.0, 1.0]):
        points = []
        for i in range(0, len(reach_able_positions), point_freq):
            try:
                new_point = Sphere(radius=0.025, color=color)
                new_point.T = SE3(reach_able_positions[i])
                self.env.add(new_point)
                points.append(new_point)
                
                if i % (point_freq * 10) == 0:
                    self.env.step(0.01)
            except Exception as e:
                print(f"Warning: Failed to create sphere at position {i}: {e}")
                # Add None to maintain index consistency, but we'll filter these out later
                points.append(None)
            
        self.env.step(0.02)
        
        # Filter out None values before returning
        return [p for p in points if p is not None]

    def add_kitchen_environment(self, env, room_size, room_center):
        length, width, height = room_size
        center_x, center_y, center_z = room_center
        environment_objects = []
        
        # Kitchen floor (light gray/beige color)
        floor = Cuboid(
            scale=[length, width, 0.1],  # Thin floor
            color=[0.9, 0.8, 0.7, 1.0],  # Light beige color
            pose=SE3(center_x, center_y, center_z - 0.05)  # Position slightly below center Z
        )
        env.add(floor)
        environment_objects.append(floor)
        
        # Wall colors (light kitchen colors)
        wall_color = [0.95, 0.95, 0.9, 1.0]  # Light gray with slight transparency
        
        # Back wall (behind robot)
        back_wall = Cuboid(
            scale=[0.1, width, height],
            color=wall_color,
            pose=SE3(center_x - length/2, center_y, center_z + height/2)
        )
        env.add(back_wall)
        environment_objects.append(back_wall)
        
        # Front wall (in front of robot)
        front_wall = Cuboid(
            scale=[0.1, width, height],
            color=wall_color,
            pose=SE3(center_x + length/2, center_y, center_z + height/2)
        )
        env.add(front_wall)
        environment_objects.append(front_wall)
        
        # Left wall
        left_wall = Cuboid(
            scale=[length, 0.1, height],
            color=wall_color,
            pose=SE3(center_x, center_y - width/2, center_z + height/2)
        )
        env.add(left_wall)
        environment_objects.append(left_wall)
        
        # Right wall
        right_wall = Cuboid(
            scale=[length, 0.1, height],
            color=wall_color,
            pose=SE3(center_x, center_y + width/2, center_z + height/2)
        )
        env.add(right_wall)
        environment_objects.append(right_wall)
        
        print(f"Kitchen environment added: {length}m x {width}m x {height}m room at center {room_center}")
        return environment_objects

    def add_stl_object(self, env, file_path, position=[0, 0, 0], rotation=[0, 0, 0], 
                      scale=1.0, color=None, name="Mesh_Object"):

        # Check if file exists
        if not os.path.exists(file_path):
            print(f"Warning: Mesh file not found at {file_path}")
            return None
        
        print(f"Loading mesh from: {file_path}")
        print(f"File size: {os.path.getsize(file_path)} bytes")
        
        try:
            print("Creating Mesh object...")
            # Create mesh object
            mesh = Mesh(filename=file_path)
            print("Mesh created successfully")
            
            print("Applying transformations...")
            # Apply transformations
            # 1. Create rotation matrix from roll, pitch, yaw (in degrees)
            roll, pitch, yaw = np.radians(rotation)
            rotation_matrix = trotx(roll) @ troty(pitch) @ trotz(yaw)
            
            # 2. Create translation matrix
            translation_matrix = SE3(position[0], position[1], position[2])
            
            # 3. Combine rotation and translation
            rotation_se3 = SE3(rotation_matrix)
            transform_matrix = translation_matrix @ rotation_se3
            
            # 4. Apply transform
            mesh.T = transform_matrix.A
            print("Transform applied")
            
            # 5. Only set color if provided
            if color is not None:
                mesh.color = color
                print(f"Color applied: {color}")
            else:
                print("Preserving original colors")
            
            # Set the name attribute on the mesh object
            mesh.name = name
            
            print("Adding to environment...")
            # Add to environment
            env.add(mesh)
            print("Added to environment")
            
            print(f"Added {name} at position {position}, rotation {rotation} degrees")
            return mesh
            
        except Exception as e:
            print(f"Error creating mesh for {name}: {e}")
            import traceback
            traceback.print_exc()
            return None

    def add_kitchen_furniture_stls(self, env):
        furniture_objects = []
        
        # Kitchen sink (if STL file exists)
        sink_path = r"C:\Users\Jordan\Downloads\Industrial_Robotics\Assessment_2_Clean\kitchen_sink_0.1Scale.stl"
        plate_path = r"C:\Users\Jordan\Downloads\Industrial_Robotics\Assessment_2_Clean\plate.stl"
        stack_path = r"C:\Users\Jordan\Downloads\Industrial_Robotics\Assessment_2_Clean\plate_stack.stl"
        drying_rack_path = r"C:\Users\Jordan\Downloads\Industrial_Robotics\Assessment_2_Clean\drying_rack.stl"
        storage_shelf_path = r"C:\Users\Jordan\Downloads\Industrial_Robotics\Assessment_2_Clean\storage_shelf.stl"
        emergency_button_path = r"C:\Users\Jordan\Downloads\Industrial_Robotics\Assessment_2_Clean\emergency_button.stl"
        cone_barrier_path = r"C:\Users\Jordan\Downloads\Industrial_Robotics\Assessment_2_Clean\cone_barrier.stl"
        fire_extinguisher_path = r"C:\Users\Jordan\Downloads\Industrial_Robotics\Assessment_2_Clean\fire_extinguisher.stl"
        hazard_sign_path = r"C:\Users\Jordan\Downloads\Industrial_Robotics\Assessment_2_Clean\hazard_sign.stl"
        
        
        if os.path.exists(cone_barrier_path):
            cone_barrier = self.add_stl_object(
                env=env,
                file_path=cone_barrier_path,
                position=[6.5, 2, 0.05],  
                rotation=[0, 0, 180],  # 
                color=[1.0, 0.5, 0.0, 1.0],  # Orange color for cone barrier [1.0, 0.5, 0.0, 1.0]
                name="Cone Barrier"
            )
            if cone_barrier:
                furniture_objects.append(cone_barrier)
        
        if os.path.exists(fire_extinguisher_path):
            fire_extinguisher = self.add_stl_object(
                env=env,
                file_path=fire_extinguisher_path,
                position=[11, 1, 0.05],  
                rotation=[0, 0, 0],  
                color=[1.0, 0.0, 0.0, 1.0],  # Red color for fire extinguisher [1.0, 0.0, 0.0, 1.0]
                name="Fire Extinguisher"
            )
            if fire_extinguisher:
                furniture_objects.append(fire_extinguisher)
        
        if os.path.exists(hazard_sign_path):
            hazard_sign = self.add_stl_object(
                env=env,
                file_path=hazard_sign_path,
                position=[7.5, 3, 0.05], 
                rotation=[0, 0, -180],  
                color=[1.0, 1.0, 0.0, 1.0],  # Yellow color for hazard sign [1.0, 1.0, 0.0, 1.0]
                name="Hazard Sign"
            )
            if hazard_sign:
                furniture_objects.append(hazard_sign)
        
        if os.path.exists(sink_path):
            sink = self.add_stl_object(
                env=env,
                file_path=sink_path,
                position=[7.5, 1, 0.05],  # Against back wall, counter height
                rotation=[0, 0, 180],  # rotation
                color=None,  
                name="Kitchen Sink"
            )
            if sink:
                furniture_objects.append(sink)
                
        if os.path.exists(emergency_button_path):
            emergency_button = self.add_stl_object(
                env=env,
                file_path=emergency_button_path,
                position=[11, 0.5, 2],  # Against back wall, counter height
                rotation=[180, 0, 0],  # No rotation
                color=[1.0, 0.0, 0.0, 1.0],  # Red color for emergency button [1.0, 0.0, 0.0, 1.0]
                name="Emergency Button"
            )
            if emergency_button:
                furniture_objects.append(emergency_button)
        
        if os.path.exists(storage_shelf_path):
            storage_shelf = self.add_stl_object(
                env=env,
                file_path=storage_shelf_path,
                position=[5, 1, 0.05],  # Against back wall, counter height
                rotation=[0, 0, 180],  # No rotation
                color=[0.5, 0.25, 0.0, 1.0],  # brown wood color for storage shelf [0.5, 0.25, 0.0, 1.0]
                name="Storage Shelf"
            )
            if storage_shelf:
                furniture_objects.append(storage_shelf)
        
        if os.path.exists(drying_rack_path):
            drying_rack = self.add_stl_object(
                env=env,
                file_path=drying_rack_path,
                position=[6.5, 1, 1.2],  # Against back wall, counter height
                rotation=[0, 0, 180],  # No rotation
                color=[1.0, 1.0, 1.0, 1.0],  # white color for plate [1.0, 1.0, 1.0, 1.0]
                name="Drying Rack"
            )
            if drying_rack:
                furniture_objects.append(drying_rack)
        
        if os.path.exists(stack_path):
            stack = self.add_stl_object(
                env=env,
                file_path=stack_path,
                position=[8.7, 0.8, 1.2],  # Against back wall, counter height
                rotation=[0, 0, 0],  # No rotation
                color=[1.0, 1.0, 1.0, 1.0],  # white color for plate [1.0, 1.0, 1.0, 1.0]
                name="Plate Stack"
            )
            if stack:
                furniture_objects.append(stack)
        
        # Store the plate mesh for later use
        self.plate_mesh = None
        if os.path.exists(plate_path):
            plate_1 = self.add_stl_object(
                env=env,
                file_path=plate_path,
                position=[8.2, 1.25, 1.35],  # Against back wall, counter height
                rotation=[180, 0, 0],  # No rotation
                color=[1.0, 1.0, 1.0, 1.0],  # white color for plate [1.0, 1.0, 1.0, 1.0]
                name="Plate_1"
            )
            if plate_1:
                furniture_objects.append(plate_1)
                self.plate_mesh = plate_1  # Store reference for attachment
                print("✅ Plate mesh stored for attachment")
        
        return furniture_objects


    def demonstrate_robot_gui(self):
        # Create GUI for Robot 3 (index 2)
        if len(self.robots) >= 3:
            robot3 = self.robots[1]  # Robot 3 (index 2)
            
            print("GUI created successfully!")
            print("Use the GUI controls to move Robot 3.")
            print("Press 'Enter' in the console when you're done with the GUI demonstration.")
            
            # Try to create and run GUI
            try:
                gui = RobotTeachGUI(robot3, self.env)
                gui.run()
            except Exception as e:
                print(f"GUI error: {e}")
                print("Continuing without GUI...")
                # Provide a simple console-based alternative
                self.simple_robot_control(robot3)
            
            # Wait for user input to continue
            try:
                input("Press Enter to continue to the main demonstration...")
            except (EOFError, KeyboardInterrupt):
                print("Continuing automatically...")
            
            
    def execute_robot_movements(self, robot_index, target_poses, trajectory_steps=50, step_delay=0.05, 
                        plate_mesh=None, attachment_events=None, attachment_offset=[0, 0, 0], obstacles=None):
        robot_names = ["ABB IRB 120", "myCobot280", "UR3"]
        
        print("Starting robot movement demonstration...")
        print(f"Controlling Robot {robot_index + 1} ({robot_names[robot_index]})")
        print(f"Planning {len(target_poses)} movements...")
        
        

        # Get the selected robot
        robot = self.robots[robot_index]
        
        # Initialize attachment state
        plate_attached = False
        attachment_events = attachment_events or []

        # Execute each target pose
        for movement_num, target_config in enumerate(target_poses):
            print(f"\n--- Movement {movement_num + 1}/{len(target_poses)} ---")
            
            # Check for attachment events at start of movement
            for event_movement, action in attachment_events:
                if event_movement == movement_num:
                    if action == 'attach' and plate_mesh and not plate_attached:
                        self.attach_plate_to_robot(robot_index, plate_mesh, attachment_offset)
                        plate_attached = True
                    elif action == 'detach' and plate_mesh and plate_attached:
                        self.detach_plate_from_robot(plate_mesh)
                        plate_attached = False
            
            # Parse target configuration
            x, y, z, roll, pitch, yaw = target_config
            target_pose = SE3(x, y, z) * SE3.RPY(roll, pitch, yaw, unit='deg')
            
            print(f"Target pose: Position({x:.2f}, {y:.2f}, {z:.2f}), Orientation({roll}°, {pitch}°, {yaw}°)")
            
            # COLLISION AVOIDANCE: Only for myCobot280 (robot_index == 1)
            if robot_index == 1 and obstacles:
                print("🔍 Checking trajectory for collisions...")
                
                # Check if the entire trajectory path would cause collision
                trajectory_collision = self.check_trajectory_collision(robot, target_pose, obstacles)
                
                if trajectory_collision:
                    print("❌ COLLISION DETECTED in trajectory! Adjusting path...")
                    # Get a safe pose instead
                    safe_pose_config = self.get_safe_pose(target_config, obstacles)
                    x, y, z, roll, pitch, yaw = safe_pose_config
                    target_pose = SE3(x, y, z) * SE3.RPY(roll, pitch, yaw, unit='deg')
                    print(f"✅ Using safe pose: Position({x:.2f}, {y:.2f}, {z:.2f})")
                else:
                    print("✅ Trajectory path is clear")
            
            # Get current state
            q_start = robot.q.copy()
            current_pose = robot.fkine(robot.q)
            print(f"Current position: ({current_pose.t[0]:.2f}, {current_pose.t[1]:.2f}, {current_pose.t[2]:.2f})")
            
            # Check reachability
            current_distance = np.linalg.norm(current_pose.t)
            target_distance = np.linalg.norm(target_pose.t)
            print(f"Distance check: Current={current_distance:.2f}m, Target={target_distance:.2f}m")
            
            if target_distance <= current_distance * 1.5:
                print("✅ Target appears REACHABLE")
            else:
                print("❌ Target may be UNREACHABLE - proceeding anyway...")
            
            # Calculate inverse kinematics (only once now)
            result = robot.ikine_LM(target_pose, q0=robot.q)
            if not result.success:
                print("IK failed - trying alternative approach...")
                result = robot.ikine_LM(target_pose, q0=np.zeros(robot.n))
                if not result.success:
                    print("❌ IK completely failed - skipping this movement")
                    continue
            
            q_target = result.q
            print(f"✅ IK solution found: {q_target}")
            
            # Plan trajectory
            trajectory = rtb.jtraj(q_start, q_target, trajectory_steps)
            print(f"Trajectory planned: {len(trajectory.q)} steps")
            
            # Execute trajectory
            print("Executing movement...")
            for i, q in enumerate(trajectory.q):
                robot.q = q
                self.env.step(0.05)
                time.sleep(step_delay)
                
                # Update plate position if attached
                if plate_attached and plate_mesh:
                    self.update_plate_position(robot_index, plate_mesh, attachment_offset)
                
                # Progress feedback
                if i % 10 == 0:
                    current_pose = robot.fkine(robot.q)
                    print(f"  Step {i}")
            
            print(f"✅ Movement {movement_num + 1} completed!")

        print("\n All movements completed!")
        
        
    def check_trajectory_collision(self, robot, target_pose, obstacles):
        
        
        # Try to solve IK for the target pose
        result = robot.ikine_LM(target_pose, q0=robot.q)
        if not result.success:
            print("⚠️ IK failed for collision check - assuming safe")
            return False
        
        # Create a temporary robot for collision checking
        temp_robot = copy.deepcopy(robot)
        temp_robot.q = result.q
        
        # Check collision at the target pose
        if self.check_collision_with_bounding_boxes(temp_robot, obstacles):
            print(f"❌ Target pose collision detected")
            return True
        
        # Check collision along the trajectory path
        # Plan a trajectory from current to target
        try:
            trajectory = rtb.jtraj(robot.q, result.q, 20)  # Check 20 points along path
            
            for i, q in enumerate(trajectory.q):
                temp_robot.q = q
                if self.check_collision_with_bounding_boxes(temp_robot, obstacles):
                    print(f"❌ Trajectory collision detected at step {i}")
                    return True
                    
        except Exception as e:
            print(f"⚠️ Trajectory planning failed: {e}")
            return False
        
        return False
        
    
    def attach_plate_to_robot(self, robot_index, plate_mesh, attachment_offset=[0, 0, 0]):
        robot = self.robots[robot_index]
        
        # Get current end effector transform
        ee_transform = robot.fkine(robot.q)
        
        # Apply attachment offset
        offset_transform = SE3(attachment_offset[0], attachment_offset[1], attachment_offset[2])
        
        # Set plate transform to follow end effector
        plate_mesh.T = ee_transform.A @ offset_transform.A
        
        print(f"✅ Plate attached to Robot {robot_index + 1} end effector")
        return plate_mesh

    def detach_plate_from_robot(self, plate_mesh):
        print(f"✅ Plate detached from robot")
        return plate_mesh

    def update_plate_position(self, robot_index, plate_mesh, attachment_offset=[0, 0, 0]):
        robot = self.robots[robot_index]
        ee_transform = robot.fkine(robot.q)
        offset_transform = SE3(attachment_offset[0], attachment_offset[1], attachment_offset[2])
        plate_mesh.T = ee_transform.A @ offset_transform.A
        
    def execute_rmrc_circle(self, robot_index, center_position, radius=0.2, steps=200, delta_t=0.05):
        robot = self.robots[robot_index]
        robot_names = ["ABB IRB 120", "myCobot280", "UR3"]
        
        print(f"Starting RMRC circle with Robot {robot_index + 1} ({robot_names[robot_index]})")
        print(f"Circle center: {center_position}")
        print(f"Circle radius: {radius}m")
        
        # STEP 1: Move robot to the desired circle center first
        print("Moving robot to circle center...")
        target_pose = SE3(center_position[0], center_position[1], center_position[2])
        result = robot.ikine_LM(target_pose, q0=robot.q)
        if result.success:
            # Plan trajectory to circle center
            trajectory = rtb.jtraj(robot.q, result.q, 50)
            for q in trajectory.q:
                robot.q = q
                self.env.step(0.05)
                time.sleep(0.05)
            print("✅ Robot moved to circle center")
        else:
            print("❌ Failed to move to circle center - using current position")
        
        # STEP 2: Now generate circle waypoints around the robot's current position
        x = np.zeros([3, steps])
        current_position = robot.fkine(robot.q).t  # Get current end effector position
        center = np.array([current_position[0], current_position[1], current_position[2]])

        print(f"Drawing circle around current position: {center}")

        for i in range(steps):
            # Change this line to multiply by number of circles
            num_circles = 4  # Draw 2 complete circles
            theta = 2 * np.pi * i / steps * num_circles  # This will draw 2 circles
            x[0, i] = center[0] + radius * np.cos(theta)  # X coordinate
            x[1, i] = center[1] + radius * np.sin(theta)  # Y coordinate
            x[2, i] = center[2]                          # Z coordinate (fixed)

        # Initialize joint angles using current robot position
        q_matrix = np.zeros([steps, robot.n])
        q_matrix[0, :] = robot.q  # Start from current position
        print("✅ Starting RMRC from current robot position")
        
        # RMRC loop
        for i in range(steps - 1):
            xdot = (x[:, i + 1] - x[:, i]) / delta_t  # Cartesian velocity
            J = robot.jacob0(q_matrix[i, :])          # Full Jacobian at current joint config
            J_pos = J[:3, :]                         # Use only position part
            try:
                q_dot = np.linalg.pinv(J_pos) @ xdot  # Joint velocities
            except np.linalg.LinAlgError:
                print(f"Jacobian not invertible at step {i}")
                q_dot = np.zeros(robot.n)
            q_matrix[i + 1, :] = q_matrix[i, :] + delta_t * q_dot  # Integrate
        
        # Animate in Swift
        print("Executing RMRC circle movement...")
        for i, q in enumerate(q_matrix):
            robot.q = q
            ee_position = robot.fkine(q).A[:3, 3]
            
            # Create visual marker for end effector path
            # if i % 10 == 0:  # Add marker every 10 steps to avoid clutter
            cuboid = Cuboid(scale=[0.02, 0.02, 0.02], pose=SE3(ee_position), color=[1, 0, 0, 1.0])
            self.env.add(cuboid)
            
            self.env.step(delta_t)
            time.sleep(delta_t)
        
        print("✅ RMRC circle movement completed!")
        
    
    def create_obstacle_bounding_boxes(self):
        obstacles = [
            {
                'name': 'storage_shelf',
                'position': [4.8, 1.01, 0.05],      # From   existing code
                'size': [1.0, 1.4, 5.0],       # [width, depth, height] - measure   actual shelf
                'visible': True
            },
            # {
            #     'name': 'test_curtain', 
            #     'position': [5.9, 1, 2.2],    # From   existing code
            #     'size': [0.5, 1.0, 0.4],       
            #     'visible': True
            # }
        ]
        return obstacles
    
    def visualize_bounding_boxes(self, obstacles):
        bounding_box_objects = []
        
        for obstacle in obstacles:
            if obstacle.get('visible', False):  # Only show if marked as visible
                pos = obstacle['position']
                size = obstacle['size']
                
                # Create a semi-transparent cuboid for the bounding box
                bounding_box = Cuboid(
                    scale=size,  # [width, depth, height]
                    pose=SE3(pos[0], pos[1], pos[2]),  # Center position
                    color=[1.0, 0.0, 0.0, 0.3]  # Red with 30% transparency
                )
                
                self.env.add(bounding_box)
                bounding_box_objects.append(bounding_box)
                print(f"✅ Added visible bounding box for {obstacle['name']}")
        
        return bounding_box_objects
    
    
    def check_collision_with_bounding_boxes(self, robot, obstacles):
        """Check if robot end effector is inside any bounding box"""
        ee_pos = robot.fkine(robot.q).t  # Get end effector position
        
        print(f"\n🔍 COLLISION CHECK:")
        print(f"Robot end effector position: [{ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}]")
        
        for obstacle in obstacles:
            pos = obstacle['position']  # Box center
            size = obstacle['size']      # Box dimensions
            
            print(f"\nChecking {obstacle['name']}:")
            print(f"  Box center: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
            print(f"  Box size: [{size[0]:.3f}, {size[1]:.3f}, {size[2]:.3f}]")
            
            # Calculate box boundaries
            x_min = pos[0] - size[0]/2
            x_max = pos[0] + size[0]/2
            y_min = pos[1] - size[1]/2
            y_max = pos[1] + size[1]/2
            z_min = pos[2] - size[2]/2
            z_max = pos[2] + size[2]/2
            
            print(f"  X range: {x_min:.3f} to {x_max:.3f}")
            print(f"  Y range: {y_min:.3f} to {y_max:.3f}")
            print(f"  Z range: {z_min:.3f} to {z_max:.3f}")
            
            # Check if end effector is inside the box
            inside_x = x_min <= ee_pos[0] <= x_max
            inside_y = y_min <= ee_pos[1] <= y_max
            inside_z = z_min <= ee_pos[2] <= z_max
            
            print(f"  Inside X: {inside_x} (EE X: {ee_pos[0]:.3f})")
            print(f"  Inside Y: {inside_y} (EE Y: {ee_pos[1]:.3f})")
            print(f"  Inside Z: {inside_z} (EE Z: {ee_pos[2]:.3f})")
            
            if inside_x and inside_y and inside_z:
                print(f"  ❌ COLLISION DETECTED with {obstacle['name']}!")
                return True
            else:
                print(f"  ✅ No collision with {obstacle['name']}")
        
        return False
    
    
    
    def get_safe_pose(self, target_pose, obstacles):
        x, y, z, roll, pitch, yaw = target_pose
        
        print(f"🔧 TESTING MULTIPLE RANDOM POSES for target: [{x:.2f}, {y:.2f}, {z:.2f}]")
        
        # Get current robot position as reference
        current_pos = self.robots[1].fkine(self.robots[1].q).t
        print(f"Current robot position: [{current_pos[0]:.2f}, {current_pos[1]:.2f}, {current_pos[2]:.2f}]")
        
        # Generate multiple random poses around current position
        num_tests = 20  # Test 20 different poses
        test_poses = []
        
        for i in range(num_tests):
            # Random offsets in different directions
            offset_x = np.random.uniform(-1.5, 1.5)  # ±1.5m in X
            offset_y = np.random.uniform(-1.5, 1.5)  # ±1.5m in Y  
            offset_z = np.random.uniform(-0.5, 1.0)   # ±0.5m to +1m in Z
            
            # Create test pose relative to current position
            test_x = current_pos[0] + offset_x
            test_y = current_pos[1] + offset_y
            test_z = current_pos[2] + offset_z
            
            # Keep original orientation
            test_pose = [test_x, test_y, test_z, roll, pitch, yaw]
            test_poses.append(test_pose)
        
        print(f"Generated {len(test_poses)} test poses")
        
        # Test each pose for collision and IK success
        valid_poses = []
        
        for i, test_pose in enumerate(test_poses):
            print(f"  Testing pose {i+1}: [{test_pose[0]:.2f}, {test_pose[1]:.2f}, {test_pose[2]:.2f}]")
            
            # Create temporary robot for testing
            import copy
            temp_robot = copy.deepcopy(self.robots[1])
            
            try:
                # Try IK for this pose
                target_se3 = SE3(test_pose[0], test_pose[1], test_pose[2]) * SE3.RPY(test_pose[3], test_pose[4], test_pose[5], unit='deg')
                result = temp_robot.ikine_LM(target_se3, q0=temp_robot.q)
                
                if result.success:
                    temp_robot.q = result.q
                    
                    # Check for collision
                    if not self.check_collision_with_bounding_boxes(temp_robot, obstacles):
                        # Calculate distance to original target (closer is better)
                        distance_to_target = np.sqrt(
                            (test_pose[0] - x)**2 + 
                            (test_pose[1] - y)**2 + 
                            (test_pose[2] - z)**2
                        )
                        
                        valid_poses.append({
                            'pose': test_pose,
                            'distance': distance_to_target,
                            'index': i
                        })
                        print(f"    ✅ Valid pose found! Distance to target: {distance_to_target:.2f}m")
                    else:
                        print(f"    ❌ Collision detected")
                else:
                    print(f"    ❌ IK failed")
                    
            except Exception as e:
                print(f"    ❌ Error: {e}")
        
        # Choose the best valid pose
        if valid_poses:
            # Sort by distance to target (closest first)
            valid_poses.sort(key=lambda x: x['distance'])
            best_pose = valid_poses[0]['pose']
            
            print(f"✅ BEST POSE SELECTED: [{best_pose[0]:.2f}, {best_pose[1]:.2f}, {best_pose[2]:.2f}]")
            print(f"   Distance to original target: {valid_poses[0]['distance']:.2f}m")
            print(f"   Found {len(valid_poses)} valid poses out of {num_tests} tested")
            
            return best_pose
        else:
            print("❌ NO VALID POSES FOUND! Using fallback position...")
            # Fallback: move far away from obstacles
            return [x + 2.0, y, z + 1.0, roll, pitch, yaw]
        
        
    def generate_smart_test_poses(self, current_pos, target_pos, num_poses=20):
        """Generate smarter test poses that are more likely to be valid"""
        test_poses = []
        
        # Strategy 1: Poses that move toward target but avoid obstacles
        for i in range(num_poses // 2):
            # Interpolate between current and target, but with random offsets
            alpha = np.random.uniform(0.3, 0.8)  # 30-80% toward target
            
            test_x = current_pos[0] + alpha * (target_pos[0] - current_pos[0]) + np.random.uniform(-0.5, 0.5)
            test_y = current_pos[1] + alpha * (target_pos[1] - current_pos[1]) + np.random.uniform(-0.5, 0.5)
            test_z = current_pos[2] + alpha * (target_pos[2] - current_pos[2]) + np.random.uniform(-0.3, 0.3)
            
            test_poses.append([test_x, test_y, test_z, 0, 180, 0])
        
        # Strategy 2: Completely random poses around current position
        for i in range(num_poses // 2):
            offset_x = np.random.uniform(-1.0, 1.0)
            offset_y = np.random.uniform(-1.0, 1.0)
            offset_z = np.random.uniform(-0.3, 0.5)
            
            test_x = current_pos[0] + offset_x
            test_y = current_pos[1] + offset_y
            test_z = current_pos[2] + offset_z
            
            test_poses.append([test_x, test_y, test_z, 0, 180, 0])
        
        return test_poses
    
    
    
    

    def run(self): 
        # Define robot positions and rotations
        robot_configs = [
            {"position": [8, 0.5, 1.7], "rotation": [-90, 0, 0]},      # Robot 1 - Taj = inded 0
            {"position": [6, -0.5, 2.5], "rotation": [-90, 0, 0]},      # Robot 3 - Jordan = index 1
            {"position": [7.25, 0.5, 1.7], "rotation": [-90, 0, 0]},     # Robot 2 - UR3 = index 2
        ]
        # Create all robots 
        self.robots = []
        robot_classes = [ABB_IRB_120_Cyl, myCobot280, UR3]
        for i, config in enumerate(robot_configs):
            # Select robot class (fall back to myCobot280 if index out of range)
            RobotClass = robot_classes[i] if i < len(robot_classes) else myCobot280
            robot = RobotClass()
            # Set base pose
            robot.base = SE3(config["position"][0], config["position"][1], config["position"][2]) @ SE3.RPY(config["rotation"][0], config["rotation"][1], config["rotation"][2], unit='deg')
            # Add to environment (some robot classes may have different add signatures; use add_to_env if present)
            try:
                robot.add_to_env(self.env)
            except Exception:
                # Fallback: try env.add(robot) if add_to_env not available
                try:
                    self.env.add(robot)
                except Exception as e:
                    print(f"Warning: could not add robot {i+1} to env: {e}")
            self.robots.append(robot)
            print(f"Added Robot {i+1} ({RobotClass.__name__}) at position {config['position']} with rotation {config['rotation']}")
        self.env.step(0.02)
        
        
        # Load the kitchen environment and furniture
        print("Loading kitchen environment...")
        self.add_kitchen_environment(self.env, room_size=[15, 15, 15], room_center=[7.5, 7.5, 0.05])
        self.env.set_camera_pose([7.5, 5, 2], [7.5, 1, 2])
        print("Kitchen environment added")
        self.env.step(0.02)
        print("Loading kitchen furniture...")
        furniture_objects = self.add_kitchen_furniture_stls(self.env)
        print("Kitchen furniture added")
        self.env.step(0.02)
        
        # Now demonstrate the advanced GUI for Robot 3 control with full environment
        print("Environment fully loaded!")
        # self.demonstrate_robot_gui()
        
        # Create obstacle bounding boxes
        obstacles = self.create_obstacle_bounding_boxes()
        
        # VISUALIZE the bounding boxes
        print("Adding visible bounding boxes to environment...")
        bounding_box_objects = self.visualize_bounding_boxes(obstacles)
        self.env.step(0.02)  # Update environment to show boxes
           
           
           
        print("Starting Washing demonstration...")
        
        # # First parameter is the robot index (0=ABB_IRB_120, 1=myCobot280, 2=UR3), second parameter is the sequence
        # self.execute_robot_movements(0, plate_sequence) 
        # Use the existing plate mesh that was stored during furniture creation
        plate_mesh = getattr(self, 'plate_mesh', None)
        
        # Define movement sequence
        plate_sequence = [
            [8.20, 1.25, 1.35, 0, 180, 0],      # Move to plate position = 0
            [8.20, 1.25, 1.63, 0, 180, 0],        # Move to sink = 1
            [8.00, 1.00, 1.63, 0, 180, 0],        # needed for detatch step
            [7.48, 1.00, 1.55, 0, 180, 0],
            [7.48, 1.00, 1.55, 0, 180, 0],        # needed for detatch step
        ]

        # Define attachment events (movement_number, action)
        # Movement 0: Attach plate when robot reaches plate
        # Movement 2: Detach plate when robot reaches drying rack
        attachment_events = [
            (1, 'attach'),   # Attach at start of movement 1 (after reaching plate)
            (4, 'detach'),   # Detach at start of movement 3 (after reaching drying rack)
        ]

        # Attachment offset - adjust this to match where   robot touches the plate
        # [x, y, z] offset from end effector center
        attachment_offset = [0, 0, 0]  # 5cm below end effector (adjust as needed)

        # Execute movements with plate attachment
        if plate_mesh:
            self.execute_robot_movements(
                robot_index=0, 
                target_poses=plate_sequence, 
                trajectory_steps=50, 
                step_delay=0.05,
                plate_mesh=plate_mesh,
                attachment_events=attachment_events,
                attachment_offset=attachment_offset,
                obstacles=None      
            )
        
        # RMRC Circle movement with Robot 2 (UR3)
        print("\n" + "="*50)
        print("Starting RMRC Circle demonstration...")
        
        # Circle parameters
        circle_center = [7.5, 1.00, 1.5]  # [x, y, z] center of circle, ur3 mount = [7.25, 0.5, 1.7]
        circle_radius = 0.05              # Radius in meters
        circle_steps = 100                # Number of steps (higher = smoother)
        
        # Execute RMRC circle with Robot 2 (UR3)
        self.execute_rmrc_circle(
            robot_index=2,                    # Robot 2 (UR3)
            center_position=circle_center,   # Center of circle
            radius=circle_radius,           # Radius of circle
            steps=circle_steps,             # Number of steps
            delta_t=0.05                    # Time step
        )

        # Jtraj to move UR3
        plate_mesh = getattr(self, 'plate_mesh', None)
               
        # Define movement sequence
        plate_sequence = [
            [7.48, 1.00, 1.50, 0, 0, 0],      # Move to plate position = 0
            [6.90, 1.00, 1.95, 0, 0, 0],        # Move to sink = 1
            [6.90, 1.00, 1.95, 0, 0, 0],        # needed for detatch step
        ]

        # Define attachment events (movement_number, action)
        # Movement 0: Attach plate when robot reaches plate
        # Movement 2: Detach plate when robot reaches drying rack
        attachment_events = [
            (1, 'attach'),   # Attach at start of movement 1 (after reaching plate)
            (2, 'detach'),   # Detach at start of movement 3 (after reaching drying rack)
        ]

        # Attachment offset - adjust this to match where   robot touches the plate
        # [x, y, z] offset from end effector center
        attachment_offset = [0, 0, 0]  # 5cm below end effector (adjust as needed)

        # Execute movements with plate attachment
        if plate_mesh:
            self.execute_robot_movements(
                robot_index=2, 
                target_poses=plate_sequence, 
                trajectory_steps=50, 
                step_delay=0.05,
                plate_mesh=plate_mesh,
                attachment_events=attachment_events,
                attachment_offset=attachment_offset,
                obstacles=None   
            )
            
    # Jtraj to move myCobot280
        plate_mesh = getattr(self, 'plate_mesh', None)
        
               
        # Define movement sequence
        plate_sequence = [
            [6.90, 1.00, 2.00, 0, 180, 0],      # Move to plate position = 0
            [5.00, 1.00, 2.60, 0, 180, 0],        # Move to sink = 1
            [6.00, 1.00, 2.60, 0, 180, 0],        # needed for detatch step
        ]

        
        # Define attachment events (movement_number, action)
        # Movement 0: Attach plate when robot reaches plate
        # Movement 2: Detach plate when robot reaches drying rack
        attachment_events = [
            (1, 'attach'),   # Attach at start of movement 1 (after reaching plate)
            (2, 'detach'),   # Detach at start of movement 3 (after reaching drying rack)
        ]

        # Attachment offset - adjust this to match where   robot touches the plate
        # [x, y, z] offset from end effector center
        attachment_offset = [0, 0, 0]  # 5cm below end effector (adjust as needed)

        # Execute movements with plate attachment
        if plate_mesh:
            self.execute_robot_movements(
                robot_index=1, 
                target_poses=plate_sequence, 
                trajectory_steps=50, 
                step_delay=0.05,
                plate_mesh=plate_mesh,
                attachment_events=attachment_events,
                attachment_offset=attachment_offset,
                obstacles=obstacles   
            )




if __name__ == "__main__":
    try:
        program = Assessment_2()
        program.run()
        program.env.hold()
        time.sleep(3)
        
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user.")
    except Exception as e:
        print(f"Error during simulation: {e}")
        import traceback
        traceback.print_exc()