import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons
from spatialmath import SE3
from spatialmath.base import rpy2r, tr2rpy
from scipy import linalg


class RobotTeachGUI:
    def __init__(self, robot, env):
        self.robot = robot
        self.env = env
        self.fig = None
        self.sliders = []
        self.labels = []
        self.cartesian_sliders = []
        self.cartesian_labels = []
        self.control_mode = "joint"  # "joint" or "cartesian"
        self.running = True
        
        # Store initial robot configuration
        self.initial_q = robot.q.copy()
        
        # Create GUI
        self.create_gui()
        
    def create_gui(self):
        plt.close('all')
        
        # Set matplotlib backend to avoid threading issues
        import matplotlib
        matplotlib.use('TkAgg')  # Use TkAgg backend which is more stable
        
        self.fig = plt.figure(figsize=(12, 8))
        self.fig.suptitle('Robot 3 Advanced Teach GUI - Joint & Cartesian Control', fontsize=14, fontweight='bold')
        
        # Create control mode selector
        self.create_mode_selector()
        
        # Create joint control panel
        self.create_joint_controls()
        
        # Create Cartesian control panel
        self.create_cartesian_controls()
        
        # Create action buttons
        self.create_action_buttons()
        
        # Create status display
        self.create_status_display()
        
        plt.tight_layout()
        plt.show(block=False)
        
    def create_mode_selector(self):
        """Create radio buttons for control mode selection"""
        ax_radio = plt.axes([0.02, 0.85, 0.15, 0.1])
        self.radio = RadioButtons(ax_radio, ('Joint', 'Cartesian'))
        self.radio.on_clicked(self.mode_changed)
        
    def create_joint_controls(self):
        """Create joint angle sliders"""
        # Joint control title
        plt.figtext(0.02, 0.75, 'Joint Control (Degrees)', fontsize=12, fontweight='bold')
        
        # Create sliders for each joint
        slider_height = 0.03
        slider_width = 0.4
        slider_spacing = 0.05
        
        for i in range(self.robot.n):
            # Convert joint limits from radians to degrees
            qlim_deg = np.degrees(self.robot.qlim[:, i])
            
            # Create slider
            ax_slider = plt.axes([0.02, 0.65 - i * slider_spacing, slider_width, slider_height])
            slider = Slider(ax_slider, f'J{i+1}', qlim_deg[0], qlim_deg[1], 
                          valinit=np.degrees(self.robot.q[i]), valstep=1.0)
            slider.on_changed(lambda val, idx=i: self.joint_slider_callback(val, idx))
            self.sliders.append(slider)
            
            # Create label for current value
            label_text = f'Joint {i+1}: {np.degrees(self.robot.q[i]):.1f}°'
            label = plt.figtext(0.45, 0.65 - i * slider_spacing + 0.01, label_text, fontsize=10)
            self.labels.append(label)
            
    def create_cartesian_controls(self):
        """Create Cartesian position and orientation controls"""
        # Cartesian control title
        plt.figtext(0.55, 0.75, 'Cartesian Control', fontsize=12, fontweight='bold')
        
        # Get current end-effector pose
        current_pose = self.robot.fkine(self.robot.q)
        current_pos = current_pose.t.flatten()
        current_rpy = tr2rpy(current_pose.R, unit='deg')
        
        # Position sliders (X, Y, Z)
        pos_labels = ['X (m)', 'Y (m)', 'Z (m)']
        pos_ranges = [(-1.0, 1.0), (-1.0, 1.0), (0.0, 1.5)]  # Reasonable workspace
        
        for i, (label, (min_val, max_val)) in enumerate(zip(pos_labels, pos_ranges)):
            ax_slider = plt.axes([0.55, 0.65 - i * 0.05, 0.4, 0.03])
            slider = Slider(ax_slider, label, min_val, max_val, 
                          valinit=current_pos[i], valstep=0.01)
            slider.on_changed(lambda val, idx=i: self.cartesian_position_callback(val, idx))
            self.cartesian_sliders.append(slider)
            
            # Create label for current value
            label_text = f'{label}: {current_pos[i]:.3f}'
            label = plt.figtext(0.55, 0.65 - i * 0.05 + 0.04, label_text, fontsize=10)
            self.cartesian_labels.append(label)
            
        # Orientation sliders (Roll, Pitch, Yaw)
        rpy_labels = ['Roll (°)', 'Pitch (°)', 'Yaw (°)']
        rpy_ranges = [(-180, 180), (-180, 180), (-180, 180)]
        
        for i, (label, (min_val, max_val)) in enumerate(zip(rpy_labels, rpy_ranges)):
            ax_slider = plt.axes([0.55, 0.45 - i * 0.05, 0.4, 0.03])
            slider = Slider(ax_slider, label, min_val, max_val, 
                          valinit=current_rpy[i], valstep=1.0)
            slider.on_changed(lambda val, idx=i: self.cartesian_orientation_callback(val, idx))
            self.cartesian_sliders.append(slider)
            
            # Create label for current value
            label_text = f'{label}: {current_rpy[i]:.1f}°'
            label = plt.figtext(0.55, 0.45 - i * 0.05 + 0.04, label_text, fontsize=10)
            self.cartesian_labels.append(label)
            
    def create_action_buttons(self):
        """Create action buttons for robot control"""
        # Reset button
        ax_reset = plt.axes([0.02, 0.15, 0.12, 0.05])
        self.reset_button = Button(ax_reset, 'Reset to Zero')
        self.reset_button.on_clicked(self.reset_robot)
        
        # Home button
        ax_home = plt.axes([0.16, 0.15, 0.12, 0.05])
        self.home_button = Button(ax_home, 'Home Position')
        self.home_button.on_clicked(self.home_robot)
        
        # Close button
        ax_close = plt.axes([0.30, 0.15, 0.12, 0.05])
        self.close_button = Button(ax_close, 'Close GUI')
        self.close_button.on_clicked(self.close_gui)
        
    def create_status_display(self):
        """Create status display area"""
        self.status_text = plt.figtext(0.55, 0.25, 'Status: Ready', fontsize=10, 
                                     bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue"))
        
        # Display current end-effector pose
        self.update_status_display()
        
    def mode_changed(self, label):
        """Handle control mode change"""
        self.control_mode = label.lower()
        self.update_status_display()
        print(f"Control mode changed to: {self.control_mode}")
        
    def joint_slider_callback(self, value, joint_index):
        """Handle joint slider changes"""
        if self.control_mode == "joint":
            # Convert degrees to radians
            self.robot.q[joint_index] = np.radians(value)
            
            # Update robot in environment
            self.env.step(0.01)
            
            # Update joint label
            self.labels[joint_index].set_text(f'Joint {joint_index+1}: {value:.1f}°')
            
            # Update Cartesian sliders to reflect new position
            self.update_cartesian_sliders()
            
    def cartesian_position_callback(self, value, axis_index):
        """Handle Cartesian position slider changes"""
        if self.control_mode == "cartesian":
            # Get current pose
            current_pose = self.robot.fkine(self.robot.q)
            current_pos = current_pose.t.flatten()
            
            # Update position
            current_pos[axis_index] = value
            
            # Create new target pose
            target_pose = SE3(current_pos) @ SE3(current_pose.R)
            
            # Solve inverse kinematics
            result = self.robot.ikine_LM(target_pose, q0=self.robot.q, mask=[1,1,1,0,0,0])
            
            if result.success:
                self.robot.q = result.q
                self.env.step(0.01)
                
                # Update Cartesian label
                self.cartesian_labels[axis_index].set_text(f'{["X (m)", "Y (m)", "Z (m)"][axis_index]}: {value:.3f}')
                
                # Update joint sliders to reflect new configuration
                self.update_joint_sliders()
            else:
                print(f"IK failed for position {current_pos}")
                
    def cartesian_orientation_callback(self, value, axis_index):
        """Handle Cartesian orientation slider changes"""
        if self.control_mode == "cartesian":
            # Get current pose
            current_pose = self.robot.fkine(self.robot.q)
            current_pos = current_pose.t.flatten()
            
            # Get current RPY and update the specified axis
            current_rpy = tr2rpy(current_pose.R, unit='deg')
            current_rpy[axis_index] = value
            
            # Create new rotation matrix
            new_rotation = rpy2r(np.radians(current_rpy))
            
            # Create new target pose
            target_pose = SE3(current_pos) @ SE3(new_rotation)
            
            # Solve inverse kinematics
            result = self.robot.ikine_LM(target_pose, q0=self.robot.q)
            
            if result.success:
                self.robot.q = result.q
                self.env.step(0.01)
                
                # Update Cartesian label
                rpy_labels = ['Roll (°)', 'Pitch (°)', 'Yaw (°)']
                self.cartesian_labels[3 + axis_index].set_text(f'{rpy_labels[axis_index]}: {value:.1f}°')
                
                # Update joint sliders to reflect new configuration
                self.update_joint_sliders()
            else:
                print(f"IK failed for orientation {current_rpy}")
                
    def update_joint_sliders(self):
        """Update joint sliders to reflect current robot configuration"""
        for i, slider in enumerate(self.sliders):
            slider.set_val(np.degrees(self.robot.q[i]))
            self.labels[i].set_text(f'Joint {i+1}: {np.degrees(self.robot.q[i]):.1f}°')
            
    def update_cartesian_sliders(self):
        """Update Cartesian sliders to reflect current robot configuration"""
        current_pose = self.robot.fkine(self.robot.q)
        current_pos = current_pose.t.flatten()
        current_rpy = tr2rpy(current_pose.R, unit='deg')
        
        # Update position sliders
        for i in range(3):
            self.cartesian_sliders[i].set_val(current_pos[i])
            self.cartesian_labels[i].set_text(f'{["X (m)", "Y (m)", "Z (m)"][i]}: {current_pos[i]:.3f}')
            
        # Update orientation sliders
        for i in range(3):
            self.cartesian_sliders[3 + i].set_val(current_rpy[i])
            self.cartesian_labels[3 + i].set_text(f'{["Roll (°)", "Pitch (°)", "Yaw (°)"][i]}: {current_rpy[i]:.1f}°')
            
    def reset_robot(self, event):
        """Reset robot to zero configuration"""
        self.robot.q = np.zeros(self.robot.n)
        self.env.step(0.01)
        self.update_joint_sliders()
        self.update_cartesian_sliders()
        self.update_status_display()
        print("Robot reset to zero configuration")
        
    def home_robot(self, event):
        """Move robot to home position"""
        self.robot.q = self.initial_q.copy()
        self.env.step(0.01)
        self.update_joint_sliders()
        self.update_cartesian_sliders()
        self.update_status_display()
        print("Robot moved to home position")
        
    def close_gui(self, event):
        """Close the GUI"""
        self.running = False
        try:
            plt.close(self.fig)
        except:
            pass  # Ignore errors when closing
        print("GUI closed")
        
    def update_status_display(self):
        """Update the status display with current robot information"""
        current_pose = self.robot.fkine(self.robot.q)
        current_pos = current_pose.t.flatten()
        
        status_text = f"Status: {self.control_mode.title()} Mode\n"
        status_text += f"EE Position: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]\n"
        status_text += f"Joint Config: {np.degrees(self.robot.q).round(1)}°"
        
        self.status_text.set_text(status_text)
        
    def run(self):
        """Main GUI loop"""
        print("GUI started. Use the controls to move Robot 3.")
        print("Press 'q' in the console to quit the GUI.")
        
        try:
            # Show the figure
            plt.show(block=False)
            
            # Simple event loop
            while self.running:
                # Process matplotlib events
                plt.pause(0.01)
                time.sleep(0.01)
                
                # Check if figure is still open
                if not plt.get_fignums():
                    break
                    
        except KeyboardInterrupt:
            print("GUI interrupted by user")
        except Exception as e:
            print(f"GUI error: {e}")
        finally:
            self.close_gui(None)
