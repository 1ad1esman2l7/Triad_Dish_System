import numpy as np
import matplotlib.pyplot as plt
import time
import keyboard
import threading

class EmergencyStopGUI:
  def __init__(self, assessment_instance):
      self.assessment = assessment_instance
      self.emergency_active = False
      self.system_paused = False
      self.resume_requested = False
      self.running = True
      
      # Create simple GUI
      self.create_gui()
      
      # Start keyboard monitoring thread
      self.keyboard_thread = threading.Thread(target=self.monitor_keyboard, daemon=True)
      self.keyboard_thread.start()
      
  def create_gui(self):
      """Create a simple matplotlib GUI"""
      plt.close('all')
      self.fig = plt.figure(figsize=(8, 4))
      self.fig.suptitle('EMERGENCY STOP SYSTEM', fontsize=14, fontweight='bold', color='red')
      
      # Set background color
      self.fig.patch.set_facecolor('lightgray')
      
      # Create status display
      self.status_text = plt.figtext(0.5, 0.7, 'SYSTEM NORMAL', fontsize=12, fontweight='bold', 
                                    ha='center', color='green', 
                                    bbox=dict(boxstyle="round,pad=0.3", facecolor="white", edgecolor="green"))
      
      # Create instructions
      instructions = """EMERGENCY STOP SYSTEM - KEYBOARD CONTROLS:

Press 'E' to activate emergency stop
Press 'R' to request resume (after E-stop is deactivated)
Press 'C' to confirm resume (after resume is requested)
Press 'Q' to quit

⚠️  Two-step process required for safety ⚠️"""
      
      plt.figtext(0.5, 0.3, instructions, fontsize=10, ha='center', color='black', 
                  bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.8))
      
      plt.tight_layout()
      plt.show(block=False)
      
  def monitor_keyboard(self):
      """Monitor keyboard input in a separate thread"""
      while self.running:
          try:
              if keyboard.is_pressed('e') and not self.emergency_active:
                  self.activate_emergency_stop()
                  time.sleep(0.5)  # Prevent multiple triggers
                  
              elif keyboard.is_pressed('r') and self.emergency_active and not self.resume_requested:
                  self.request_resume()
                  time.sleep(0.5)
                  
              elif keyboard.is_pressed('c') and self.resume_requested:
                  self.confirm_resume()
                  time.sleep(0.5)
                  
              elif keyboard.is_pressed('q'):
                  self.close_gui()
                  break
                  
              time.sleep(0.01)  # Small delay to prevent high CPU usage
              
          except Exception as e:
              print(f"Keyboard monitoring error: {e}")
              break
              
  def activate_emergency_stop(self):
      """Activate emergency stop"""
      if not self.emergency_active:
          self.emergency_active = True
          self.system_paused = True
          self.resume_requested = False
          
          # Update GUI
          self.status_text.set_text('EMERGENCY STOP ACTIVATED')
          self.status_text.set_color('red')
          self.status_text.set_bbox(dict(boxstyle="round,pad=0.3", facecolor="white", edgecolor="red"))
          
          print("\n" + "="*60)
          print("🛑 EMERGENCY STOP ACTIVATED BY USER")
          print("🛑 ALL ROBOT OPERATIONS HALTED")
          print("🛑 SYSTEM FROZEN - Manual intervention required")
          print("="*60)
  
  def request_resume(self):
      """Request system resume (first step)"""
      if self.emergency_active and not self.resume_requested:
          self.resume_requested = True
          self.status_text.set_text('RESUME REQUESTED - Press C to confirm')
          self.status_text.set_color('orange')
          self.status_text.set_bbox(dict(boxstyle="round,pad=0.3", facecolor="white", edgecolor="orange"))
          
          print("\n" + "="*60)
          print("🔄 RESUME REQUESTED")
          print("🔄 Press 'C' to confirm system restart")
          print("="*60)
  
  def confirm_resume(self):
      """Confirm system resume (second step)"""
      if self.resume_requested:
          self.emergency_active = False
          self.system_paused = False
          self.resume_requested = False
          
          # Update GUI
          self.status_text.set_text('SYSTEM NORMAL')
          self.status_text.set_color('green')
          self.status_text.set_bbox(dict(boxstyle="round,pad=0.3", facecolor="white", edgecolor="green"))
          
          print("\n" + "="*60)
          print("✅ SYSTEM RESUME CONFIRMED")
          print("✅ EMERGENCY STOP DEACTIVATED")
          print("✅ ROBOT OPERATIONS RESUMED")
          print("="*60)
          
          return True
      return False
  
  def is_emergency_active(self):
      """Check if emergency stop is active"""
      return self.emergency_active
  
  def is_system_paused(self):
      """Check if system is paused"""
      return self.system_paused
  
  def update_gui(self):
      """Update the GUI (call this periodically from main thread)"""
      try:
          plt.pause(0.01)
      except:
          pass
  
  def close_gui(self):
      """Close the GUI"""
      try:
          self.running = False
          plt.close(self.fig)
      except:
          pass