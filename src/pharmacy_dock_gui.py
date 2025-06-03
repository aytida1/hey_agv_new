#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk
import threading
from concurrent.futures import ThreadPoolExecutor

from nav2_msgs.action import DockRobot
from geometry_msgs.msg import PoseStamped


class PharmacyDockGUI(Node):
    def __init__(self):
        super().__init__('pharmacy_dock_gui')
        self._action_client = ActionClient(self, DockRobot, '/dock_robot')
        self.get_logger().info('Pharmacy Dock GUI initialized')
        
        # Track current operation
        self.current_operation = None
        self.operation_complete = threading.Event()
        
        # Create GUI
        self.setup_gui()
        
        # Thread pool for handling actions
        self.thread_pool = ThreadPoolExecutor(max_workers=1)

    def setup_gui(self):
        """Setup the GUI window and buttons"""
        self.root = tk.Tk()
        self.root.title("Pharmacy Dock Control")
        self.root.geometry("500x400")
        self.root.configure(bg='#f0f0f0')
        
        # Title label
        title_label = tk.Label(
            self.root, 
            text="Pharmacy Dock Control System", 
            font=('Arial', 16, 'bold'),
            bg='#f0f0f0',
            fg='#333333'
        )
        title_label.pack(pady=20)
        
        # Button frame
        button_frame = tk.Frame(self.root, bg='#f0f0f0')
        button_frame.pack(pady=20)
        
        # Create 4 dock buttons
        self.dock_buttons = []
        for i in range(1, 5):
            btn = tk.Button(
                button_frame,
                text=f"Pharmacy Dock {i}",
                font=('Arial', 12, 'bold'),
                width=15,
                height=2,
                bg='#4CAF50',
                fg='white',
                command=lambda dock_num=i: self.dock_button_clicked(dock_num)
            )
            btn.grid(row=(i-1)//2, column=(i-1)%2, padx=10, pady=10)
            self.dock_buttons.append(btn)
        
        # Status display
        status_frame = tk.Frame(self.root, bg='#f0f0f0')
        status_frame.pack(pady=20, padx=20, fill='x')
        
        tk.Label(
            status_frame, 
            text="Status:", 
            font=('Arial', 12, 'bold'),
            bg='#f0f0f0'
        ).pack(anchor='w')
        
        # Status text widget with scrollbar
        status_container = tk.Frame(status_frame)
        status_container.pack(fill='both', expand=True)
        
        self.status_text = tk.Text(
            status_container,
            height=8,
            font=('Courier', 10),
            bg='#ffffff',
            fg='#333333',
            state='disabled'
        )
        
        scrollbar = tk.Scrollbar(status_container)
        scrollbar.pack(side='right', fill='y')
        self.status_text.pack(side='left', fill='both', expand=True)
        
        self.status_text.config(yscrollcommand=scrollbar.set)
        scrollbar.config(command=self.status_text.yview)
        
        # Clear status button
        clear_btn = tk.Button(
            self.root,
            text="Clear Status",
            font=('Arial', 10),
            bg='#ff9800',
            fg='white',
            command=self.clear_status
        )
        clear_btn.pack(pady=10)

    def dock_button_clicked(self, dock_number):
        """Handle dock button clicks"""
        if self.current_operation is not None:
            self.update_status(f"Operation already in progress. Please wait...", 'warning')
            return
        
        # Disable all buttons during operation
        self.set_buttons_enabled(False)
        
        # Start docking operation in background
        self.thread_pool.submit(self.perform_docking, dock_number)

    def perform_docking(self, dock_number):
        """Perform the docking operation"""
        dock_id = f"pharmacist_dock{dock_number}"
        dock_type = "pharmacist_docks"
        
        self.current_operation = dock_number
        self.operation_complete.clear()
        
        self.update_status(f"Initiating docking to {dock_id}...", 'info')
        
        try:
            # Send docking goal
            future = self.send_goal(
                dock_id=dock_id,
                dock_type=dock_type,
                navigate_to_staging_pose=True
            )
            
            if future is not None:
                # Wait for completion
                rclpy.spin_until_future_complete(self, future)
                
                # Give some time for result processing
                import time
                start_time = time.time()
                while time.time() - start_time < 2.0:
                    rclpy.spin_once(self, timeout_sec=0.1)
            else:
                self.update_status(f"Failed to send docking goal to {dock_id}", 'error')
                
        except Exception as e:
            self.update_status(f"Error during docking to {dock_id}: {str(e)}", 'error')
        
        finally:
            # Re-enable buttons
            self.current_operation = None
            self.operation_complete.set()
            self.root.after(0, lambda: self.set_buttons_enabled(True))

    def send_goal(self, dock_id='', dock_type='', dock_pose=None, navigate_to_staging_pose=True):
        """Send docking goal to the action server"""
        goal_msg = DockRobot.Goal()
        goal_msg.dock_id = dock_id
        goal_msg.dock_type = dock_type
        goal_msg.navigate_to_staging_pose = navigate_to_staging_pose
        goal_msg.use_dock_id = True
        
        if dock_pose is not None:
            goal_msg.dock_pose = dock_pose
        else:
            goal_msg.dock_pose = PoseStamped()
            goal_msg.dock_pose.header.frame_id = 'map'
            goal_msg.dock_pose.header.stamp = self.get_clock().now().to_msg()
        
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.update_status('Action server not available!', 'error')
            return None
        
        self.update_status(f'Sending docking goal: {dock_id} ({dock_type})', 'info')
        
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        return send_goal_future

    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.update_status('Goal rejected by server', 'error')
            return
        
        self.update_status('Goal accepted by server', 'success')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Handle action result"""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            if result.success:
                self.update_status(f'✅ Docking completed successfully!', 'success')
            else:
                error_msg = self.get_error_message(result.error_code)
                self.update_status(f'❌ Docking failed: {error_msg}', 'error')
        elif status == 5:  # ABORTED
            error_msg = self.get_error_message(result.error_code)
            self.update_status(f'❌ Docking aborted: {error_msg}', 'error')
        elif status == 6:  # CANCELED
            self.update_status(f'⚠️ Docking canceled', 'warning')
        else:
            self.update_status(f'⚠️ Docking finished with status: {status}', 'warning')

    def feedback_callback(self, feedback_msg):
        """Handle action feedback"""
        feedback = feedback_msg.feedback
        state_name = self.get_state_name(feedback.state)
        
        self.update_status(f'State: {state_name}', 'info')
        
        if feedback.num_retries > 0:
            self.update_status(f'Retries: {feedback.num_retries}', 'warning')

    def get_state_name(self, state_code):
        """Convert state code to human readable name"""
        states = {
            0: "NONE",
            1: "NAVIGATING_TO_STAGING_POSE", 
            2: "INITIAL_PERCEPTION",
            3: "CONTROLLING",
            4: "WAIT_FOR_CHARGE",
            5: "RETRY"
        }
        return states.get(state_code, f"UNKNOWN_STATE_{state_code}")

    def get_error_message(self, error_code):
        """Convert error code to human readable message"""
        errors = {
            0: "No error",
            901: "Dock not found in database",
            902: "Dock not valid", 
            903: "Failed to reach staging pose",
            904: "Failed to detect dock",
            905: "Failed to control robot during docking",
            906: "Failed to charge",
            999: "Unknown error"
        }
        return errors.get(error_code, f"Error code {error_code}")

    def update_status(self, message, msg_type='info'):
        """Update status display"""
        import datetime
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        
        # Color coding
        colors = {
            'info': '#333333',
            'success': '#4CAF50', 
            'warning': '#ff9800',
            'error': '#f44336'
        }
        
        def _update():
            self.status_text.config(state='normal')
            self.status_text.insert('end', f"[{timestamp}] {message}\n")
            
            # Color the last line
            last_line_start = self.status_text.index("end-2l linestart")
            last_line_end = self.status_text.index("end-2l lineend")
            
            tag_name = f"color_{msg_type}_{timestamp.replace(':', '')}"
            self.status_text.tag_add(tag_name, last_line_start, last_line_end)
            self.status_text.tag_config(tag_name, foreground=colors.get(msg_type, '#333333'))
            
            self.status_text.config(state='disabled')
            self.status_text.see('end')
        
        # Schedule GUI update in main thread
        self.root.after(0, _update)

    def clear_status(self):
        """Clear the status display"""
        self.status_text.config(state='normal')
        self.status_text.delete(1.0, 'end')
        self.status_text.config(state='disabled')

    def set_buttons_enabled(self, enabled):
        """Enable/disable all dock buttons"""
        state = 'normal' if enabled else 'disabled'
        for btn in self.dock_buttons:
            btn.config(state=state)

    def run_gui(self):
        """Run the GUI main loop"""
        def ros_spin():
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
        
        # Start ROS spinning in background thread
        ros_thread = threading.Thread(target=ros_spin, daemon=True)
        ros_thread.start()
        
        self.update_status("Pharmacy Dock GUI ready", 'success')
        
        # Run GUI main loop
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            pass

    def shutdown(self):
        """Cleanup when shutting down"""
        self.thread_pool.shutdown(wait=True)
        self.destroy_node()


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        gui = PharmacyDockGUI()
        gui.run_gui()
    except KeyboardInterrupt:
        pass
    finally:
        if 'gui' in locals():
            gui.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()