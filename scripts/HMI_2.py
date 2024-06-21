# -*- coding: utf-8 -*-
import Tkinter as tk
import rospy
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Point  # Assuming the coordinates are published as geometry_msgs/Point

class HMIApp:
    def __init__(self, root):
        self.root = root
        self.root.title("HMI with Slider, Buttons, Coordinates, and Camera View")

        # Create a frame for the entire layout
        self.main_frame = tk.Frame(root)
        self.main_frame.pack(pady=10)

        # Left Frame for Buttons
        self.left_frame = tk.Frame(self.main_frame)
        self.left_frame.grid(row=0, column=0, padx=10)

        self.start_button = tk.Button(self.left_frame, text="Start", command=self.start_process, font=("Helvetica", 12))
        self.start_button.pack(pady=5)

        self.stop_button = tk.Button(self.left_frame, text="Stop", command=self.stop_process, font=("Helvetica", 12), state="disabled")
        self.stop_button.pack(pady=5)

        # Light Indicators
        self.status_indicator = tk.Label(root, text="Idle", font=("Helvetica", 20), fg="grey")
        self.status_indicator.pack(pady=10)

        self.light_frame = tk.Frame(root)
        self.light_frame.pack(pady=10)
        
        self.light1 = tk.Label(self.light_frame, text="Light 1", font=("Helvetica", 16), fg="grey")
        self.light1.grid(row=0, column=0, padx=5)
        self.light2 = tk.Label(self.light_frame, text="Light 2", font=("Helvetica", 16), fg="grey")
        self.light2.grid(row=0, column=1, padx=5)
        self.light3 = tk.Label(self.light_frame, text="Light 3", font=("Helvetica", 16), fg="grey")
        self.light3.grid(row=0, column=2, padx=5)

        # Process Variables
        self.process_running = False

        # Set initial light colors
        self.update_light_colors("idle")

        # ROS Initialization
        rospy.init_node('hmi_publisher', anonymous=True)
        self.choice_publisher = rospy.Publisher('hmi_choice', Int32, queue_size=10)
        self.signal_publisher = rospy.Publisher('hmi_signal', Bool, queue_size=10)
        rospy.Subscriber('coordinates', Point, self.coordinates_callback)  # Assuming the topic 'coordinates' publishes geometry_msgs/Point

        # Run Tkinter main loop in a way that doesn't block ROS callbacks
        self.root.after(100, self.ros_spin)

    def start_process(self):
        if not self.process_running:
            self.process_running = True
            self.start_button.config(state="disabled")
            self.stop_button.config(state="normal")
            self.signal_publisher.publish(True)  # Publish start signal
            # Publish selected option to ROS
            self.choice_publisher.publish(self.selected_option)
            # Update status indicator to Running
            self.status_indicator.config(text="Running", fg="green")
            # Update lights
            self.update_light_colors("running")

    def stop_process(self):
        if self.process_running:
            self.process_running = False
            self.start_button.config(state="normal")
            self.stop_button.config(state="disabled")
            self.signal_publisher.publish(False)  # Publish stop signal
            # Update status indicator to Stopped
            self.status_indicator.config(text="Stopped", fg="red")
            # Update lights
            self.update_light_colors("stopped")

    def update_slider_label(self, value):
        self.selected_option = int(value)

    def coordinates_callback(self, msg):
        pass

    def ros_spin(self):
        # Allow ROS to process incoming messages
        rospy.rostime.wallsleep(0.1)
        # Call this method again after 100 ms
        self.root.after(100, self.ros_spin)

    def update_light_colors(self, status):
        if status == "idle":
            self.light1.config(fg="grey")
            self.light2.config(fg="grey")
            self.light3.config(fg="orange")
        elif status == "running":
            self.light1.config(fg="green")
            self.light2.config(fg="grey")
            self.light3.config(fg="grey")
        elif status == "stopped":
            self.light1.config(fg="grey")
            self.light2.config(fg="red")
            self.light3.config(fg="grey")

if __name__ == "__main__":
    root = tk.Tk()
    app = HMIApp(root)
    root.mainloop()
