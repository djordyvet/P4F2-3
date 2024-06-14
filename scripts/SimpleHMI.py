# -*- coding: utf-8 -*-
import Tkinter as tk
import rospy
from std_msgs.msg import Int32, Bool, Float32

class HMIApp:
    def __init__(self, root):
        self.root = root
        self.root.title("HMI with Counter, Buttons, Slider, and Coordinates")

        # Counter
        self.counter_value = 0
        self.counter_label = tk.Label(root, text="Counter: " + str(self.counter_value), font=("Helvetica", 16))
        self.counter_label.pack(pady=10)

        # Buttons
        self.start_button = tk.Button(root, text="Start", command=self.start_process, font=("Helvetica", 12))
        self.start_button.pack(pady=5)

        self.stop_button = tk.Button(root, text="Stop", command=self.stop_process, font=("Helvetica", 12), state="disabled")
        self.stop_button.pack(pady=5)

        self.reset_button = tk.Button(root, text="Reset", command=self.reset_process, font=("Helvetica", 12))
        self.reset_button.pack(pady=5)

        # Slider
        self.slider_label = tk.Label(root, text="Select Option", font=("Helvetica", 14))
        self.slider_label.pack(pady=10)

        self.slider = tk.Scale(root, from_=1, to=5, orient="horizontal", command=self.update_slider_label)
        self.slider.pack(pady=10)

        self.selected_option = 1
        self.selected_option_label = tk.Label(root, text="Selected Option: 1", font=("Helvetica", 14))
        self.selected_option_label.pack(pady=10)

        # Light Indicator
        self.light_indicator = tk.Label(root, text="Idle", font=("Helvetica", 20), fg="grey")
        self.light_indicator.pack(pady=10)

        # Coordinates Text Boxes
        self.coord_frame = tk.Frame(root)
        self.coord_frame.pack(pady=10)
        self.x_coord_label = tk.Label(self.coord_frame, text="X Coordinate:", font=("Helvetica", 12))
        self.x_coord_label.grid(row=0, column=0, padx=5)
        self.x_coord_text = tk.Entry(self.coord_frame, font=("Helvetica", 12))
        self.x_coord_text.grid(row=0, column=1, padx=5)
        self.y_coord_label = tk.Label(self.coord_frame, text="Y Coordinate:", font=("Helvetica", 12))
        self.y_coord_label.grid(row=1, column=0, padx=5)
        self.y_coord_text = tk.Entry(self.coord_frame, font=("Helvetica", 12))
        self.y_coord_text.grid(row=1, column=1, padx=5)

        # Process Variables
        self.process_running = False
        self.process_id = None

        # ROS Initialization
        rospy.init_node('hmi_publisher', anonymous=True)
        self.choice_publisher = rospy.Publisher('hmi_choice', Int32, queue_size=10)
        self.signal_publisher = rospy.Publisher('hmi_signal', Bool, queue_size=10)
        rospy.Subscriber('coordinates', Float32, self.coordinates_callback)
        
        # Run Tkinter main loop in a way that doesn't block ROS callbacks
        self.root.after(100, self.ros_spin)

    def start_process(self):
        if not self.process_running:
            self.process_running = True
            self.start_button.config(state="disabled")
            self.stop_button.config(state="normal")
            self.signal_publisher.publish(True)  # Publish start signal
            self.process_id = self.root.after(1000, self.update_counter)
            # Publish start message as a ROS message
            start_msg = Bool()
            start_msg.data = True
            self.signal_publisher.publish(start_msg)
            # Update light indicator to Running
            self.light_indicator.config(text="Running", fg="green")

    def stop_process(self):
        if self.process_running:
            self.process_running = False
            self.start_button.config(state="normal")
            self.stop_button.config(state="disabled")
            self.signal_publisher.publish(False)  # Publish stop signal
            if self.process_id:
                self.root.after_cancel(self.process_id)
            # Publish stop message as a ROS message
            stop_msg = Bool()
            stop_msg.data = False
            self.signal_publisher.publish(stop_msg)
            # Update light indicator to Stopped
            self.light_indicator.config(text="Stopped", fg="red")

    def reset_process(self):
        self.counter_value = 0
        self.counter_label.config(text="Counter: " + str(self.counter_value))
        # Update light indicator to Idle
        self.light_indicator.config(text="Idle", fg="grey")

    def update_counter(self):
        if self.process_running:
            self.counter_value += 1
            self.counter_label.config(text="Counter: " + str(self.counter_value))
            self.process_id = self.root.after(1000, self.update_counter)

    def update_slider_label(self, value):
        self.selected_option = int(value)
        self.selected_option_label.config(text="Selected Option: " + str(value))

        # Update light indicator color based on selected option
        if self.selected_option == 1:
            self.light_indicator.config(fg="red")
        elif self.selected_option == 2:
            self.light_indicator.config(fg="yellow")
        elif self.selected_option == 3:
            self.light_indicator.config(fg="green")
        elif self.selected_option == 4:
            self.light_indicator.config(fg="blue")
        elif self.selected_option == 5:
            self.light_indicator.config(fg="purple")

        # Publish selected option to ROS
        self.choice_publisher.publish(self.selected_option)

    def coordinates_callback(self, msg):
        # Assume msg is a tuple or a similar structure with x and y attributes
        self.x_coord_text.delete(0, tk.END)
        self.x_coord_text.insert(0, str(msg.x))
        self.y_coord_text.delete(0, tk.END)
        self.y_coord_text.insert(0, str(msg.y))

    def ros_spin(self):
        # Allow ROS to process incoming messages
        rospy.rostime.wallsleep(0.1)
        # Call this method again after 100 ms
        self.root.after(100, self.ros_spin)

if __name__ == "__main__":
    root = tk.Tk()
    app = HMIApp(root)
    root.mainloop()
