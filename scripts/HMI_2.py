import tkinter as tk
import rospy
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Point  # Assuming the coordinates are published as geometry_msgs/Point
from std_msgs.msg import Float32MultiArray

class HMIApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Human Machine Interface")

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

        self.reset_button = tk.Button(self.left_frame, text="Reset", command=self.reset_process, font=("Helvetica", 12))
        self.reset_button.pack(pady=5)

        self.emergency_button = tk.Button(self.left_frame, text="Noodstop", command=self.emergency_stop, font=("Helvetica", 12), fg="red")
        self.emergency_button.pack(pady=5)

        # Middle Frame for Slider
        self.middle_frame = tk.Frame(self.main_frame)
        self.middle_frame.grid(row=0, column=1, padx=10)

        self.slider_label = tk.Label(self.middle_frame, text="Select Option", font=("Helvetica", 14))
        self.slider_label.pack(pady=10)

        self.slider = tk.Scale(self.middle_frame, from_=1, to=5, orient="horizontal", command=self.update_slider_label)
        self.slider.pack(pady=10)

        self.slider_name_box = tk.Entry(self.middle_frame, font=("Helvetica", 14))
        self.slider_name_box.pack(pady=10)

        # Right Frame for Info Box
        self.right_frame = tk.Frame(self.main_frame)
        self.right_frame.grid(row=0, column=2, padx=10)

        # Info Box
        self.info_box = tk.Text(self.right_frame, height=10, width=30, font=("Helvetica", 12))
        self.info_box.pack(pady=10)

        # Light Indicators
        self.light_frame = tk.Frame(root)
        self.light_frame.pack(pady=10)
        
        self.light1 = tk.Label(self.light_frame, text="O", font=("Helvetica", 16), fg="grey")
        self.light1.grid(row=0, column=0, padx=5)
        self.light2 = tk.Label(self.light_frame, text="O", font=("Helvetica", 16), fg="Orange")
        self.light2.grid(row=0, column=1, padx=5)
        self.light3 = tk.Label(self.light_frame, text="O", font=("Helvetica", 16), fg="grey")
        self.light3.grid(row=0, column=2, padx=5)

        # Process Variables
        self.process_running = False
        self.emergency_active = False

        # ROS Initialization
        rospy.init_node('hmi_publisher', anonymous=True)
        self.choice_publisher = rospy.Publisher('hmi_choice', Int32, queue_size=10)
        self.signal_publisher = rospy.Publisher('hmi_signal', Bool, queue_size=10)
        rospy.Subscriber('coordinates', Point, self.coordinates_callback)  # Assuming the topic 'coordinates' publishes geometry_msgs/Point
        self.xyz_publisher = rospy.Publisher('xyz_position', Float32MultiArray, queue_size=10)

        # Initialize the name box with the first option
        self.update_slider_label("1")
        
        # Display initial message
        self.update_info_box("Klaar om signaal te ontvangen.")

        # Run Tkinter main loop in a way that doesn't block ROS callbacks
        self.root.after(100, self.ros_spin)

    def start_process(self):
        if not self.process_running and not self.emergency_active:
            self.process_running = True
            self.start_button.config(state="disabled")
            self.stop_button.config(state="normal")
            self.emergency_button.config(state="normal")
            self.signal_publisher.publish(True)  # Publish start signal
            # Publish start message as a ROS message
            start_msg = Bool()
            start_msg.data = True
            self.signal_publisher.publish(start_msg)
            # Publish selected option to ROS
            self.choice_publisher.publish(self.selected_option)
            # Update lights
            self.light1.config(text="O", fg="green")
            self.light2.config(text="O", fg="grey")
            self.light3.config(text="O", fg="grey")
            # Update info box
            self.update_info_box("Proces is gestart.")

    def stop_process(self):
        if self.process_running and not self.emergency_active:
            self.process_running = False
            self.start_button.config(state="normal")
            self.stop_button.config(state="disabled")
            self.emergency_button.config(state="normal")
            self.signal_publisher.publish(False)  # Publish stop signal
            # Publish stop message as a ROS message
            stop_msg = Bool()
            stop_msg.data = False
            self.signal_publisher.publish(stop_msg)
            # Update lights
            self.light1.config(text="O", fg="grey")
            self.light2.config(text="O", fg="grey")
            self.light3.config(text="O", fg="red")
            # Update info box
            self.update_info_box("Proces is gestopt.")

    def emergency_stop(self):
        if not self.emergency_active:
            self.emergency_active = True
            self.process_running = False
            self.start_button.config(state="disabled")
            self.stop_button.config(state="disabled")
            self.emergency_button.config(state="disabled")
            self.signal_publisher.publish(False)  # Publish stop signal
            # Publish stop message as a ROS message
            stop_msg = Bool()
            stop_msg.data = False
            self.signal_publisher.publish(stop_msg)
            # Start blinking lights
            self.blink_lights()
            # Update info box
            self.update_info_box("Noodknop is ingedrukt.")

    def reset_process(self):
        # Reset the interface to initial state
        self.process_running = False
        self.emergency_active = False
        self.start_button.config(state="normal")
        self.stop_button.config(state="disabled")
        self.emergency_button.config(state="normal")
        # Publish reset signal if needed
        reset_msg = Bool()
        reset_msg.data = False
        self.signal_publisher.publish(reset_msg)
        # Reset lights
        self.light1.config(text="O", fg="grey")
        self.light2.config(text="O", fg="orange")
        self.light3.config(text="O", fg="grey")
        # Clear the info box
        self.info_box.delete(1.0, tk.END)
        # Reset slider to initial state
        self.slider.set(1)
        self.update_slider_label("1")
        # Update info box
        self.update_info_box("Interface is gereset.")
        self.update_info_box("Klaar om signaal te ontvangen.")

    def blink_lights(self):
        if self.emergency_active:
            current_color1 = self.light1.cget("fg")
            next_color1 = "red" if current_color1 == "grey" else "grey"
            current_color2 = self.light2.cget("fg")
            next_color2 = "red" if current_color2 == "grey" else "grey"
            current_color3 = self.light3.cget("fg")
            next_color3 = "red" if current_color3 == "grey" else "grey"
            
            self.light1.config(fg=next_color1)
            self.light2.config(fg=next_color2)
            self.light3.config(fg=next_color3)
            
            self.root.after(500, self.blink_lights)  # Toggle color every 500ms

    def update_slider_label(self, value):
        self.selected_option = int(value)
        # Update slider name box based on selected option
        names = {1: "Dop_10", 2: "KleineSchroevendraaier", 3: "Plattekop", 4: "Spanningstester", 5: "Alles sorteren"}
        name = names.get(self.selected_option, "Unknown")
        self.slider_name_box.delete(0, tk.END)
        self.slider_name_box.insert(0, name)

    def coordinates_callback(self, msg):
        # Update info box with the coordinates
        coordinates_message = "Coordinates received: X={}, Y={}, Z={}".format(msg.x, msg.y, msg.z)
        self.update_info_box(coordinates_message)

    def update_info_box(self, message):
        # Append a message to the info box
        self.info_box.insert(tk.END, message + "\n")
        # Automatically scroll to the bottom
        self.info_box.see(tk.END)

    def ros_spin(self):
        # Allow ROS to process incoming messages
        rospy.rostime.wallsleep(0.1)
        # Call this method again after 100 ms
        self.root.after(100, self.ros_spin)

if __name__ == "__main__":
    root = tk.Tk()
    app = HMIApp(root)
    root.mainloop()
