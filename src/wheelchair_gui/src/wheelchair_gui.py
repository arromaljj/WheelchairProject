#!/usr/bin/env python

import tkinter as tk
from PIL import Image, ImageTk
import cv2
import numpy as np
import threading
import pygame.mixer
import math

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseFeedback, MoveBaseGoal, MoveBaseResult



### {1} -- Code for my designated Hexangonal shaped EMERGENCY STOP button ###
class HexagonalButton(tk.Canvas):
    def __init__(self, master, text, command, **kwargs):
        tk.Canvas.__init__(self, master, **kwargs)
        self.text = text
        self.command = command
        self.bind("<Button-1>", self.on_click)
        self.draw_hexagon()
      

    def draw_hexagon(self):
        width = self.winfo_reqwidth() * 0.9
        height = self.winfo_reqheight() * 0.9
        side_length = min(width, height) / 2 

        # Coordinates for a regular hexagon
        hexagon_coords = [
            (width / 2, 2),
            (width - side_length / 2, height / 4),
            (width - side_length /2, 3 * height / 4),
            (width / 2, height),
            (side_length /2, 3 * height / 4),
            (side_length /2, height / 4)
        ]

        self.create_polygon(hexagon_coords, outline="black", fill="red", width=2 )

        # Add text to the center of the hexagon
        self.create_text(width / 2, height / 2, text=self.text, font=('Helvetica', '8', 'bold'))

    def on_click(self, event):
        if self.command:
            self.command()

### {1} ###
            

### {2} -- Code for my actual GUI for the LCD Touchscreen ###
class WheelchairGUI:
    def __init__(self, master):
        self.master = master
        master.title("Wheelchair Control")

        # Create header
        self.header = tk.Frame(master)
        self.header.pack(fill=tk.X)

        # Create pygame mixer
        pygame.mixer.init()

        ### {3} -- Code to Create my buttons on the screen ###

        ### Code for Mapping Button ###
        self.button1 = tk.Button(master, text="Mapping & Environment", command=lambda: self.show_content("MAPPING & ENVIRONMENT"), bg="#ADD8E6", fg="black", font=('Helvetica', '10', 'normal'), relief=tk.RAISED)
        
        ### Code for Reversing Camera Button ###
        self.button2 = tk.Button(master, text="Reversing Camera", command=lambda: self.show_content("REVERSING CAMERA"), bg="#90EE90", fg="black", font=('Helvetica', '10', 'normal'), relief=tk.RAISED)
        ### Possible code to display the reversing camera footage when the button is pressed ###
        # Create standard REVERSING CAMERA button
        #self.button2 = tk.Button(master, text="Reversing Camera", command=self.start_camera_thread, bg="#008080", fg="black", font=('Helvetica', '10', 'normal'), relief=tk.RAISED)
        #self.button2.place(x=275, y=10)

        ### Code for Horn Button ###
        self.button4 = tk.Button(master, text="Horn/Buzzer", command=lambda: self.show_content("HORN"), bg="yellow", fg="black", font=('Helvetica', '10', 'normal'), relief=tk.RAISED)
        ### Possible code to play audio file when HORN button is pressed ###
        # Create standard HORN button
        #self.button4 = tk.Button(master, text="Horn", command=self.play_horn_sound, bg="yellow", fg="black", font=('Helvetica', '10', 'normal'), relief=tk.RAISED)
        #self.button4.place(x=30, y=320)

        # Code for Creating a hexagonal EMERGENCY STOP button ###
        self.button3 = HexagonalButton(master, text="EMG STOP!", command=lambda: self.show_content("EMERGENCY STOP ACTIVATED!"), width=110, height=80)
        self.button3.place(x=10, y=320)
        ### Possible code to play audio file when EMG STOP is pressed ###
         # Create hexagonal button
        #self.button3 = HexagonalButton(master, text="EMG STOP!", command=self.play_emergency_sound, width=60, height=40)
        #self.button3.place(x=10, y=320)

        
        # Create the Centre screen content area
        self.content = tk.Text(master, height=12, width=48, state=tk.DISABLED)

        # Create touch button banner along the bottom
        self.bottom_banner = tk.Frame(master)
        self.bottom_banner.pack(side=tk.BOTTOM, fill=tk.X)

        # Create buttons for the bottom banner
        self.menu_button = tk.Button(self.bottom_banner, text="Menu", command=self.show_menu)
        self.settings_button = tk.Button(self.bottom_banner, text="Settings", command=self.show_settings)
        self.about_button = tk.Button(self.bottom_banner, text="About", command=self.show_about)
        self.exit_button = tk.Button(self.bottom_banner, text="Exit", command=self.show_exit)

        # Place buttons on the screen
        self.button1.place(x=10, y=10)
        self.button2.place(x=275, y=10)
        self.button3.place(x=275, y=300)
        self.button4.place(x=30, y=320)

        # Place content area in the center
        self.content.place(x=5, y=75)

        # Place buttons on the bottom banner
        self.menu_button.pack(side=tk.LEFT)
        self.settings_button.pack(side=tk.LEFT)
        self.about_button.pack(side=tk.LEFT)
        self.exit_button.pack(side=tk.LEFT)

        # Variables for reversing camera
        self.camera = cv2.VideoCapture(0)  # Change the index if your camera is not the default
        self.is_camera_running = False
        self.camera_thread = None

    ### Code to ensure the camera resources are released properly ###
    def on_close(self):
        # Release the camera resources properly
        self.is_camera_running = False
        if self.camera_thread:
            self.camera_thread.join()  # Wait for the camera thread to finish
        if self.camera.isOpened():
            self.camera.release()

        # Destroy the Tkinter window
        self.master.destroy()

    ### Code to display the text in the Central Screen ###
    def show_content(self, button_text):
        # Display content in the central screen
        self.content.config(state=tk.NORMAL)
        self.content.delete("1.0", tk.END)
        self.content.insert(tk.END, f"Selected: {button_text}\n")
        self.content.config(state=tk.DISABLED)
        self.center_text()

    ### Code to centre align the text ###
    def center_text(self):
        text_width = int(self.content.cget("width"))
        lines = self.content.get("1.0", tk.END).split("\n")
        for line in lines:
           line_width = self.content.winfo_reqwidth()  # Measure the line width
           if line_width < text_width:
               padding = (text_width - line_width) // 2
               self.content.tag_configure("center", justify='center')
               self.content.insert(tk.END, f"\n{line}", "center")
        self.content.config(state=tk.DISABLED)

    ### Code to play audio sounds when buttons are pressed ###
    def play_horn_sound(self):
        # Load and play the horn sound
        pygame.mixer.music.load("path/to/your/horn_sound.mp3")  # Replace with the actual path to your sound file
        pygame.mixer.music.play()

    def play_emergency_sound(self):
        # Load and play the emergency sound
        pygame.mixer.music.load("/path/to/your/emergency_sound.mp3")  # Replace with the actual path to your EMG STOP sound file
        pygame.mixer.music.play()

    ### Code for the buttons along the bottom menu tab ### 
    def show_menu(self):

        moveBaseGoalClient.send_goal(GoalPose1)

        # Handle the "Menu" button click
        print("Menu button clicked")

    def show_settings(self):
        # Handle the "Settings" button click
        print("Settings button clicked")

    def show_about(self):
        # Handle the "About" button click
        print("About button clicked")

    def show_exit(self):
        # Handle the "Exit" button click
        print("Exit button clicked")
        self.on_close()
        self.master.destroy()

    ### Code for starting the camera thread ###
    def start_camera_thread(self):
        if not self.is_camera_running:
            self.is_camera_running = True
            self.camera_thread = threading.Thread(target=self.update_camera, daemon=True)
            self.camera_thread.start()

    ### Code for updating the camera feed ###
    def update_camera(self):
        while self.is_camera_running:
            ret, frame = self.camera.read()
            if ret:
                # Resize the frame to fit the central screen
                frame = cv2.resize(frame, (400, 300))

                # Convert the frame from BGR to RGB
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                # Convert the frame to ImageTk format
                tk_frame = ImageTk.PhotoImage(Image.fromarray(rgb_frame))

                # Update the content area with the camera frame
                self.content.configure(image=tk_frame)
                self.content.image = tk_frame
            else:
                print("Error reading frame from the camera.")




if __name__ == "__main__":
    rospy.init_node("wheelchair_GUI", anonymous=False)

    moveBaseGoalClient = actionlib.SimpleActionClient('/move_base',MoveBaseAction)
    moveBaseGoalClient.wait_for_server()

    GoalPose1 = MoveBaseGoal()
    GoalPose1.target_pose.header.frame_id = 'map'
    GoalPose1.target_pose.pose.position.x = 4.5
    GoalPose1.target_pose.pose.position.y = 5.0
    GoalPose1.target_pose.pose.position.z = 0.0
    GoalPose1.target_pose.pose.orientation.x = 0.0
    GoalPose1.target_pose.pose.orientation.y = 0.0
    GoalPose1.target_pose.pose.orientation.z = 0.0
    GoalPose1.target_pose.pose.orientation.w = 0.1


    GoalPose2 = MoveBaseGoal()
    GoalPose2.target_pose.header.frame_id = 'map'
    GoalPose2.target_pose.pose.position.x = 0.0
    GoalPose2.target_pose.pose.position.y = -1.0
    GoalPose2.target_pose.pose.position.z = 0.0
    GoalPose2.target_pose.pose.orientation.x = 0.0
    GoalPose2.target_pose.pose.orientation.y = 0.0
    GoalPose2.target_pose.pose.orientation.z = 0.0
    GoalPose2.target_pose.pose.orientation.w = 0.1

    GoalPose3 = MoveBaseGoal()
    GoalPose3.target_pose.header.frame_id = 'map'
    GoalPose3.target_pose.pose.position.x = 1.0
    GoalPose3.target_pose.pose.position.y = 0.5
    GoalPose3.target_pose.pose.position.z = 0.0
    GoalPose3.target_pose.pose.orientation.x = 0.0
    GoalPose3.target_pose.pose.orientation.y = 0.0
    GoalPose3.target_pose.pose.orientation.z = 0.0
    GoalPose3.target_pose.pose.orientation.w = 0.1

    root = tk.Tk()
    app = WheelchairGUI(root)
    root.geometry("420x420")
    root.protocol("WM_DELETE_WINDOW", app.on_close)  # Close the camera properly when the window is closed
    root.mainloop()