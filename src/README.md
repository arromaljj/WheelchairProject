
Dependencies Needed/ Packages to Install 
DWA planner 
https://github.com/amslabtech/dwa_planner.git

SLAM Toolbox 
  https://github.com/SteveMacenski/slam_toolbox.git
  
twist_mux 
  https://github.com/ros/geometry2.git
  
laser_filters
  https://github.com/ros-perception/laser_filters.git
  
rplidar_ros 
  https://github.com/Slamtec/rplidar_ros.git

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Step by Step Guide to Wiring up the Wheelchair for Raspberry Pi and Dependencies for Software 
Requirements
Hardware                                         
- picamera                                         
- LCD screen                                       
- Raspberry Pi 4                                  
- Invacare electric wheelchair                     
- ToF sensors                                     
- Jumper cables                                    
- Wireless ps4 Controller                         
- Third-Party Interface (TPI) Module              
- Dynamic Control Linx bus cable 
- LEDs 
- PS4 Controller

  Software 
- Raspberry pi imager 
- Adafruit ToF sensor python library  
- OpenCV python library 
- Ubuntu 20.04 server OS 
- PyPS4Controller python library 
- RPi.GPIO python library 
- Adafruit Busio python library 
- Adafruit Board python library 

1. Connecting the ToF sensors :
   1. Connect the SDA (GPIO 2) and SCL (GPIO 3) pins to the breadboard
   2. Connect the 5V pin to the only one Live rail on breadboard
   3. Connect a GND pin to the only one GND rail on breadboard
   4. Solder a pin on to the XSHUT pin (If not done so already)
   5. Connect corresponding sensor pins to raspberry pi pins
   6. Connect XSHUT pin to raspberry pi GPIO pin (any GPIO pin will do excluding GPIO 2 and 3)
   7. Write python program to control the ToF sensors

2. Ubuntu 20.04 desktop
   
     1. Install Raspberry pi imager
     2. Mount SD card into computer/laptop
     3. Flash ‘ubuntu 20.04 server’ onto SD card
     4. Configure SD card manually to include username and password if needed. SSH can be enabled if       desired
     5. Mount SD card into raspberry pi
     6. Use Debian package ‘tasksel’ to install ubuntu 20.04 desktop
  
  4.  Installation of ROS Noetic

       1. Open terminal on the raspberry pi
       2. Run the commands to install ros (in order):
                a. Set up sources list – “sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu                     $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'”
                b. Set up keys - “sudo apt install curl” (if curl isn’t already installed)
                c. Set up keys cont. – “curl -s     https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add –“ 
                d. sudo apt update
                e. Main installation - sudo apt install ros-noetic-desktop-full
                f. Set up environment – source /opt/ros/noetic/setup.bash
                g.  Set up environment cont. – echo source /opt/ros/noetic/setup.bash >> ~/.bashrc
                h. source ~/.bashrc
                i. Build dependencies – sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
                j. Initialize ros – sudo apt install python3-rosdep
                k. Initialize ros cont. – sudo rosdep init
                l. Intialize ros cont. – rosdep update
  
         For Trouble shooting and additional Documentation seek:               
         https://wiki.ros.org/noetic/Installation/Ubuntu
      3. Run the commands to install catkin tools (in order):
           a. Sudo apt-get update
           b. Sudo apt-get install python3-catkin-tools
     For trouble shooting and additional Documentation for Catkin Tools seek:
    https://catkin-tools.readthedocs.io/en/latest/installing.html

6. Install Git
   
       1. Open terminal on the Raspberry Pi
       2. sudo apt-get update
       3. sudo apt-get install git-all
   
8. ROS Dependencies
   
       1. Open terminal on the Raspberry Pi
       2.a. Gazebo Sim installation:
       sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' 
       wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - 
       sudo apt-get update
   
       b.  sudo apt-get install libgz-sim<#>-dev
       3.a  ROS Navigation – select noetic-devel branch
       b. click the code button and copy the HTTPS
       c. type :  git clone https://github.com/ros-planning/navigation.git
       4.a  rplidar_sdk – select master branch
       b. Enter: git clone https://github.com/Slamtec/rplidar_sdk.git
       5.a. SLAM Toolbox – select noetic-devel :
       b. Enter : git clone https://github.com/SteveMacenski/slam_toolbox.git
       6.a twist_mux – select noetic-devel
       b. Enter: git clone https://github.com/ros-teleop/twist_mux.git
       7.a  laser_filter – select noetic-devel
       b.  Enter: git clone https://github.com/ros-perception/laser_filters.git
       8.a. geomtery2 – select noetic-devel
       b. Enter: git clone https://github.com/ros/geometry2.git
   
5. Pi Camera
   1. Open the Camera Serial Interface (Cpi)
   2. Insert the ribbon cable of camera into the port with the blue tag on the cable facing the     ethernet port
   3. Ensure the ribbon cable is straight and fully inserted before closing the CSi
   4. Enter the config.txt file in the boot folder of your raspberry pi (for example /boot/firmware/config.txt) and edit the file at the bottom to include ‘start_x=1’ 

      Disclaimer: root access will be required to edit this file
   5. Save and exit the config.txt file
   6. Reboot the pi
   7. Using OpenCV program the pi camera to display a preview screen on the display
  
6.  Wireless Controller Connection
     1. Install the ‘pi-bluetooth’ package
     2. Reboot the Raspberry Pi
     3. Turn on the wireless controller
  If already connected to another device, hold the share and the home button to start searching for new device.
      4. Access the Bluetooth settings on the raspberry pi
      5. Connect to ‘Wireless Controller’
      6. Set up the pyPS4controller code to read inputs from the wireless controller and work alongside the GPIO pins
  7. TPI
      1. Cut one end of the bus cable and strip down to a sufficient length to expose the data and power cables
      2. Solder header pins to the exposed cables
      3. Solder the header pins to the corresponding TPI J1 inputs
      4. Solder header pins on to the J2 inputs of the module
      5. Configure the TPI module using the table below: 
          
 
