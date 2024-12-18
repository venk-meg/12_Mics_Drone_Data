This repository serves to provide a basic understanding of how to use this data collection framework. This repository runs in ROS2/foxy

The overall data collection framework incorporates 3 python scripts that collect different types of data and processes accordingly. 
These 3 scripts included: mic12_Audio_capture.py, mic12_CSV_logger.py and mic12_Audio_plotter. 

mic12_Audio_capture.py creates 12 publishers to publish the audio data for 12 microphones in 12 different topics called db_data1 to db_data12 every 0.1 seconds, starts 12 threads and 12 input streams to record audio data from all 12 microphones simultaneously. 
The data is stored in audio chunks within a buffer that is reset after 15 seconds intervals. 
During each buffer reset, the current chunk is saved as a temporary ‘.bin’ file which is converted to a ‘.wav’ file upon program shutdown. 

mic12_CSV_logger.py subscribes to the position of all microphones and the Tello from the VRPN client nodes, and subscribes to audio data of each microphone published to the db_dataX topic (X being the current microphone index starting from 1-12) along with timestamps of each recording package sent from the microphone, 
and then computes the decibel value for all 12 microphones and logs this value for each microphone every 0.1 seconds (duration can be changed later), into a CSV file that can later be analysed. The CSV file is also populated with the current Tello position and the distance between the Tello and each microphone every 0.1 seconds.

The final script - mic12_Audio_plotter.py subscribes to the 12 db_dataX topics and obtains the audio data corresponding to each microphone and plots it to the corresponding subplot providing live visualization of the audio levels.

These 3 scripts are launched simultaneously using a ROS2 launch file package.

tellocontroller.py used to command the drone to travel a predefined path, with move_forward(x) being move drone x cm forward - likewise for all other directions and rotate_counter_clockwise(90) being rotate 90 degrees counter clockwise and likewise for clockwise rotation.

Required Libraries/Dependencies:
ROS1/noetic
ROS2/foxy
Sounddevice
RCLPY
std_msgs
threading
scipy.io
matplotlib
geometry_msgs
csv
datetime
time
djitellopy
