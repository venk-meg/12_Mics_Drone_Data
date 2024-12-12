from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import sounddevice as sd

def generate_launch_description():
    launch_description = LaunchDescription()    
    nodes = []

    deviceports = []
    deviceslist = sd.query_devices()
    for index, device in enumerate(deviceslist):
        if 'Lavalier' in device['name']:
            deviceports.append(index)

    experimentsetup = ["Muffled Foam", "Muffled Cotton", "Muffled Plastic","Muffled", "Reflected", "None","Muffled", "Reflected", "None","Muffled", "Reflected", "None"]

    node = Node(
        package='my_robot_controller',  
        executable='mic_audio_capture_test12',  
        name=f'audiocaptureandplotter',
        parameters=[{'deviceports': deviceports},
                    {'experimentsetup': experimentsetup}]
    )
    nodes.append(node)

    node = Node(
        package='my_robot_controller', 
        executable='csv_logger_12',
        name=f'csvlogger',
    )
    nodes.append(node)

    node = Node(
        package='my_robot_controller',  
        executable='db_wav_threads_buffer',  
        name=f'wavmaker',
        parameters=[{'deviceports': deviceports},
                    {'numberofmics': len(deviceports)}]
    )
    nodes.append(node)  

    return LaunchDescription(nodes)

