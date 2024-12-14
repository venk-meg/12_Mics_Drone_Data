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
        executable='12mic_Audio_plotter',  
        name=f'audiocaptureandplotter',
        parameters=[{'deviceports': deviceports},
                    {'experimentsetup': experimentsetup}]
    )
    nodes.append(node)

    node = Node(
        package='my_robot_controller', 
        executable='12mic_CSV_logger',
        name=f'csvlogger',
        parameters=[{'numberofmics': len(deviceports)}]
    )
    nodes.append(node)

    node = Node(
        package='my_robot_controller',  
        executable='12mic_Audio_capture',  
        name=f'wavmaker',
        parameters=[{'deviceports': deviceports},
                    {'numberofmics': len(deviceports)}]
    )
    nodes.append(node)  

    return LaunchDescription(nodes)

