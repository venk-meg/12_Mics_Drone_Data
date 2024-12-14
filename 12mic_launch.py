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

    experimentsetup = ["Normal Cap On", "Normal Cap On", "Normal Cap On","Normal Cap On", "Normal Cap On", "Normal Cap On","Normal Cap On", "Normal Cap On", "Normal Cap On","Normal Cap On", "Normal Cap On", "Normal Cap On"]

    node = Node(
        package='my_robot_controller',  
        executable='mic12_Audio_plotter',  
        name=f'audiocaptureandplotter',
        parameters=[{'deviceports': deviceports},
                    {'experimentsetup': experimentsetup}]
    )
    nodes.append(node)

    node = Node(
        package='my_robot_controller', 
        executable='mic12_CSV_logger',
        name=f'csvlogger',
        parameters=[{'numberofmics': len(deviceports)}]
    )
    nodes.append(node)

    node = Node(
        package='my_robot_controller',  
        executable='mic12_Audio_capture',  
        name=f'wavmaker',
        parameters=[{'deviceports': deviceports},
                    {'numberofmics': len(deviceports)}]
    )
    nodes.append(node)  

    return LaunchDescription(nodes)

