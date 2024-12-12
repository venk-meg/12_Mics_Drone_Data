#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import sounddevice as sd

class AudioPlotter(Node):
    def __init__(self):
        super().__init__('audiosubnode')
        self.declare_parameter('deviceports')
        self.declare_parameter('experimentsetup')
        self.device_indices = self.get_parameter('deviceports').value
        experimentsetup = self.get_parameter('experimentsetup').value

        self.audio_subs = []
        self.audio12data = [[] for _ in self.device_indices]

        # Initialize plots
        self.fig, self.axs = plt.subplots(3, 4, figsize=(16, 16))
        self.axs = self.axs.flatten()

        # Set up each subplot
        self.lines = []
        for i, ax in enumerate(self.axs[:len(self.device_indices)]): 
            ax.set_title(f"Microphone Audio {i + 1} - {experimentsetup[i]}")
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Amplitude')
            ax.set_ylim(-1, 1)
            line, = ax.plot([], [], lw=2) 
            self.lines.append(line)

        # Adjust subplot layout
        plt.subplots_adjust(left=0.1, right=0.95, top=0.95, bottom=0.1, wspace=0.4, hspace=0.4)
        plt.show(block = False)

        self.create_timer(0.5, self.plotgraph) 

        # Create subscribers
        for i in range(len(self.device_indices)):
            sub = self.create_subscription(
                Float32MultiArray,
                f'db_data{i}',
                lambda msg, aud=i: self.audiocallback(msg, aud),
                10
            )
            self.audio_subs.append(sub)

    #populate audio data for all mics
    def audiocallback(self, msg, i):
        if msg.data:
            self.audio12data[i] = list(msg.data)

    def plotgraph(self, duration = 1 ):
        for i, audiodata in enumerate(self.audio12data):
            if audiodata:
                audio_data = np.array(audiodata[-int(duration * 1000):]) 
                x = np.linspace(0, duration, len(audio_data))
                self.lines[i].set_data(x, audio_data)
                self.axs[i].relim()
                self.axs[i].autoscale_view(scaley=True)  

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main():
    rclpy.init(args=None)
    audioplot = AudioPlotter()
    try:
        rclpy.spin(audioplot)
    except KeyboardInterrupt:
        print("Closing Plots")
    finally:
        audioplot.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
