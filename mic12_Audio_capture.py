import sounddevice as sd
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Float32
import numpy as np
from threading import Lock
import datetime
import threading
from scipy.io import wavfile
import os
import time
# Capture and publish audio data to csv_logger, sound

class AudioPublisher(Node):
    def __init__(self, sample_rate = 44100, duration = .1):
        super().__init__('audio_publisher')

        #Get launh parameters
        self.declare_parameter('deviceports')
        self.declare_parameter('numberofmics')
        self.device_ports = self.get_parameter('deviceports').value
        self.numberofmics = self.get_parameter('numberofmics').value
        self.sample_rate = sample_rate
        self.output_filename = "audiocaptures/audiocaptures_12/withthreads/output_audio_normal_nocaps50"
        self.buffertime = 15 #buffer time

        # initialize variables
        self.db_pubs = []
        self.timestamp_pubs = []
        self.micthreads = []
        self.audiostreams = []
        self.audiodata_lock = []
        self.dbdata_lock = []
        self.audiodata = []
        self.dbdata = [[] for _ in self.device_ports]
        self.timestamps = ["empty" for _ in self.device_ports]

        #create publishers and populate data
        for i in range(len(self.device_ports)):
            pub = self.create_publisher(Float32MultiArray, f'db_data{i}',10)
            tpub = self.create_publisher(String, f'timestamp{i}',10)
            self.db_pubs.append(pub)
            self.timestamp_pubs.append(tpub)
            self.audiodata_lock.append(Lock())
            self.dbdata_lock.append(Lock())
            self.audiodata.append(np.full((self.sample_rate * self.buffertime, 1), None))
            # self.dbdata.append([])

        self.write_position = [0] * len(self.device_ports)
        self.buffer_num = [1]*len(self.device_ports)

        #start threading
        for i in range(len(self.device_ports)):
            threadt = threading.Thread(target=self.startrec, args=(self.device_ports[i],i))
            time.sleep(0.1)
            threadt.start()
            self.micthreads.append(threadt)

        self.timer = self.create_timer(duration, self.publish_db_data)
    
    def startrec(self, deviceport, deviceportindex):
        audio_stream = sd.InputStream(
            device=deviceport,
            channels=1,
            samplerate=self.sample_rate,
            blocksize=105, #factor of 44100, keeping it fixed to not lose any samples
            callback=lambda indata, frames, time, status: self.audio_callback(deviceportindex, indata, frames, time, status)
        )
        audio_stream.start()

        self.audiostreams.append(audio_stream)

        # Wait for stream to get active
        while not audio_stream.active:
           self.get_logger().info(f"audio stream for device{deviceportindex} ntemp_files_wavot working")
           pass

    def audio_callback(self, deviceportindex,indata, frames, time, status):
        
        # update indata to db data list
        with self.dbdata_lock[deviceportindex]:
            self.dbdata[deviceportindex].append(indata)
            
        if  self.timestamps[deviceportindex] == "empty":
                # print(self.get_clock().now())
                self.timestamps[deviceportindex] = str(self.get_clock().now().to_msg())
                # self.get_logger().info(f"{self.timestamps[deviceportindex]}, {type(self.timestamps[deviceportindex])}")

        # update indata to audio data list
        with self.audiodata_lock[deviceportindex]:
            start_idx = self.write_position[deviceportindex]
            end_idx = start_idx + frames
            self.audiodata[deviceportindex][start_idx:end_idx] = indata
            self.write_position[deviceportindex] = end_idx

            if self.write_position[deviceportindex] >= self.audiodata[deviceportindex].shape[0]:
                self.save_reset_buffer(deviceportindex)
    

    def save_reset_buffer(self, deviceportindex):
        if self.audiodata[deviceportindex] is None:
                self.get_logger().info(f"No audio received from mic {deviceportindex+1}")
                return
        try:
            #save buffer data in temporary .bin file
            output_file = f"{self.output_filename}_{deviceportindex+1}_{self.buffer_num[deviceportindex]}_TEMP.bin"
            with open(output_file, 'ab') as f:
                f.write(self.audiodata[deviceportindex].astype(np.float32).tobytes())
                f.flush()
            self.buffer_num[deviceportindex] += 1
        except Exception as e:
            self.get_logger().error(f"Failed to save audio for mic {deviceportindex+1}: {e}")

        #Reset buffer data
        self.audiodata[deviceportindex] = np.full((self.sample_rate * self.buffertime, 1), None)
        self.write_position[deviceportindex] = 0
  

    def publish_db_data(self):
        #publish data 
        for i in range(len(self.device_ports)):
            msg = Float32MultiArray()
            with self.dbdata_lock[i]:
                if len(self.dbdata[i]) > 0: 
                    msg.data = np.vstack(self.dbdata[i]).flatten().tolist()
                else:
                    self.get_logger().info(f"audio data for mic{i+1} not obtained, {datetime.datetime.now()}")
            #publish messages
            self.dbdata[i] = []
            self.db_pubs[i].publish(msg)

            tmsg = String()
            # tmsg.data = float(self.timestamps[i].sec + self.timestamps[i].nanosec * 1e-9) if self.timestamps[i] is not None else 0.0
            tmsg.data = self.timestamps[i]

            self.timestamps[i] = "empty"
            self.timestamp_pubs[i].publish(tmsg)   

        
    def savewav(self):
        for i in range(len(self.audiodata_lock)):
            with self.audiodata_lock[i]:
                self.save_reset_buffer(i)
                try:
                    currentdateandtime = datetime.datetime.now()
                    output_file = f"{self.output_filename}_{currentdateandtime}_{i+1}.wav"

                    temp_files = [f"{self.output_filename}_{i + 1}_{buffer_idx}_TEMP.bin" for buffer_idx in range(1, self.buffer_num[i])]
                    
                    for buffer_idx in range(1, self.buffer_num[i]):
                        self.binary_to_wav(i, buffer_idx)

                    temp_files_wav = [f"{self.output_filename}_{i + 1}_{buffer_idx}_TEMP.wav" for buffer_idx in range(1, self.buffer_num[i])]
                    self.concatenate_and_save_wav(temp_files_wav, output_file)
                    sd.wait()
                    self.delete_temp_files((temp_files + temp_files_wav))
                except Exception as e:
                    self.get_logger().error(f"Failed to save final audio for mic {i+1}: {e}")

    #concatenate temp .wav files into .wav file and save
    def concatenate_and_save_wav(self, temp_files, output_file):
        all_audio_data = []
        for temp_file in temp_files:
            _, audio_data = wavfile.read(temp_file)
            all_audio_data.append(audio_data)
        concatenated_audio_data = np.concatenate(all_audio_data, axis=0)
        wavfile.write(output_file, self.sample_rate, concatenated_audio_data)

    #delete all temp files
    def delete_temp_files(self, temp_files):
        for file in temp_files:
            if os.path.exists(file):
                os.remove(file)

    #convert .bin files to .wav files
    def binary_to_wav(self, deviceportindex, buffer_num):
        try:
            input_file = f"{self.output_filename}_{deviceportindex+1}_{buffer_num}_TEMP.bin"
            output_file = f"{self.output_filename}_{deviceportindex+1}_{buffer_num}_TEMP.wav"
            with open(input_file, 'rb') as f:
                audio_data = np.frombuffer(f.read(), dtype=np.float32)
            wavfile.write(output_file, self.sample_rate, audio_data)
        except Exception as e:
            self.get_logger().error(f"Failed to convert binary to WAV for mic {deviceportindex+1}: {e}")
    
    #close all streams and join threads to save wav file
    def delete(self):
        for i in range(len(self.audiodata_lock)):
            self.audiostreams[i].stop()

        #join threads here before saving any wavs
        for threadt in self.micthreads:
            threadt.join()

        self.savewav()

        for i in range(len(self.audiodata_lock)):
            self.audiostreams[i].close()
        self.get_logger().info(f"Audio stream stopped")


def main():
    rclpy.init(args=None)
    audio_publisher = AudioPublisher()
    try:
        rclpy.spin(audio_publisher)
    except KeyboardInterrupt:
        audio_publisher.get_logger().info("Shutting down publisher.")
    finally:
        audio_publisher.delete()
        audio_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()