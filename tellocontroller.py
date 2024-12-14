from djitellopy import Tello 
import time

if __name__ == '__main__':
    try:
        # Connect to Tello
        tello = Tello()
        tello.connect()
        print(f"battery = {tello.get_battery()}%")
        tello.takeoff()
        tello.move_up(50)
        time.sleep(0.1)
        tello.move_forward(500)
        time.sleep(0.1)
        tello.rotate_counter_clockwise(90)
        time.sleep(0.1)
        tello.move_forward(100)
        time.sleep(0.1)
        tello.rotate_counter_clockwise(90)
        time.sleep(0.1)
        tello.move_forward(250)
        time.sleep(0.1)
        tello.rotate_counter_clockwise(90)
        time.sleep(0.1)
        tello.move_forward(260)
        time.sleep(0.1)
        tello.rotate_clockwise(90)
        time.sleep(0.1)
        tello.move_forward(300)
        time.sleep(0.1)
        tello.rotate_clockwise(90)
        time.sleep(0.1)
        tello.move_forward(150)
        time.sleep(0.1)
        tello.rotate_clockwise(90)
        time.sleep(0.1)
    except KeyboardInterrupt:
        tello.land()
        tello.end()