from flask import Flask, Response
import threading
import cv2
import time
import numpy as np
from picamera2 import Picamera2
from PCA9685 import PCA9685
import signal
import sys
from handDetector import handDetector
from PID import PID
import queue


# Initialize Flask app
app = Flask(__name__)

# Global variables to store the latest frame and a lock for thread-safe access
latest_frame = None
frame_lock = threading.Lock()

##############################################################################
# Global variables to keep track of angles
angles= [60,120]
errors = [0,0, -np.infty]
end_angles= [100,100]
center= np.array([320,250])
stop_threads = False
signal_handled = False
last_known_positions = None
hand_in_frame = False






# Setup Picamera2 and PCA9685
picam2 = Picamera2()
picam2.start()

pwm = PCA9685()
pwm.setPWMFreq(50)

detector = handDetector()



def hand_detection_and_video_stream():
    global errors, latest_frame, stop_threads, last_known_positions, hand_in_frame
    pTime = 0
    seqid = 0
    while not stop_threads:
        img = picam2.capture_array()
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        img = cv2.flip(img, 0)
        img = detector.findHands(img)
        lmlist =np.array( detector.findPosition(img))
        if len(lmlist) != 0:
            hand_in_frame = True
            #c = np.array([np.mean(lmlist[:, 1]), np.mean(lmlist[:, 2])])
            c = np.array([lmlist[8, 1], lmlist[8, 2]])
            last_known_positions = c
            tmp= c - np.array([320, 250])
            errors[0] = tmp[0]
            errors[1] = tmp[1]
            errors[2] = seqid
            seqid += 1
        else: hand_in_frame = False

        if not hand_in_frame and last_known_positions is not None:
            if last_known_positions[0] > 370:  # Hand left to the right
                errors[0] = 200  # Move right
            elif last_known_positions[0] < 270:  # Hand left to the left
                errors[0] = -200  # Move left
            else: errors[0] = 0
            if last_known_positions[1] > 300:  # Hand left to the bottom
                errors[1] = 40  # Move down
            elif last_known_positions[1] < 200:  # Hand left to the top
                errors[1] = -40  # Move up
            else: errors[1] = 0
            errors[2] = seqid
            seqid += 1
            

        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime
        cv2.circle(img, (320, 250), 50, (0, 255, 0), 2)
        cv2.putText(img, str(int(fps)), (10, 70), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)
        # Store the latest frame with thread safety
        with frame_lock:
            latest_frame = img
        # Add a small delay to reduce CPU usage
        time.sleep(0.01)


def pwm_control_loop(channel = None):
    # channel = 0 for x, 1 for y
    if channel is None: ValueError("Channel must be specified")
    pwm.moveToAngle(channel=channel, start_angle=end_angles, end_angle=angles, steps=100, delay=0.01)
    print(f"pwm channel {channel} loop starting...")
    moveto_angle = angles[channel]
    steps = 100
    pid = PID(0.005, 0.000001, 0.0000001)
    pid.initialize()
    seqid = 0
    while not stop_threads:
        if np.abs(errors[channel]-center[channel]) > 2 and seqid!= errors[2]:
            seqid = errors[2]
            angle = angles[channel]
            correction = pid.update(errors[channel])
            moveto_angle = angle + np.min(correction)
            print(f"does something, error : {correction}")
            if (np.abs(correction)> 1):
                steps =int( 10* np.abs(correction))
            else: 
                steps = 20
            delay = 1/int(np.sqrt(steps)) * 0.01
            if pwm.moveToAngle(channel= channel, start_angle=angle, end_angle=moveto_angle, steps=steps, delay=delay):
                angles[channel] = moveto_angle
        else:
            seqid = errors[2]
            time.sleep(0.1)





def generate_frames():
    global latest_frame
    while True:
        with frame_lock:
            if latest_frame is None:
                with open('Three_Raspberries.jpg', 'rb') as f:
                    frame = f.read()
            else:
                # Draw a dot in the center of the frame
                height, width, _ = latest_frame.shape
                center_coordinates = (width // 2, height // 2)
                radius = 5
                color = (0, 0, 255)  # Red color in BGR
                thickness = -1  # Solid circle
                cv2.circle(latest_frame, center_coordinates, radius, color, thickness)

                ret, buffer = cv2.imencode('.jpg', latest_frame)
                frame = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        
        
@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == "__main__":
    try:
        threads = []
        # Start the video processing thread
        video_thread = threading.Thread(target=hand_detection_and_video_stream)
        video_thread.daemon = True
        video_thread.start()
        threads.append(video_thread)
        # Start the PWM in seperate threads to allow concurrent movements
        pwm_thread1 = threading.Thread(target=pwm_control_loop, args=(0,))
        pwm_thread2 = threading.Thread(target=pwm_control_loop, args=(1,))
        pwm_thread1.daemon = True
        pwm_thread2.daemon = True
        threads.append(pwm_thread1)
        threads.append(pwm_thread2)
        pwm_thread1.start()
        pwm_thread2.start()
    

        app.run(host='2a02:8388:17c0:6980:50c7:42b8:8754:8ca5', port=5000)
        while True:
            time.sleep(100)
    except (KeyboardInterrupt, SystemExit):
        print('Received keyboard interrupt, quitting threads.')
        stop_threads = True
        for thread in threads:
            thread.join()
        picam2.stop()
        pwm.moveToAngle(channel=0, start_angle=angles, end_angle=end_angles, steps=200, delay=0.0005)
        pwm.moveToAngle(channel=1, start_angle=angles, end_angle=end_angles, steps=400, delay=0.0005)
 