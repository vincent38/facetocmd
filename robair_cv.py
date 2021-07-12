import cv2
import rospy as rp
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8, Int8
from cv_bridge import CvBridge, CvBridgeError
import sys
import time

"""
TODO:

    - Fix delay problem between robot and control computer (especially webcam feed)
"""

"""
This class manages the camera feed processing and the sending of commands to the robot.

Needs to be connected on the same network than the robot to work, if running on a separate platform.
Please ensure that ROS is correctly configured on both platforms in this case.
"""
class CamCtrl:
    """
    Handles the initialization of the node, the registration of the different publishers,
    and the registration on the webcam feed sent by the tablet.
    """
    def __init__(self):
        print("[INFO] Initializing class...")
        self.bridge = CvBridge()
        self.pV = rp.Publisher('verin_speed', UInt8, queue_size=10)
        self.pH = rp.Publisher('cmdhead', Int8, queue_size=10)
        self.face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
        self.s = rp.Subscriber('webcam/image_raw', Image, self.printIt)
        self.sH = rp.Subscriber('head', Int8, self.updateHead)
        self.head = 0
        rp.init_node("Test", anonymous=True)
        #self.rate = rp.Rate(10)
        print("[INFO] Init done.")

    """
    Callback for current head position setting
    """
    def updateHead(self, data):
        self.head = data.data
        print("[INFO] New head position:", self.head)

    """
    Callback used when we receive data on the subscribed topic
    """
    def printIt(self, data):
        bigw = 0
        bigh = 0

        bigx = 320
        bigy = 240
        try:
            cvi = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            exit()
        gray = cv2.cvtColor(cvi, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)
        for (x, y, w, h) in faces:
            cv2.rectangle(cvi, (x, y), (x+w, y+h), (255, 0, 0), 2)
            if w > bigw and h  > bigh:
                bigx = x + w/2
                bigy = y + h/2
        print("Want to level to person position in", bigx, bigy)
        # Right/Left on h
        if bigx < 300:
            # Left
            self.head = self.head - 5
            if self.head < -128:
                self.head = -128
        elif bigx > 340:
            # Right
            self.head = self.head + 5
            if self.head > 127:
                self.head = 127
        self.pH.publish(self.head)
        # Up/Down on h
        if bigy < 220:
            #Command up
            self.pV.publish(127)
        elif bigy > 260:
            #Command down
            self.pV.publish(129)
        self.pV.publish(128)
        cv2.imshow("Webcam feed", cvi)
        cv2.waitKey(3)
        time.sleep(2)

    """
    Main loop , checks what's happening on ros and process the callbacks and data publishing
    """
    def run(self):
        print("[INFO] Running application.")
        try:
            rp.spin()
        except KeyboardInterrupt:
            print("[INFO] Called keyboard interruption...")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    print("[INFO] Starting application...")
    try:
        app = CamCtrl()
        app.run()
    except:
        print("[ERROR] Application stopped. Reason:",sys.exc_info()[0])
        exit()