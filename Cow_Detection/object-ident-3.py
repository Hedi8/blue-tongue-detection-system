import cv2

import pyrebase

import board
import busio as io
import adafruit_mlx90614
from time import sleep

import paho.mqtt.client as mqtt

import time
from gpiozero import AngularServo

servo =AngularServo(18, initial_angle=0, min_pulse_width=0.0006, max_pulse_width=0.0023)


firebaseconfig={'apiKey': "AIzaSyDc1AW03L71zvUZ2-Ka15S_B9X5mMX5fII",
  'authDomain': "farme-ca685.firebaseapp.com",
  'databaseURL': "https://farme-ca685-default-rtdb.europe-west1.firebasedatabase.app",
  'projectId': "farme-ca685",
  'storageBucket': "farme-ca685.appspot.com",
  'messagingSenderId': "683038567999",
  'appId': "1:683038567999:web:3816116b403daf5585128a",
  'measurementId': "G-J15BMGM2P2"}

firebase = pyrebase.initialize_app(firebaseconfig)
db = firebase.database()

storage = firebase.storage()
#thres = 0.45 # Threshold to detect object
img_counter = 'A'
classNames = []
classFile = "/home/pi/Desktop/Object_Detection_Files/coco.names"
with open(classFile,"rt") as f:
    classNames = f.read().rstrip("\n").split("\n")

configPath = "/home/pi/Desktop/Object_Detection_Files/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
weightsPath = "/home/pi/Desktop/Object_Detection_Files/frozen_inference_graph.pb"

net = cv2.dnn_DetectionModel(weightsPath,configPath)
net.setInputSize(320,320)
net.setInputScale(1.0/ 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

broker_address="172.20.10.2"

def on_message(client, userdata, message ):
    print("message received" ,str(message.payload.decode("utf-8")))
    print("message topic=",message.topic)
    print("message qos=",message.qos)
    print("message retain flag=",message.retain)

def getObjects(img, thres, nms, draw=True, objects=[]):
    classIds, confs, bbox = net.detect(img,confThreshold=thres,nmsThreshold=nms)
    #print(classIds,bbox)
    if len(objects) == 0: objects = classNames
    objectInfo =[]
    if len(classIds) != 0:
        for classId, confidence,box in zip(classIds.flatten(),confs.flatten(),bbox):
            className = classNames[classId - 1]
            if className in objects: 
                objectInfo.append([box,className])
                if (draw):
                    cv2.rectangle(img,box,color=(0,255,0),thickness=2)
                    cv2.putText(img,classNames[classId-1].upper(),(box[0]+10,box[1]+30),
                    cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)
                    cv2.putText(img,str(round(confidence*100,2)),(box[0]+200,box[1]+30),
                    cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)
                    
                    servo.angle = -90
                    time.sleep = 2
                    servo.angle = 90
                    
                    i2c = io.I2C(board.SCL, board.SDA, frequency=100000)
                    mlx = adafruit_mlx90614.MLX90614(i2c)

                    ambientTemp = "{:.2f}".format(mlx.ambient_temperature)
                    targetTemp = "{:.2f}".format(mlx.object_temperature)

                    sleep(1)

                    print("Target Temperature:", targetTemp,"°C")
                    print("ambient Temperature:", ambientTemp,"°C")
                    
                    if(float(targetTemp)<30.0):
                        cv2.imwrite('image'+img_counter+'.jpg', img)
                        storage.child('vache/image'+img_counter+'.jpg').put('image'+img_counter+'.jpg')
                        data={'image':'image'+img_counter,'targetTemp':targetTemp,'ambientTemp':ambientTemp}
                        db.child("vache").push(data)
                        
                        
                        print("creating new instance")
                        client = mqtt.Client("P1")
                        client.on_message=on_message
                        print("connecting to broker")
                        client.connect(broker_address)
                        client.loop_start()
                        print("subscribing to topic","My_Topic")
                        client.subscribe("My_Topic")
                        print("publishing message to topic","house/bulbs/bulb1")
                        client.publish("My_Topic","hi from raspberry")
                        client.loop_stop
                        
    return img,objectInfo


if __name__ == "__main__":

    cap = cv2.VideoCapture(0)
    cap.set(3,640)
    cap.set(4,480)
    #cap.set(10,70)
    
    while True:
        success, img = cap.read()
        result, objectInfo = getObjects(img,0.45,0.2, objects=['cow'])
        #print(objectInfo)
        img_counter = chr(ord(img_counter)+1)
        cv2.imshow("Output",img)
        cv2.waitKey(1)
    