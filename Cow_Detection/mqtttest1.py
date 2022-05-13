import paho.mqtt.client as mqtt
import time

def on_message(client, userdata, message ):
    print("message received" ,str(message.payload.decode("utf-8")))
    print("message topic=",message.topic)
    print("message qos=",message.qos)
    print("message retain flag=",message.retain)

broker_address="192.168.1.177"

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
time.sleep(4)
client.loop_stop